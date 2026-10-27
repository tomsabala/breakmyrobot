# Copyright 2026 Resilient Autonomy Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
The gateway's honesty rules, as tests.

Each case here is a way the dashboard could show a number that was never
measured: a no-return drawn as a wall, an unknown twist drawn as a standstill,
a manager string paraphrased, a broken run directory hidden from the index, or
a fault request whose pose does not match the one a scenario would have sent.
Nothing here tests wiring.
"""

import json
import math
import os
import subprocess
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ral_eval.dashboard_api import (
    Broadcaster,
    Gateway,
    _floats,
    _pid_namespace,
    build_run_index,
    encode_estimate,
    encode_fault_event,
    encode_mission,
    encode_scan,
    encode_truth,
)
from ral_eval.frames import quaternion_of_yaw, yaw_of_quaternion
from ral_eval.run_scenario import DISPATCH
from ral_interfaces.msg import FaultEvent, MissionState
from sensor_msgs.msg import LaserScan


def test_encode_scan_maps_every_no_return_to_null():
    """+inf, NaN, -inf and out-of-window ranges are null, finite ones exact."""
    scan = LaserScan()
    scan.range_min = 0.15
    scan.range_max = 12.0
    scan.ranges = [1.25, float('inf'), float('nan'), float('-inf'), 0.05, 12.5, 11.999]

    ranges = encode_scan(scan)['ranges']

    assert ranges[0] == 1.25
    assert ranges[1] is None, '+inf is a no-return, not a 12 m wall'
    assert ranges[2] is None, 'NaN is a no-return'
    assert ranges[3] is None
    assert ranges[4] is None, 'below range_min is outside the sensor window'
    assert ranges[5] is None, 'above range_max is outside the sensor window'
    # LaserScan.ranges is float32; the encoder widens the stored value and
    # must not round it, so the expectation is the message's own storage.
    assert ranges[6] == scan.ranges[6]
    assert abs(ranges[6] - 11.999) < 1e-5
    # A null must survive JSON: the console draws a point only where the value
    # is not null, and NaN would not round-trip.
    assert json.dumps(ranges, allow_nan=False).count('null') == 5


def test_encode_truth_never_reports_an_unknown_twist_as_zero():
    """covariance[0] == -1.0 means unknown; 0.0 would state "stationary"."""
    msg = Odometry()
    msg.header.frame_id = 'world'
    msg.pose.pose.orientation.w = 1.0
    msg.twist.covariance[0] = -1.0
    msg.twist.twist.linear.x = 0.0

    twist = encode_truth(msg)['twist']

    assert twist['known'] is False
    assert twist['linear_x'] is None
    assert twist['angular_z'] is None

    msg.twist.covariance[0] = 0.0
    msg.twist.twist.linear.x = 0.37
    msg.twist.twist.angular.z = -0.2
    twist = encode_truth(msg)['twist']
    assert twist == {'linear_x': 0.37, 'angular_z': -0.2, 'known': True}


def test_encode_truth_reports_the_pose_in_the_publisher_s_frame():
    """The frame is the message's, and yaw is read off the quaternion."""
    msg = Odometry()
    msg.header.frame_id = 'world'
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 500000000
    msg.pose.pose.position.x = 3.25
    msg.pose.pose.position.y = -1.5
    qx, qy, qz, qw = quaternion_of_yaw(1.2)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    payload = encode_truth(msg)

    assert payload['frame'] == 'world'
    assert payload['stamp_sim_s'] == 12.5
    assert payload['pose']['x'] == 3.25
    assert payload['pose']['y'] == -1.5
    assert math.isclose(payload['pose']['yaw'], 1.2, abs_tol=1e-9)


def test_encode_estimate_has_a_covariance_trace_only_where_one_exists():
    """AMCL publishes a covariance; the car's EKF3 publishes none."""
    amcl = PoseWithCovarianceStamped()
    amcl.pose.pose.orientation.w = 1.0
    amcl.pose.covariance[0] = 0.05
    amcl.pose.covariance[7] = 0.04
    amcl.pose.covariance[35] = 0.01
    payload = encode_estimate(amcl, 'AMCL (nav2_amcl) over /scan', 'map')
    assert math.isclose(payload['covariance_trace'], 0.10)
    assert payload['label'] == 'AMCL (nav2_amcl) over /scan'
    assert payload['frame'] == 'map'

    ekf = Odometry()
    ekf.pose.pose.orientation.w = 1.0
    # A filled covariance block on an Odometry is NOT a pose uncertainty this
    # platform publishes; reading a trace off it would invent one.
    ekf.pose.covariance[0] = 0.5
    payload = encode_estimate(ekf, 'ArduPilot EKF3 via MAVROS', 'ekf_local')
    assert payload['covariance_trace'] is None
    assert payload['frame'] == 'ekf_local'


def test_encode_fault_event_reproduces_every_field_verbatim():
    """A manager string is evidence; paraphrasing one loses the evidence."""
    detail = ("gz created static box 'blocker_1' (1.200 x 0.200 x 0.600 m) at world\n"
              '(6.000, 3.000, z=0.300 set to half the box height so it rests on the '
              'floor); the lidar sees it from the next sweep')
    msg = FaultEvent()
    msg.stamp.sec = 41
    msg.stamp.nanosec = 250000000
    msg.id = 'spawn_obstacle_ui_1'
    msg.kind = FaultEvent.KIND_SPAWN_OBSTACLE
    msg.injection_point = '/world/ral_apartment/create (gz UserCommands)'
    msg.target = 'blocker_1'
    msg.state = FaultEvent.STATE_ACTIVATED
    msg.detail = detail

    payload = encode_fault_event(msg)

    assert payload == {
        'stamp_sim_s': 41.25,
        'id': 'spawn_obstacle_ui_1',
        'kind': 'spawn_obstacle',
        'injection_point': '/world/ral_apartment/create (gz UserCommands)',
        'target': 'blocker_1',
        'state': 'activated',
        'detail': detail,
    }
    assert '\n' in payload['detail'], 'a multi-line detail keeps its newlines'


def test_encode_mission_keeps_an_empty_outcome_empty():
    """`outcome` is empty until terminal; '' must not become a placeholder."""
    msg = MissionState()
    msg.stamp.sec = 9
    msg.experiment_id = 'ui_roomba_20261026T101500Z'
    msg.state = MissionState.STATE_IDLE
    msg.goal.header.frame_id = 'map'
    qx, qy, qz, qw = quaternion_of_yaw(1.57)
    msg.goal.pose.position.x = 10.0
    msg.goal.pose.position.y = 6.0
    msg.goal.pose.orientation.x = qx
    msg.goal.pose.orientation.y = qy
    msg.goal.pose.orientation.z = qz
    msg.goal.pose.orientation.w = qw
    msg.detail = 'stack up, goal not yet sent'

    payload = encode_mission(msg)

    assert payload['outcome'] == ''
    assert payload['state'] == 'idle'
    assert payload['experiment_id'] == 'ui_roomba_20261026T101500Z'
    assert payload['goal']['frame_id'] == 'map'
    assert payload['goal']['x'] == 10.0
    assert math.isclose(payload['goal']['yaw_rad'], 1.57, abs_tol=1e-9)


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as handle:
        handle.write(text)


def test_build_run_index_keeps_broken_and_old_runs_and_flags_them(tmp_path):
    """
    A directory the index cannot read is a RESULT, not a reason to hide it.

    Three directories: a complete v5 run, one whose run.json never got
    written, and one recorded under metrics_version 4. All three appear; the
    second is `partial` with the reason, and the third keeps its data while
    being marked unscorable against today's vocabulary.
    """
    runs = str(tmp_path)
    good = os.path.join(runs, 'baseline_roomba_20261025T133934Z')
    _write(os.path.join(good, 'scenario.yaml'),
           'experiment_id: baseline_roomba\nrobot: roomba\nworld: ral_apartment\nseed: 1\n')
    _write(os.path.join(good, 'env.json'), json.dumps(
        {'image_ref': 'sha256:abc', 'git_sha': 'a41c9e0', 'git_dirty': False,
         'map_sha256': '47d1e2ad', 'robot': 'roomba', 'world': 'ral_apartment', 'seed': 1}))
    _write(os.path.join(good, 'run.json'), json.dumps(
        {'exit_code': 0, 'mission_state': 'succeeded', 'outcome': 'nav2:STATUS_SUCCEEDED',
         'detail': 'from /navigate_to_pose status 4', 't_accept_sim': 30.214,
         't_terminal_sim': 67.952, 'started_utc': '2026-10-25T13:39:34Z',
         'finished_utc': '2026-10-25T13:41:00Z', 'world_to_map': [0.5, 0.5, 0.0],
         'world_to_ekf_local': None}))
    _write(os.path.join(good, 'metrics.json'), json.dumps(
        {'meta': {'metrics_version': 5}, 'metrics': {}}))

    broken = os.path.join(runs, 'obstacle_roomba_20261025T144510Z')
    _write(os.path.join(broken, 'scenario.yaml'), 'experiment_id: obstacle_roomba\n')

    old = os.path.join(runs, 'baseline_roomba_20261023T101500Z')
    _write(os.path.join(old, 'scenario.yaml'), 'experiment_id: baseline_roomba\n')
    _write(os.path.join(old, 'run.json'), json.dumps(
        {'exit_code': 0, 'mission_state': 'succeeded', 'outcome': 'STATUS_SUCCEEDED'}))
    _write(os.path.join(old, 'metrics.json'), json.dumps(
        {'meta': {'metrics_version': 4}, 'metrics': {}}))

    index = {entry['id']: entry for entry in build_run_index(runs)}

    assert len(index) == 3
    complete = index['baseline_roomba_20261025T133934Z']
    assert complete['partial'] is False
    assert complete['scorable'] is True
    assert complete['outcome'] == 'nav2:STATUS_SUCCEEDED'
    assert complete['prov']['git_sha'] == 'a41c9e0'
    assert complete['sim'] == [30.214, 67.952]

    missing = index['obstacle_roomba_20261025T144510Z']
    assert missing['partial'] is True
    assert 'run.json' in missing['note']
    assert missing['scenario'] == 'obstacle_roomba'

    stale = index['baseline_roomba_20261023T101500Z']
    assert stale['partial'] is False
    assert stale['version'] == 4
    assert stale['scorable'] is False, 'version 4 outcomes carry no runner: prefix'
    assert stale['outcome'] == 'STATUS_SUCCEEDED', 'its data is kept, not discarded'


def test_fault_requests_are_built_by_the_runner_s_own_builders():
    """
    The dashboard cannot ask for something a scenario could not.

    The builders are run_scenario's, and `fault_id` is empty because the
    manager mints the id. The obstacle yaw must be the same quaternion the
    scenario path produces, or the box would be spawned at a different
    orientation from the browser than from a run.
    """
    params = {'name': 'blocker_1', 'pose': {'x': 6.0, 'y': 3.0, 'yaw': 0.7},
              'size': {'x': 1.2, 'y': 0.2, 'z': 0.6}}
    request = DISPATCH['spawn_obstacle'][2](params, '')

    assert request.fault_id == '', 'an empty id is what makes the manager mint one'
    assert request.name == 'blocker_1'
    assert (request.pose.position.x, request.pose.position.y) == (6.0, 3.0)
    assert math.isclose(yaw_of_quaternion(request.pose.orientation), 0.7, abs_tol=1e-12)
    expected = quaternion_of_yaw(0.7)
    assert request.pose.orientation.z == expected[2]
    assert request.pose.orientation.w == expected[3]
    assert (request.size.x, request.size.y, request.size.z) == (1.2, 0.2, 0.6)

    kidnap = DISPATCH['kidnap'][2]({'pose': {'x': 2.0, 'y': 6.5, 'yaw': -1.1}}, '')
    assert kidnap.fault_id == ''
    assert math.isclose(yaw_of_quaternion(kidnap.new_pose.orientation), -1.1, abs_tol=1e-12)


def test_a_json_body_with_whole_numbers_still_builds_a_request():
    """
    JSON has ONE number type, and rosidl has two.

    A browser that sends 6.0 serialises it as `6` - `JSON.stringify(6.0)` is
    the string "6" - and Python decodes that as an int, which rosidl refuses
    outright: the operator gets no fault and an assertion instead of a
    refusal from the manager. MEASURED from the console: `AssertionError: The
    'x' field must be of type 'float'`. The scenario path never meets this
    because YAML keeps 6.0 a float, so only the dashboard boundary can
    restore it.
    """
    body = json.loads('{"name": "blocker_1", "pose": {"x": 6, "y": 3, "yaw": 0},'
                      ' "size": {"x": 1, "y": 2, "z": 1}}')
    assert isinstance(body['pose']['x'], int), 'the premise: JSON lost the float'

    request = DISPATCH['spawn_obstacle'][2](_floats(body), '')

    assert request.pose.position.x == 6.0
    assert request.size.x == 1.0
    assert request.size.z == 1.0

    # enabled is a BOOL, and bool is a subclass of int: coercing it to 1.0
    # would turn a scan_degradation disable into an invalid request.
    scan = _floats(json.loads('{"enabled": false, "range_noise_stddev_m": 0,'
                              ' "max_range_m": 12, "blind_sector_start_deg": 0,'
                              ' "blind_sector_end_deg": 0, "dropout_probability": 1}'))
    assert scan['enabled'] is False
    built = DISPATCH['scan_degradation'][2](scan, '')
    assert built.enabled is False
    assert built.max_range_m == 12.0
    assert built.dropout_probability == 1.0


class _FakeChild:
    """A `ros2 launch` process that can be made to exit on command."""

    def __init__(self, pid=424242):
        self.pid = pid
        self.returncode = None
        self.killed = False

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = self.returncode if self.returncode is not None else 0
        return self.returncode

    def exit_with(self, code):
        self.returncode = code


class _FakeTelemetry:
    """The fields stack_payload reads; nothing in these cases publishes."""

    def __init__(self):
        self.bringup_seen = {}
        self.bringup_at_wall = {}
        self.bumper_available = None
        self.bumper_reason = 'no stack in this test'

    def detach_profile(self):
        """Match TelemetryNode's interface; there is nothing to detach."""


def _gateway(tmp_path):
    """
    Build a Gateway with no ROS and no child, for the state-machine cases.

    __init__ builds a TelemetryNode and an rclpy executor, neither of which
    these cases touch; `__new__` plus the fields the lifecycle actually reads
    keeps the test to the state machine it is about.
    """
    gateway = Gateway.__new__(Gateway)
    gateway.repo_root = str(tmp_path)
    gateway.runs_dir = os.path.join(str(tmp_path), 'runs')
    gateway.lock = threading.RLock()
    gateway._epoch = 0
    gateway._orphan_pgid = None
    gateway._state_path = os.path.join(str(tmp_path), 'logs', 'stack_state.json')
    gateway._audit_path = os.path.join(str(tmp_path), 'logs', 'stack_lifecycle.jsonl')
    gateway._rtf_recent = []
    gateway.state = 'idle'
    gateway.detail = ''
    gateway.robot = None
    gateway.seed = 1
    gateway.spawn = None
    gateway.started_wall = None
    gateway.started_wall_utc = None
    gateway.wall_limit_s = None
    gateway.log_path = None
    gateway.prepared = False
    gateway.prepare_record = {}
    gateway.launch = None
    gateway.launch_log = None
    gateway.profile = None
    gateway.mission_node = None
    gateway.driver = None
    gateway.worker = None
    gateway.bringup_expected = []
    gateway._worker_stop = threading.Event()
    gateway.mission = None
    gateway.pending_mission = None
    gateway.cancel_requested = False
    gateway._stop = threading.Event()
    gateway.out = Broadcaster()
    gateway.telemetry = _FakeTelemetry()
    return gateway


def test_a_superseded_worker_may_not_write_the_stack_state(tmp_path):
    """
    The wedge this guard exists for.

    A bring-up abandoned by a stop keeps running until its own deadline and
    then writes `failed` - or worse, `ready` - over a gateway that has been
    idle for minutes. The operator then sees a stack that does not exist and
    every control acts on nothing.
    """
    gateway = _gateway(tmp_path)
    gateway._epoch = 1                       # a start happened
    gateway._epoch = 2                       # ... and then a stop superseded it

    gateway._set_state('ready', 'stack up and prepared to accept a goal', epoch=1)

    assert gateway.state == 'idle', 'the stale worker wrote state it no longer owns'
    # The same write from the CURRENT epoch is accepted: the guard is about
    # ownership, not about forbidding writes.
    gateway._set_state('ready', 'stack up', epoch=2)
    assert gateway.state == 'ready'


def test_a_child_that_exits_on_its_own_becomes_crashed_and_is_reported(tmp_path):
    """`ready` with a dead child is a lie the operator cannot see through."""
    gateway = _gateway(tmp_path)
    child = _FakeChild()
    gateway.launch = child
    gateway.robot = 'car'
    gateway.started_wall = time.monotonic() - 42.0
    gateway.state = 'ready'

    gateway._watchdog_tick()
    assert gateway.state == 'ready', 'a live child must not be reported as crashed'

    child.exit_with(137)                     # SIGKILL from a second shell
    gateway._watchdog_tick()

    assert gateway.state == 'crashed'
    assert '137' in gateway.detail, gateway.detail
    assert gateway._worker_stop.is_set(), 'the service loop must be told to stop'


def test_a_second_stop_while_stopping_is_refused_and_tears_nothing_down(tmp_path):
    """
    Two stops used to run the whole teardown twice.

    The second one walked a half-cleared gateway and left the state machine at
    `stopping -> ready -> idle` for a single operator action.
    """
    gateway = _gateway(tmp_path)
    child = _FakeChild()
    gateway.launch = child
    gateway.state = 'stopping'

    answer = gateway.stop_stack()

    assert answer['accepted'] is False
    assert 'already in progress' in answer['message']
    assert gateway.state == 'stopping', 'the refused stop changed the state'
    assert child.returncode is None, 'the refused stop tore the child down'


def test_a_live_stack_from_a_previous_gateway_is_adopted_and_can_be_stopped(tmp_path):
    """
    The wedge: a gateway restart used to leave the old stack unreachable.

    Start refused on the old stack's /clock and Stop answered "no stack is
    running", so the only way out was a shell. The handle on disk names the
    process GROUP, which is what the kill ladder can address.
    """
    gateway = _gateway(tmp_path)
    # A stand-in for `ros2 launch`: its own session, so it owns a process
    # group exactly as the real child does.
    child = subprocess.Popen(['sleep', '300'], start_new_session=True)
    os.makedirs(os.path.dirname(gateway._state_path), exist_ok=True)
    with open(gateway._state_path, 'w') as handle:
        json.dump({'epoch': 1, 'pid_ns': _pid_namespace(), 'state': 'ready',
                   'robot': 'roomba', 'seed': 1, 'spawn': {'x': 0.5, 'y': 0.5,
                                                           'yaw': 0.0},
                   'pid': child.pid, 'pgid': os.getpgid(child.pid),
                   'started_wall_utc': '2026-10-26T00:00:00Z',
                   'log_path': '/ws/logs/stack_roomba.log'}, handle)
    try:
        gateway.adopt_existing_stack()

        assert gateway.state == 'orphaned', 'a live previous stack was not adopted'
        assert str(child.pid) in gateway.detail
        assert gateway.robot == 'roomba'

        answer = gateway.stop_stack()

        assert answer['accepted'] is True
        assert gateway.state == 'idle'
        assert child.poll() is not None, 'the adopted process group outlived the stop'
        assert not os.path.exists(gateway._state_path), 'the handle outlived the stop'
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=10)


def test_a_handle_from_another_pid_namespace_is_never_signalled(tmp_path):
    """
    A container's pgid means nothing in the next container.

    scripts/dashboard.sh runs each gateway in its own container, so the pid
    recorded by the previous one names a DIFFERENT process here - or none.
    Adopting it would make Stop signal an unrelated process group.
    """
    gateway = _gateway(tmp_path)
    own_group = os.getpgid(os.getpid())
    os.makedirs(os.path.dirname(gateway._state_path), exist_ok=True)
    with open(gateway._state_path, 'w') as handle:
        json.dump({'epoch': 1, 'pid_ns': 'pid:[4026531999]', 'state': 'ready',
                   'robot': 'car', 'pid': os.getpid(), 'pgid': own_group,
                   'started_wall_utc': '2026-10-26T00:00:00Z',
                   'log_path': '/ws/logs/stack_car.log'}, handle)

    gateway.adopt_existing_stack()

    assert gateway.state == 'idle', 'a handle from another namespace was adopted'
    assert gateway._orphan_pgid is None
    assert not os.path.exists(gateway._state_path)
