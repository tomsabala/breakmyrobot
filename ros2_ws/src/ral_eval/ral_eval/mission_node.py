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
The ONE ROS node a mission is driven through, whoever is driving it.

`ral_eval.run_scenario` drives a scenario with it; `ral_eval.dashboard_api`
drives an operator-issued mission with it. Both get the same subscriptions,
the same publishers and the same QoS, which is the point: a MissionDriver is
written against this surface (`sim_now`, `spin_wall`, `spin_sim`, `stop`,
`cmd_pub`, `publish_initial_pose`, `amcl`, `converged_sim`, `wtm`, `grid`),
not against `rclpy.node.Node`, and a dashboard that reimplemented any part of
it would be driving a different robot than the harness measures.

`publish_mission_state` is the single gate on the mission vocabulary: an
outcome outside `mission_driver.OUTCOMES` raises rather than reaching a bag or
a dashboard. `publish_fault_rejected` is the single FaultEvent a mission
driver is entitled to publish - the fault manager owns every other state,
because only it observes the underlying call.

Only one thread may spin a given MissionNode instance.
"""

import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from ral_eval.frames import quaternion_of_yaw
from ral_eval.mission_driver import OUTCOMES, outcome_is_known
from ral_interfaces.msg import FaultEvent, MissionState
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import LaserScan


def injection_point(kind: str, world: str) -> str:
    """Return the literal gz service/topic or ROS node a fault kind acts on."""
    return {
        FaultEvent.KIND_SPAWN_OBSTACLE: f'/world/{world}/create (gz UserCommands)',
        FaultEvent.KIND_CLEAR_OBSTACLE: f'/world/{world}/remove (gz UserCommands)',
        FaultEvent.KIND_SCAN_DEGRADATION: 'scan_fault_relay (driver/transport level)',
        FaultEvent.KIND_MOBILITY_FAULT: f'/world/{world}/wrench/persistent (gz physics)',
        FaultEvent.KIND_KIDNAP: f'/world/{world}/set_pose (gz entity pose)',
    }[kind]


class MissionNode(Node):
    """Drives one mission against a running stack. Observes; never fakes."""

    def __init__(self, experiment_id: str, profile: dict, wtm,
                 node_name: str = 'run_scenario') -> None:
        super().__init__(
            node_name,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.experiment_id = experiment_id
        self.profile = profile
        # The measured world -> map alignment this run is scored in. The
        # MAVROS driver needs it to express a `map` goal in the autopilot's
        # frame; the Nav2 driver works in `map` throughout.
        self.wtm = wtm

        self.odom = None
        self.truth = None
        self.scan = None
        self.scan_raw = None
        self.grid = None
        self.amcl = None
        # The sim instant the filter's covariance trace FIRST fell below the
        # profile's convergence limit, observed in the callback. Sampling it
        # later, after the extra rotation and the yaw alignment, would measure
        # the whole warm-up and publish it under the convergence metric's
        # name - measured 17.01 s against metrics.json's 2.786 s for the same
        # run. None for a platform with no covariance to watch.
        self.converged_sim = None
        # One flag per profile `bringup_topics` entry, set by a generic
        # callback: the gate is "a message arrived", and the type comes from
        # the profile rather than from a hardcoded import, so adding a robot
        # does not mean editing this class.
        self.bringup_seen = {}

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Depth 10, transient local: a dashboard or a recorder that subscribes
        # after a transition still receives it. The bag gets them anyway - the
        # recorder is started before the first publication - but a late
        # /mission_state subscriber seeing nothing would look like a stalled
        # run rather than a missed message.
        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Typed subscriptions, for the streams this runner itself reads.
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan_raw,
                                 qos_profile_sensor_data)
        bringup = [entry['topic'] for entry in profile['bringup_topics']]
        if '/map' in bringup:
            self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        if '/odom' in bringup:
            self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        # The estimate, whatever it is for this robot. Built from the
        # descriptor so neither this class nor the metrics has to know which
        # estimator a platform runs.
        estimate = profile['estimate']
        if estimate['type'] == 'geometry_msgs/msg/PoseWithCovarianceStamped':
            self.create_subscription(PoseWithCovarianceStamped, estimate['topic'],
                                     self._on_amcl, 10)

        # The bring-up gate. A generic "a message arrived" subscription per
        # declared topic, on sensor-data QoS where the publisher is a sensor
        # stream and RELIABLE otherwise - both are matched by subscribing with
        # the lenient profile, so this never silently receives nothing.
        for entry in profile['bringup_topics']:
            topic = entry['topic']
            self.bringup_seen[topic] = False
            self.create_subscription(
                get_message(entry['type']), topic,
                lambda _msg, topic=topic: self.bringup_seen.__setitem__(topic, True),
                qos_profile_sensor_data)

        # Created only for a platform the runner can drive directly. An
        # Ackermann vehicle commanded by position setpoint has no /cmd_vel and
        # no /initialpose, and a publisher on a topic nothing subscribes to
        # would appear in the graph as a command path that does not exist.
        self.initial_pose_pub = None
        self.cmd_pub = None
        if profile['command']['kind'] == 'velocity_twist':
            self.initial_pose_pub = self.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10)
            # Used ONLY for the pre-mission localization warm-up. Nav2's
            # velocity_smoother owns /cmd_vel once a goal is active.
            self.cmd_pub = self.create_publisher(Twist, profile['command']['topic'], 10)

        self.mission_pub = self.create_publisher(MissionState, '/mission_state', event_qos)
        self.fault_pub = self.create_publisher(FaultEvent, '/fault_events', event_qos)

    # -- subscriptions --------------------------------------------------------
    def _on_odom(self, msg):
        self.odom = msg

    def _on_truth(self, msg):
        self.truth = msg

    def _on_scan(self, msg):
        self.scan = msg

    def _on_scan_raw(self, msg):
        self.scan_raw = msg

    def _on_map(self, msg):
        self.grid = msg

    def _on_amcl(self, msg):
        self.amcl = msg
        if self.converged_sim is None:
            cov = msg.pose.covariance
            limit = self.profile['estimate']['uncertainty'].get('converged_below')
            if limit is not None and cov[0] + cov[7] + cov[35] < float(limit):
                self.converged_sim = self.sim_now()

    # -- time -----------------------------------------------------------------
    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_wall(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_sim(self, sim_seconds: float, until=None, cmd: Twist = None) -> bool:
        """
        Spin until `sim_seconds` of SIM time pass, or `until()` fires.

        Republishes `cmd` at 20 Hz while spinning, for the localization
        warm-up only. The wall guard exists so a stopped /clock cannot hang
        the run for ever; it is deliberately ten times the sim budget.
        """
        start = self.sim_now()
        wall_deadline = time.monotonic() + sim_seconds * 10.0 + 60.0
        last_cmd = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if cmd is not None and now - last_cmd >= 0.05:
                self.cmd_pub.publish(cmd)
                last_cmd = now
            if until is not None and until():
                return True
            if self.sim_now() - start >= sim_seconds or now > wall_deadline:
                return False
        return False

    def stop(self) -> None:
        """Five zero velocity commands, for a platform that takes them."""
        if self.cmd_pub is None:
            return
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    # -- publications ---------------------------------------------------------
    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_of_yaw(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Nav2's own RViz tool uses these; they state a believable initial
        # uncertainty rather than claiming the pose is exact.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        self.initial_pose_pub.publish(msg)

    def publish_mission_state(self, state: str, goal: PoseStamped,
                              outcome: str = '', detail: str = '') -> None:
        # An outcome outside the closed set is a programming error, not an
        # operational one: every consumer - the metrics, the plot, the checks -
        # reads this string as a member of mission_driver.OUTCOMES, and one
        # that is not a member silently becomes an outcome nobody can score.
        # Raising here fails the run loudly at the moment the typo is
        # published rather than leaving it in the bag to be discovered later.
        if outcome and not outcome_is_known(outcome):
            raise ValueError(
                f'refusing to publish MissionState.outcome {outcome!r}: it is not a '
                'member of ral_eval.mission_driver.OUTCOMES (templates expanded). '
                'Add it there and to ral_interfaces/msg/MissionState.msg\'s closed '
                f'set, which are one artifact. Known: {list(OUTCOMES)}')
        msg = MissionState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.experiment_id = self.experiment_id
        msg.state = state
        msg.goal = goal
        msg.outcome = outcome
        msg.detail = detail
        self.mission_pub.publish(msg)
        print(f'--- mission_state {state} outcome={outcome!r} {detail}', flush=True)

    def publish_fault_rejected(self, fault_id: str, kind: str, world: str,
                               target: str, detail: str) -> None:
        """
        Publish the ONE FaultEvent this runner is entitled to publish.

        The fault manager owns requested/activated/cleared because only it
        observes the underlying call. This covers the case it cannot report:
        its service was not there to be called at all.
        """
        msg = FaultEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.id = fault_id
        msg.kind = kind
        msg.injection_point = injection_point(kind, world)
        msg.target = target
        msg.state = FaultEvent.STATE_REJECTED
        msg.detail = detail
        self.fault_pub.publish(msg)
        print(f'--- fault_event rejected {fault_id}: {detail}', flush=True)


def goal_pose_stamped(node: MissionNode, goal_map) -> PoseStamped:
    """Build the goal in `map`, stamped now: used by the action and the state."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = goal_map[0]
    pose.pose.position.y = goal_map[1]
    qx, qy, qz, qw = quaternion_of_yaw(goal_map[2])
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose
