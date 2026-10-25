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
Own fault injection: every /faults/* service and the active-fault registry.

This is the one node that calls the underlying gz and relay services, so it
is the one node that can say what happened.

Why one node
------------
`requested -> activated | rejected -> cleared` (FaultEvent.msg) is a claim
about what happened in the simulator, and only the caller of the underlying gz
service or relay service can make it. Splitting those callers across nodes
would split the registry too, and two owners of "what is in force" eventually
disagree - which is the failure CLAUDE.md forbids: a fault claimed on the
strength of a message having been sent.

So this node is the single authority on three things:

  * the CAPABILITY GATE   - the robot profile
                            (ral_bringup/config/robots/<robot>.yaml) says
                            which faults are real for the selected robot; an
                            unsupported request is `rejected` with that file's
                            reason VERBATIM, never attempted.
  * the EVENT STREAM      - it publishes `requested`, `activated`, `rejected`
                            and `cleared` on /fault_events. The scenario runner
                            publishes exactly one thing, `rejected`, and only
                            when this node's service was not there to call.
  * /faults/active        - the latched snapshot of what is still in force.

What `activated` means, per kind
--------------------------------
Not all five have the same evidence available, and the difference is stated
rather than smoothed over:

  spawn_obstacle, clear_obstacle, kidnap   gz's own service reply said success.
  scan_degradation                         scan_fault_relay replied that the
                                           parameters are in force.
  mobility_fault                           the EntityWrench was PUBLISHED with
                                           the gz bridge subscribed. gz
                                           acknowledges a wrench publication
                                           with nothing at all, so a
                                           subscriber count is the strongest
                                           pre-publication fact that exists;
                                           the FaultEvent detail says exactly
                                           that.

Wrench accumulation, the failure that looks like success
--------------------------------------------------------
gz's ApplyLinkWrench keeps a LIST of persistent wrenches
(`persistentWrenches.push_back`) and applies every entry each physics step.
Publishing a persistent wrench twice applies it twice, so re-projecting a
body-frame force as the robot turns would multiply the force without bound and
produce a dramatic, entirely fake result. Every publication here is therefore
preceded by a clear on /world/<world>/wrench/clear, which erases the entity's
persistent wrenches. During the `wrench_clear_settle_s` gap the force is
genuinely absent for that fraction of a second.

ApplyLinkWrench takes WORLD-frame forces ("Forces and torques are expressed in
world coordinates"), and a model target resolves to its canonical link
(base_link for ral_roomba). SetMobilityFault's force is BODY-frame, because a
"resistive push" that silently rotated with the map would be meaningless, so
this node projects it through the robot's current yaw - from ground truth,
which is legitimate here: this is the fault injector, not the robot's own
estimate, and the robot is never told.

Executor, and the lock that makes it safe
-----------------------------------------
MultiThreadedExecutor, with every service callback and every gz client in ONE
ReentrantCallbackGroup. A gz service call made from inside a service callback
deadlocks a SingleThreadedExecutor: the outer callback owns the only thread
and the inner future can never be served. The reentrant group plus several
threads is what makes "serve /faults/kidnap by calling /world/.../set_pose"
possible at all.

That arrangement serializes nothing, so every state-changing callback holds
`self.registry.lock` from its first line until its `_publish_active()` has
returned - including the gz or relay call in between. ral_faults/lifecycle.py
states the full contract and the two measured failures it prevents. The
consequence is deliberate: a gz call under the lock can hold it for up to
`service_wait_s` (10 s) and a second fault request waits that long, because
two fault mutations in flight at once is the failure being prevented. The one
caller that must not wait, the re-projection timer, takes the lock
non-blocking and skips the tick instead.
"""

import math
import re
import time

from nav_msgs.msg import Odometry
from ral_faults.catalogue import (
    check_params,
    kind_entry,
    load_catalogue,
    target_for,
)
from ral_faults.lifecycle import FaultRegistry, mission_start_is_new
from ral_eval.robot_profile import load_profile, supports
from ral_interfaces.msg import ActiveFaults, FaultEvent, MissionState
from ral_interfaces.srv import (
    ClearObstacle,
    Kidnap,
    SetMobilityFault,
    SetScanDegradation,
    SpawnObstacle,
)
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity, EntityWrench
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from std_srvs.srv import Trigger

# A static box, built from the requested size. Static: a dynamic obstacle
# would also be a moving-obstacle fault, which is a different experiment, and
# a 0.2 m-thin box tipped over by the first contact would make the run
# unrepeatable for reasons nothing records.
OBSTACLE_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>0.8 0.2 0.2 1</ambient><diffuse>0.8 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

# The obstacle name is SUBSTITUTED INTO THE SDF above, inside
# `<model name="{name}">`, and the result is sent to gz. A name carrying XML
# therefore injects arbitrary world content - `"><plugin filename="..."
# name="..."/><model name="x` loads a plugin into the running world - and a
# bare `&` or `<` corrupts the document into a confusing gz refusal. Phase 6
# puts an operator text field on this service, so the name is validated at the
# boundary rather than trusted.
OBSTACLE_NAME = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_-]{0,63}$')


def yaw_of_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class FaultManager(Node):
    """Serves every /faults/* service and owns what is in force."""

    def __init__(self) -> None:
        super().__init__('fault_manager')

        self.declare_parameter('world', 'ral_apartment')
        self.declare_parameter('robot', 'roomba')
        self.declare_parameter('catalogue_path', '')
        # ral_bringup/config/robots/<robot>.yaml. Supplied by the launch file,
        # never resolved here: see robot_profile.py's module docstring.
        self.declare_parameter('profile_path', '')
        self.declare_parameter('truth_topic', '/ground_truth/odom')
        self.declare_parameter('relay_service', '/scan_fault_relay/set_degradation')
        # WALL seconds. These guard discovery and a gz reply, both of which
        # happen outside the simulation; a sim-time deadline would never fire
        # before /clock exists.
        self.declare_parameter('service_wait_s', 10.0)
        # rad of yaw change before a body-frame wrench is re-projected. 0.15
        # rad is ~8.6 deg: at that heading error the force is still within
        # 1% of its intended body direction (1 - cos(0.15)).
        self.declare_parameter('wrench_reproject_rad', 0.15)
        self.declare_parameter('wrench_clear_settle_s', 0.05)
        # The spawn height is the robot's, and comes from its profile: the
        # scenario schema's kidnap pose carries x/y/yaw only, and a z of 0
        # would bury the wheels of either platform at a different depth.

        self.world = self._string_param('world')
        self.robot = self._string_param('robot')
        self.truth_topic = self._string_param('truth_topic')
        self.relay_service = self._string_param('relay_service')
        self.service_wait_s = self.get_parameter(
            'service_wait_s').get_parameter_value().double_value
        self.reproject_rad = self.get_parameter(
            'wrench_reproject_rad').get_parameter_value().double_value
        self.clear_settle_s = self.get_parameter(
            'wrench_clear_settle_s').get_parameter_value().double_value

        # A missing or malformed catalogue or profile RAISES here, so the node
        # fails to start instead of serving faults whose metadata, gz entity
        # or capability list it would have to invent.
        self.catalogue = load_catalogue(self._string_param('catalogue_path'))
        profile_path = self._string_param('profile_path')
        if not profile_path:
            raise ValueError(
                'profile_path is empty: the robot profile lives in '
                'ral_bringup/config/robots/<robot>.yaml and its path must be supplied '
                'by the launch file, because ral_bringup depends on this package')
        self.profile = load_profile(profile_path)
        if self.profile['robot'] != self.robot:
            raise ValueError(
                f'the `robot` parameter is {self.robot!r} but {profile_path} describes '
                f'{self.profile["robot"]!r}; the manager would gate faults for one '
                'platform and apply them to another')
        self.model = self.profile['model']
        self.kidnap_z = float(self.profile['kidnap_z_m'])
        self.estimator = self.profile['estimator_label']

        # One reentrant group for everything that can call out while serving:
        # see the module docstring.
        group = ReentrantCallbackGroup()

        # What is in force, and the single lock every state-changing callback
        # holds end to end. ral_faults/lifecycle.py carries the contract.
        self.registry = FaultRegistry()
        self.truth = None               # latest ground-truth Odometry, no lock
        self._last_mission_start = None  # stamp of the goal_accepted handled

        event_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.event_pub = self.create_publisher(FaultEvent, '/fault_events', event_qos)
        self.active_pub = self.create_publisher(ActiveFaults, '/faults/active', latched)
        self.wrench_pub = self.create_publisher(
            EntityWrench, f'/world/{self.world}/wrench/persistent', 10)
        self.wrench_clear_pub = self.create_publisher(
            Entity, f'/world/{self.world}/wrench/clear', 10)

        self.create_subscription(Odometry, self.truth_topic, self._on_truth, 10,
                                 callback_group=group)
        # Must match the runner's publisher QoS (depth 10, reliable,
        # transient local) or this subscription receives nothing at all and
        # the mission-start auto-clear silently never fires.
        self.create_subscription(MissionState, '/mission_state', self._on_mission,
                                 event_qos, callback_group=group)

        self.spawn_client = self.create_client(
            SpawnEntity, f'/world/{self.world}/create', callback_group=group)
        self.delete_client = self.create_client(
            DeleteEntity, f'/world/{self.world}/remove', callback_group=group)
        self.pose_client = self.create_client(
            SetEntityPose, f'/world/{self.world}/set_pose', callback_group=group)
        self.relay_client = self.create_client(
            SetScanDegradation, self.relay_service, callback_group=group)

        self.create_service(SpawnObstacle, '/faults/spawn_obstacle',
                            self._on_spawn_obstacle, callback_group=group)
        self.create_service(ClearObstacle, '/faults/clear_obstacle',
                            self._on_clear_obstacle, callback_group=group)
        self.create_service(SetScanDegradation, '/faults/set_scan_degradation',
                            self._on_set_scan_degradation, callback_group=group)
        self.create_service(SetMobilityFault, '/faults/set_mobility_fault',
                            self._on_set_mobility_fault, callback_group=group)
        self.create_service(Kidnap, '/faults/kidnap', self._on_kidnap,
                            callback_group=group)
        self.create_service(Trigger, '/faults/clear_all', self._on_clear_all,
                            callback_group=group)

        # 10 Hz on SIM time: the re-projection has to track the body, and the
        # body moves in sim time.
        self.create_timer(0.1, self._on_reproject, callback_group=group)

        # The latch, published before anything can be requested: a subscriber
        # that joins later must be able to tell "nothing is in force" from
        # "the manager is not running", and only a latched empty snapshot says
        # the first.
        self._publish_active()

        self.get_logger().info(
            f'fault_manager: world {self.world}, robot {self.robot} (gz model '
            f'{self.model}), faults {sorted(self.profile["faults"])}, '
            f'relay {self.relay_service}')

    def _string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    # -- subscriptions --------------------------------------------------------
    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def _on_mission(self, msg: MissionState) -> None:
        """Clear every active fault when a new mission starts."""
        stamp = (msg.stamp.sec, msg.stamp.nanosec)
        with self.registry.lock:
            if not mission_start_is_new(msg.state, stamp, self._last_mission_start):
                return
            self._last_mission_start = stamp
            cleared, failed = self._clear_everything('cleared at mission start')
        if cleared or failed:
            self.get_logger().warn(
                f'mission start: cleared {cleared} fault(s), {failed} could not be '
                'cleared')

    # -- events and the registry ----------------------------------------------
    def _event(self, fault_id: str, kind: str, target: str, state: str,
               detail: str) -> FaultEvent:
        msg = FaultEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.id = fault_id
        msg.kind = kind
        msg.injection_point = kind_entry(self.catalogue, kind)['injection_point'].format(
            world=self.world)
        msg.target = target
        msg.state = state
        msg.detail = detail
        self.event_pub.publish(msg)
        self.get_logger().info(f'fault_event {state} {fault_id} ({kind}): {detail}')
        return msg

    def _publish_active(self) -> None:
        msg = ActiveFaults()
        msg.stamp = self.get_clock().now().to_msg()
        entries = [entry for _key, entry in self.registry.snapshot()]
        for entry in entries:
            event = entry['event']
            # The activated event verbatim, with detail refreshed: the
            # re-projection count and the elapsed time change while the fault
            # is in force, and a snapshot that still said "0 re-projections"
            # would be stale the moment the robot turned.
            refreshed = FaultEvent()
            refreshed.stamp = event.stamp
            refreshed.id = event.id
            refreshed.kind = event.kind
            refreshed.injection_point = event.injection_point
            refreshed.target = event.target
            refreshed.state = event.state
            refreshed.detail = entry['detail'](entry)
            msg.active.append(refreshed)
        self.active_pub.publish(msg)

    def _gate(self, kind: str) -> 'str | None':
        """Return the capability refusal reason for this robot, or None."""
        return supports(self.profile, kind)

    def _register(self, key, fault_id: str, kind: str, target: str, params: dict,
                  event: FaultEvent, detail, extra: dict = None) -> None:
        """
        Record one fault as IN FORCE, complete in a single assignment.

        `detail` is called with the entry when /faults/active is published, so
        a fault whose description changes while it is in force (the wrench's
        re-projection count) reports the current one. The entry is still built
        whole before it enters the registry, and the lock contract (see
        ral_faults/lifecycle.py) means no reader can observe it half-built:
        every caller that reads an entry holds the same lock this `put` takes.
        """
        entry = {
            'fault_id': fault_id, 'kind': kind, 'target': target,
            'params': params, 'event': event, 'detail': detail,
            'activated_stamp': (event.stamp.sec, event.stamp.nanosec),
        }
        entry.update(extra or {})
        self.registry.put(key, entry)

    # -- gz plumbing ----------------------------------------------------------
    def _call(self, client, request, name: str) -> 'tuple[bool, str, object]':
        """
        Call one service and report what happened, never what was intended.

        Returns (ok, reason, response). `ok` false carries the verbatim reason:
        the client was not there, or it never answered. A response that itself
        says success=false is `ok` true with the response attached - that is a
        refusal by the far side, and the caller reports ITS message.
        """
        if not client.wait_for_service(timeout_sec=self.service_wait_s):
            return (False,
                    f'{name} not available after {self.service_wait_s:.1f} s; the gz '
                    'service bridge is not running, so the fault was NOT applied',
                    None)
        future = client.call_async(request)
        deadline = time.monotonic() + self.service_wait_s
        while not future.done() and time.monotonic() < deadline:
            # The executor's other threads complete this future; this callback
            # must not spin the node itself (it is already inside one).
            time.sleep(0.01)
        if not future.done():
            return (False,
                    f'{name} did not answer within {self.service_wait_s:.1f} s; the '
                    'fault cannot be shown to have been applied',
                    None)
        return True, '', future.result()

    def _apply_wrench(self, force_x: float, force_y: float,
                      torque_z: float) -> 'tuple[bool, str, float]':
        """
        Publish one persistent wrench, body frame in, world frame out.

        Returns (ok, detail, yaw used). The CLEAR is mandatory and comes
        first: ApplyLinkWrench accumulates persistent wrenches, so publishing
        without clearing multiplies the force by the number of publications.
        Both topics are therefore checked for a subscriber before anything is
        published - a clear that nobody receives is exactly as harmful here as
        a wrench that nobody receives.
        """
        if self.truth is None:
            return (False,
                    f'no ground-truth pose yet on {self.truth_topic}; the body-frame '
                    'wrench cannot be projected into the world frame gz requires',
                    0.0)
        if self.wrench_pub.get_subscription_count() == 0:
            return (False,
                    f'no subscriber on /world/{self.world}/wrench/persistent: the gz '
                    'bridge is not running, so the wrench would be dropped',
                    0.0)
        if self.wrench_clear_pub.get_subscription_count() == 0:
            return (False,
                    f'no subscriber on /world/{self.world}/wrench/clear: without the '
                    'clear, gz accumulates persistent wrenches and the applied force '
                    'would be a multiple of the requested one',
                    0.0)

        yaw = yaw_of_quaternion(self.truth.pose.pose.orientation)
        world_x = force_x * math.cos(yaw) - force_y * math.sin(yaw)
        world_y = force_x * math.sin(yaw) + force_y * math.cos(yaw)

        cleared, clear_detail = self._clear_wrench()
        if not cleared:
            return False, clear_detail, 0.0
        # Give gz the clear before the new wrench arrives. Measured
        # consequence: for this fraction of a second the force is absent.
        time.sleep(self.clear_settle_s)

        target = Entity()
        target.name = self.model
        target.type = Entity.MODEL
        wrench = EntityWrench()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.entity = target
        wrench.wrench.force.x = world_x
        wrench.wrench.force.y = world_y
        wrench.wrench.force.z = 0.0
        wrench.wrench.torque.x = 0.0
        wrench.wrench.torque.y = 0.0
        # Planar robot: the body z axis is the world z axis, so a yaw torque
        # needs no projection.
        wrench.wrench.torque.z = torque_z
        self.wrench_pub.publish(wrench)

        detail = (f'body-frame force ({force_x:.3f}, {force_y:.3f}) N and torque '
                  f'{torque_z:.3f} N.m published on /world/{self.world}/wrench/'
                  f'persistent as world-frame ({world_x:.3f}, {world_y:.3f}) N at yaw '
                  f'{yaw:.3f} rad, after clearing the previous persistent wrench')
        return True, detail, yaw

    def _clear_wrench(self) -> 'tuple[bool, str]':
        """
        Erase the entity's persistent wrenches. Returns (ok, detail).

        gz acknowledges neither the wrench nor the clear, so a subscribed
        bridge is the strongest pre-publication fact that exists - the same
        evidence `activated` rests on for this kind. Without it the clear is
        unevidenced, and a `cleared` published anyway would be the project's
        forbidden claim: a fault reported gone because a message was sent.
        """
        if self.wrench_clear_pub.get_subscription_count() == 0:
            return False, (
                f'no subscriber on /world/{self.world}/wrench/clear: the gz bridge is '
                'not running, so the persistent wrench CANNOT be shown to have been '
                'removed and must be assumed still in force')
        target = Entity()
        target.name = self.model
        target.type = Entity.MODEL
        self.wrench_clear_pub.publish(target)
        return True, (f'clear published on /world/{self.world}/wrench/clear with the '
                      'gz bridge subscribed')

    # -- services -------------------------------------------------------------
    def _on_spawn_obstacle(self, request, response):
        with self.registry.lock:
            return self._spawn_obstacle(request, response)

    def _spawn_obstacle(self, request, response):
        kind = FaultEvent.KIND_SPAWN_OBSTACLE
        params = {'name': request.name,
                  'pose': {'x': request.pose.position.x, 'y': request.pose.position.y},
                  'size': {'x': request.size.x, 'y': request.size.y, 'z': request.size.z}}
        fault_id = self.registry.mint_id(kind, request.fault_id)
        target = target_for(self.catalogue, kind, {'name': request.name}, self.model)
        self._event(fault_id, kind, target, FaultEvent.STATE_REQUESTED,
                    f'spawn {request.name}: {request.size.x:.3f} x {request.size.y:.3f} '
                    f'x {request.size.z:.3f} m box at world '
                    f'({request.pose.position.x:.3f}, {request.pose.position.y:.3f})')

        refusal = self._gate(kind)
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)

        if not OBSTACLE_NAME.match(request.name):
            return self._reject(
                response, fault_id, kind, target,
                f'obstacle name {request.name!r} is not '
                '[A-Za-z0-9_][A-Za-z0-9_-]{0,63}; the name is substituted into the '
                'SDF this manager sends to gz, so a name carrying XML would inject '
                'arbitrary world content')

        if self.registry.get((kind, request.name)) is not None:
            return self._reject(
                response, fault_id, kind, target,
                f'obstacle {request.name!r} is already spawned in this run; gz entity '
                'names are unique and a second spawn would fail')

        refusal = check_params(self.catalogue, kind, {
            'pose.position.x': request.pose.position.x,
            'pose.position.y': request.pose.position.y,
            'size.x': request.size.x, 'size.y': request.size.y,
            'size.z': request.size.z})
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)
        for axis in ('x', 'y', 'z'):
            extent = getattr(request.size, axis)
            if extent <= 0.0:
                return self._reject(
                    response, fault_id, kind, target,
                    f'size.{axis} is {extent}; a box extent must be positive')

        gz_request = SpawnEntity.Request()
        gz_request.entity_factory.name = request.name
        # False: a renamed entity could not be removed by the name the
        # scenario clears, and the run would record a cleared fault that is
        # still standing in the world.
        gz_request.entity_factory.allow_renaming = False
        gz_request.entity_factory.sdf = OBSTACLE_SDF.format(
            name=request.name, sx=request.size.x, sy=request.size.y, sz=request.size.z)
        gz_request.entity_factory.relative_to = 'world'
        # Built field by field rather than assigned: `pose = request.pose`
        # aliases the CALLER's message, and the z substitution below would
        # then rewrite the request itself, which is the record of what was
        # asked for.
        gz_request.entity_factory.pose.position.x = request.pose.position.x
        gz_request.entity_factory.pose.position.y = request.pose.position.y
        gz_request.entity_factory.pose.orientation.x = request.pose.orientation.x
        gz_request.entity_factory.pose.orientation.y = request.pose.orientation.y
        gz_request.entity_factory.pose.orientation.z = request.pose.orientation.z
        gz_request.entity_factory.pose.orientation.w = request.pose.orientation.w
        # The scenario schema carries x/y/yaw only; a box spawned at z=0 would
        # be half under the floor, so it is lifted by half its height to rest
        # ON the floor. Said here and in the FaultEvent detail, not assumed.
        gz_request.entity_factory.pose.position.z = request.size.z / 2.0

        ok, reason, gz_response = self._call(
            self.spawn_client, gz_request, f'/world/{self.world}/create')
        if not ok:
            return self._reject(response, fault_id, kind, target, reason)
        if not gz_response.success:
            return self._reject(
                response, fault_id, kind, target,
                f'/world/{self.world}/create returned success=false for '
                f'{request.name!r}; gz rejected the entity')

        detail = (f'gz created static box {request.name!r} '
                  f'({request.size.x:.3f} x {request.size.y:.3f} x {request.size.z:.3f} m) '
                  f'at world ({request.pose.position.x:.3f}, '
                  f'{request.pose.position.y:.3f}, z={request.size.z / 2.0:.3f} set to '
                  'half the box height so it rests on the floor); the lidar sees it '
                  'from the next sweep')
        event = self._event(fault_id, kind, target, FaultEvent.STATE_ACTIVATED, detail)
        self._register((kind, request.name), fault_id, kind, target, params, event,
                       lambda entry, detail=detail: detail)
        self._publish_active()
        response.fault_id = fault_id
        response.success = True
        response.message = detail
        return response

    def _on_clear_obstacle(self, request, response):
        with self.registry.lock:
            return self._clear_obstacle(request, response)

    def _clear_obstacle(self, request, response):
        kind = FaultEvent.KIND_CLEAR_OBSTACLE
        fault_id = self.registry.mint_id(kind, request.fault_id)
        target = target_for(self.catalogue, kind, {'name': request.name}, self.model)
        self._event(fault_id, kind, target, FaultEvent.STATE_REQUESTED,
                    f'remove {request.name}')

        refusal = self._gate(kind)
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)

        # This name reaches the registry only, never the SDF - but a refusal
        # here is the clearer error than "never spawned by this manager" for
        # a name that could not have been spawned in the first place.
        if not OBSTACLE_NAME.match(request.name):
            return self._reject(
                response, fault_id, kind, target,
                f'obstacle name {request.name!r} is not '
                '[A-Za-z0-9_][A-Za-z0-9_-]{0,63}; the name is substituted into the '
                'SDF this manager sends to gz, so a name carrying XML would inject '
                'arbitrary world content')

        key = (FaultEvent.KIND_SPAWN_OBSTACLE, request.name)
        spawned = self.registry.get(key)
        if spawned is None:
            return self._reject(
                response, fault_id, kind, target,
                f'{request.name!r} was never spawned by this manager; removing an '
                'entity it did not create would report a cleared fault that never '
                'existed')

        ok, reason, gz_response = self._call(
            self.delete_client, self._delete_request(request.name),
            f'/world/{self.world}/remove')
        if not ok:
            return self._reject(response, fault_id, kind, target, reason)
        if not gz_response.success:
            return self._reject(
                response, fault_id, kind, target,
                f'/world/{self.world}/remove returned success=false for '
                f'{request.name!r}; the entity is still in the world')

        self.registry.pop(key)
        # Two ids, each exactly once: the spawn's instance ends at `cleared`,
        # and the clear request is itself a fault instance that reached
        # `activated`. A reader can join either way round. Unlike a disable,
        # this `activated` is not a second state for the same thing: the
        # catalogue lifetime of clear_obstacle is `oneshot` with
        # cleared_by: null, so a reader can tell it is not something in force.
        self._event(spawned['fault_id'], FaultEvent.KIND_SPAWN_OBSTACLE,
                    spawned['target'], FaultEvent.STATE_CLEARED,
                    f'removed by {fault_id}')
        detail = (f'gz removed {request.name!r} from the world; the costmaps clear it '
                  'as the lidar sweeps the space again')
        self._event(fault_id, kind, target, FaultEvent.STATE_ACTIVATED, detail)
        self._publish_active()
        response.fault_id = fault_id
        response.success = True
        response.message = detail
        return response

    def _delete_request(self, name: str) -> DeleteEntity.Request:
        gz_request = DeleteEntity.Request()
        gz_request.entity.name = name
        gz_request.entity.type = Entity.MODEL
        return gz_request

    def _on_set_scan_degradation(self, request, response):
        with self.registry.lock:
            return self._set_scan_degradation(request, response)

    def _set_scan_degradation(self, request, response):
        kind = FaultEvent.KIND_SCAN_DEGRADATION
        fault_id = self.registry.mint_id(kind, request.fault_id)
        target = target_for(self.catalogue, kind, {}, self.model)
        self._event(
            fault_id, kind, target, FaultEvent.STATE_REQUESTED,
            f'enabled={request.enabled}, noise {request.range_noise_stddev_m} m, max '
            f'range {request.max_range_m} m, blind sector '
            f'{request.blind_sector_start_deg} -> {request.blind_sector_end_deg} deg, '
            f'dropout {request.dropout_probability}')

        refusal = self._gate(kind)
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)

        if request.enabled:
            # Only on an enable: the srv says the remaining fields are ignored
            # on a disable, so bounding them there would refuse a request
            # whose values are never applied.
            refusal = check_params(self.catalogue, kind, {
                'range_noise_stddev_m': request.range_noise_stddev_m,
                'max_range_m': request.max_range_m,
                'blind_sector_start_deg': request.blind_sector_start_deg,
                'blind_sector_end_deg': request.blind_sector_end_deg,
                'dropout_probability': request.dropout_probability})
            if refusal:
                return self._reject(response, fault_id, kind, target, refusal)

        key = (kind,)
        active = self.registry.get(key)

        if not request.enabled and active is None:
            return self._reject(
                response, fault_id, kind, target,
                'no scan_degradation is in force; disabling one that was never '
                'activated would report a cleared fault that never existed')

        ok, reason, relay_response = self._call(
            self.relay_client, request, self.relay_service)
        if not ok:
            return self._reject(
                response, fault_id, kind, target,
                f'{reason} (scan_fault_relay is the injection point; without it '
                '/scan is an undegraded copy of /scan_raw)')
        if not relay_response.success:
            return self._reject(response, fault_id, kind, target,
                                f'{self.relay_service} refused: {relay_response.message}')

        if not request.enabled:
            self.registry.pop(key)
            # ONE event: `cleared` for the instance that was in force. The
            # disable is not itself something in force - this kind's catalogue
            # lifetime is `singleton` - and publishing `activated` for it too
            # would leave a reader computing "in force = activated - cleared"
            # with a permanent phantom. The requester is named in the detail,
            # and the response carries its own id.
            self._event(active['fault_id'], kind, target, FaultEvent.STATE_CLEARED,
                        f'{relay_response.message}; cleared by {fault_id}')
            self._publish_active()
            response.fault_id = fault_id
            response.success = True
            response.message = relay_response.message
            return response

        if active is not None:
            # A singleton: the relay holds exactly one configuration, so the
            # old instance is genuinely gone and must not stay in the snapshot.
            self._event(active['fault_id'], kind, target, FaultEvent.STATE_CLEARED,
                        f'superseded by {fault_id}')
        detail = relay_response.message
        event = self._event(fault_id, kind, target, FaultEvent.STATE_ACTIVATED, detail)
        params = {'enabled': True,
                  'range_noise_stddev_m': request.range_noise_stddev_m,
                  'max_range_m': request.max_range_m,
                  'blind_sector_start_deg': request.blind_sector_start_deg,
                  'blind_sector_end_deg': request.blind_sector_end_deg,
                  'dropout_probability': request.dropout_probability}
        self._register(key, fault_id, kind, target, params, event,
                       lambda entry, detail=detail: detail)
        self._publish_active()
        response.fault_id = fault_id
        response.success = True
        response.message = detail
        return response

    def _on_set_mobility_fault(self, request, response):
        with self.registry.lock:
            return self._set_mobility_fault(request, response)

    def _set_mobility_fault(self, request, response):
        kind = FaultEvent.KIND_MOBILITY_FAULT
        fault_id = self.registry.mint_id(kind, request.fault_id)
        target = target_for(self.catalogue, kind, {}, self.model)
        self._event(
            fault_id, kind, target, FaultEvent.STATE_REQUESTED,
            f'enabled={request.enabled}, body-frame force ({request.force_x_n}, '
            f'{request.force_y_n}) N, torque {request.torque_z_nm} N.m')

        refusal = self._gate(kind)
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)

        if request.enabled:
            # Finiteness only: the catalogue deliberately declares no force
            # bounds, because any finite wrench is a wrench gz can apply and
            # an implausibly large one is a visible result, not an invalid
            # request. A NaN or infinite one is neither.
            refusal = check_params(self.catalogue, kind, {
                'force_x_n': request.force_x_n, 'force_y_n': request.force_y_n,
                'torque_z_nm': request.torque_z_nm})
            if refusal:
                return self._reject(response, fault_id, kind, target, refusal)

        key = (kind,)
        active = self.registry.get(key)

        if not request.enabled:
            if active is None:
                return self._reject(
                    response, fault_id, kind, target,
                    'no mobility_fault is in force; disabling one that was never '
                    'activated would report a cleared fault that never existed')
            # The clear comes FIRST and its failure is a refusal: the registry
            # entry stays, /faults/active keeps listing the fault, and no
            # `cleared` is published for a wrench that may still be in gz.
            cleared, clear_detail = self._clear_wrench()
            if not cleared:
                return self._reject(response, fault_id, kind, target, clear_detail)
            self.registry.pop(key)
            detail = (f'{clear_detail} after {active["reprojections"]} '
                      're-projection(s)')
            # ONE event, as for scan_degradation: see that handler's note.
            self._event(active['fault_id'], kind, target, FaultEvent.STATE_CLEARED,
                        f'{detail}; cleared by {fault_id}')
            self._publish_active()
            response.fault_id = fault_id
            response.success = True
            response.message = detail
            return response

        ok, detail, yaw = self._apply_wrench(
            request.force_x_n, request.force_y_n, request.torque_z_nm)
        if not ok:
            return self._reject(response, fault_id, kind, target, detail)

        if active is not None:
            self._event(active['fault_id'], kind, target, FaultEvent.STATE_CLEARED,
                        f'superseded by {fault_id}')
        # The catalogue's note is platform-independent: what a wrench IS, and
        # what `activated` claims. What a wrench DOES depends on the plant it
        # is applied to - a deficit ladder measured on the Roomba says nothing
        # about the car - so the measured half comes from the robot profile
        # and is published verbatim beside it.
        note = kind_entry(self.catalogue, kind)['honesty_note']
        detail = f'{detail}. {note}'
        plant_note = str(self.profile.get('mobility_note', '')).strip()
        if plant_note:
            detail = f'{detail} On {self.robot}: {plant_note}'
        event = self._event(fault_id, kind, target, FaultEvent.STATE_ACTIVATED, detail)
        params = {'enabled': True, 'force_x_n': request.force_x_n,
                  'force_y_n': request.force_y_n, 'torque_z_nm': request.torque_z_nm}
        self._register(key, fault_id, kind, target, params, event,
                       lambda entry, detail=detail: (
                           f'{detail} [{entry["reprojections"]} re-projection(s) '
                           'since]'),
                       extra={'yaw_at_apply': yaw, 'reprojections': 0})
        self._publish_active()
        response.fault_id = fault_id
        response.success = True
        response.message = detail
        return response

    def _on_kidnap(self, request, response):
        with self.registry.lock:
            return self._kidnap(request, response)

    def _kidnap(self, request, response):
        kind = FaultEvent.KIND_KIDNAP
        fault_id = self.registry.mint_id(kind, request.fault_id)
        target = target_for(self.catalogue, kind, {}, self.model)
        self._event(
            fault_id, kind, target, FaultEvent.STATE_REQUESTED,
            f'teleport {self.model} to world ({request.new_pose.position.x:.3f}, '
            f'{request.new_pose.position.y:.3f})')

        refusal = self._gate(kind)
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)

        orientation = request.new_pose.orientation
        refusal = check_params(self.catalogue, kind, {
            'new_pose.position.x': request.new_pose.position.x,
            'new_pose.position.y': request.new_pose.position.y,
            'new_pose.orientation.x': orientation.x,
            'new_pose.orientation.y': orientation.y,
            'new_pose.orientation.z': orientation.z,
            'new_pose.orientation.w': orientation.w})
        if refusal:
            return self._reject(response, fault_id, kind, target, refusal)

        gz_request = SetEntityPose.Request()
        gz_request.entity.name = self.model
        gz_request.entity.type = Entity.MODEL
        # Field by field, not `gz_request.pose = request.new_pose`: that
        # aliases the caller's message and the kidnap_z substitution below
        # would rewrite the request, which is the record of what was asked.
        gz_request.pose.position.x = request.new_pose.position.x
        gz_request.pose.position.y = request.new_pose.position.y
        gz_request.pose.orientation.x = orientation.x
        gz_request.pose.orientation.y = orientation.y
        gz_request.pose.orientation.z = orientation.z
        gz_request.pose.orientation.w = orientation.w
        gz_request.pose.position.z = self.kidnap_z

        ok, reason, gz_response = self._call(
            self.pose_client, gz_request, f'/world/{self.world}/set_pose')
        if not ok:
            return self._reject(response, fault_id, kind, target, reason)
        if not gz_response.success:
            return self._reject(
                response, fault_id, kind, target,
                f'/world/{self.world}/set_pose returned success=false for '
                f'{self.model!r}; the body did not move')

        note = kind_entry(self.catalogue, kind)['honesty_note']
        detail = (f'gz moved {self.model} to world ({gz_request.pose.position.x:.3f}, '
                  f'{gz_request.pose.position.y:.3f}, z={self.kidnap_z:.3f} substituted '
                  'by the manager). The body moved with no actuation and no inertial '
                  f'signature, so {self.estimator} is now wrong by the teleport '
                  f'distance and must recover alone. {note}')
        self._event(fault_id, kind, target, FaultEvent.STATE_ACTIVATED, detail)
        # A one-shot: the world changed once. There is no state to hold and
        # nothing to clear, so it never enters /faults/active.
        response.fault_id = fault_id
        response.success = True
        response.message = detail
        return response

    def _on_clear_all(self, request, response):
        with self.registry.lock:
            cleared, failed = self._clear_everything('cleared by /faults/clear_all')
        response.success = failed == 0
        response.message = (f'{cleared} fault(s) cleared, {failed} could not be '
                            'cleared')
        return response

    def _reject(self, response, fault_id: str, kind: str, target: str, detail: str):
        """Publish the rejection and answer with the SAME string and id."""
        self._event(fault_id, kind, target, FaultEvent.STATE_REJECTED, detail)
        response.fault_id = fault_id
        response.success = False
        response.message = detail
        return response

    # -- clearing -------------------------------------------------------------
    def _clear_everything(self, reason: str) -> 'tuple[int, int]':
        """
        Remove every fault in the registry. Returns (cleared, failed).

        Called with `registry.lock` already held by the service or mission
        callback that wants everything gone, so nothing can be added, removed
        or re-projected while this walks the snapshot.

        A removal that FAILS leaves the entry in place and publishes
        `rejected`, not `cleared`: a fault still in force must stay visible in
        /faults/active, and claiming otherwise is the one thing this node
        exists to prevent.
        """
        cleared = 0
        failed = 0
        for key, entry in self.registry.snapshot():
            kind = entry['kind']
            ok, detail = self._remove(kind, entry)
            if ok:
                self.registry.pop_if(key, entry)
                self._event(entry['fault_id'], kind, entry['target'],
                            FaultEvent.STATE_CLEARED, f'{reason}: {detail}')
                cleared += 1
            else:
                self._event(entry['fault_id'], kind, entry['target'],
                            FaultEvent.STATE_REJECTED,
                            f'{reason} FAILED, the fault is still in force: {detail}')
                failed += 1
        if cleared or failed:
            self._publish_active()
        return cleared, failed

    def _remove(self, kind: str, entry: dict) -> 'tuple[bool, str]':
        """Undo one active fault through the path that created it."""
        if kind == FaultEvent.KIND_SPAWN_OBSTACLE:
            name = entry['params']['name']
            ok, reason, gz_response = self._call(
                self.delete_client, self._delete_request(name),
                f'/world/{self.world}/remove')
            if not ok:
                return False, reason
            if not gz_response.success:
                return False, (f'/world/{self.world}/remove returned success=false for '
                               f'{name!r}')
            return True, f'gz removed {name!r}'
        if kind == FaultEvent.KIND_MOBILITY_FAULT:
            # The clear's own verdict: without a subscribed bridge it cannot
            # be shown to have happened, and this returns False so the caller
            # publishes `rejected` and keeps the entry.
            ok, detail = self._clear_wrench()
            if not ok:
                return False, detail
            return True, (f'{detail} after {entry["reprojections"]} '
                          're-projection(s)')
        if kind == FaultEvent.KIND_SCAN_DEGRADATION:
            disable = SetScanDegradation.Request()
            disable.fault_id = entry['fault_id']
            disable.enabled = False
            ok, reason, relay_response = self._call(
                self.relay_client, disable, self.relay_service)
            if not ok:
                return False, reason
            if not relay_response.success:
                return False, f'{self.relay_service} refused: {relay_response.message}'
            return True, relay_response.message
        return False, f'{kind} has no removal path in this manager'

    # -- the re-projection timer ----------------------------------------------
    def _on_reproject(self) -> None:
        """
        Keep a BODY-frame wrench pointing where the body points.

        gz accepts world-frame wrenches only, so a force applied at one
        heading becomes a sideways push once the robot turns. Re-publishing it
        for the new yaw is what keeps "a resistive force pushing back along
        the robot's x axis" true for the whole fault.

        The lock is taken NON-BLOCKING: a fault request holding it may be
        inside a gz call for seconds, and a timer that waited would queue
        re-projections behind it and publish a wrench after the fault was
        cleared - the registry empty, `cleared` on the wire and the force
        still in gz. Skipping a tick costs nothing: the next is 100 ms later
        and the yaw threshold still holds.

        No FaultEvent is published per re-projection: nothing TRANSITIONED -
        the same fault instance is still in force - and a stream of events
        would double-count in fault_to_response_s. The count is carried in
        /faults/active's detail and in the final `cleared` event.
        """
        if not self.registry.lock.acquire(blocking=False):
            self.get_logger().debug(
                're-projection skipped: a fault request holds the registry')
            return
        try:
            entry = self.registry.get((FaultEvent.KIND_MOBILITY_FAULT,))
            if entry is None or self.truth is None:
                return
            yaw = yaw_of_quaternion(self.truth.pose.pose.orientation)
            if abs(wrap_to_pi(yaw - entry['yaw_at_apply'])) <= self.reproject_rad:
                return
            params = entry['params']
            ok, detail, new_yaw = self._apply_wrench(
                params['force_x_n'], params['force_y_n'], params['torque_z_nm'])
            if not ok:
                self.get_logger().warn(
                    f're-projection failed, wrench unchanged: {detail}')
                return
            entry['yaw_at_apply'] = new_yaw
            entry['reprojections'] += 1
            self._publish_active()
        finally:
            self.registry.lock.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FaultManager()
    # Several threads, so a gz call made from inside a service callback can be
    # completed by another thread. One thread deadlocks every /faults/* call.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # `ros2 launch` SIGINTs the whole process group; rclpy surfaces that
        # as either of these, depending on where the signal lands.
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
