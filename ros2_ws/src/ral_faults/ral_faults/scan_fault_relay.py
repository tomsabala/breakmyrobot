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
The lidar fault-injection boundary, and the SOLE publisher of /scan.

Every consumer of lidar data in this project reads /scan: AMCL, both Nav2
costmaps and, while the map is being built, slam_toolbox. Nothing reads
/scan_raw except evaluation code that wants the undegraded sensor stream to
compare against. That split is the entire reason this node exists.

Two paths, and the difference between them is the whole contract:

  degradation OFF  the received object is republished UNCHANGED - the same
                   message, not a copy: no restamping, no field rebuilding, no
                   rounding. A relay that distorted the clean stream would
                   corrupt every non-fault run in the project silently, and
                   the baseline that fault runs are compared against would
                   already be wrong. Check 3 of
                   scripts/checks/roomba_nav_check.py asserts this end to end
                   (same header.stamp on /scan_raw and /scan => identical
                   ranges and frame_id) and therefore only holds while
                   degradation is off, which is the state that harness runs in.

  degradation ON   a new LaserScan carrying the SAME header, angles, times,
                   range_min/range_max and intensities, with only `ranges`
                   replaced by scan_degradation.degrade_ranges. range_max is
                   the device's advertised maximum and is NOT rewritten to
                   max_range_m: a truncated beam is already +inf, and moving
                   range_max would tell every consumer the hardware changed.

Who serves what
---------------
This node serves `/scan_fault_relay/set_degradation`, a node-private name.
The operator-facing `/faults/set_scan_degradation` is served by
ral_faults/fault_manager.py, which forwards the request here: one node has to
own the capability gate, the FaultEvent lifecycle and /faults/active, and two
owners of that state would eventually disagree about what is in force. The
relay stays the INJECTION POINT - it is the only place the stream can be
changed, because it is the only publisher of /scan.

Determinism
-----------
The RNG is `random.Random(seed)`, reseeded on every enable, so the corrupted
stream is a function of (`seed`, scans since activation) and repeats across
runs of the same scenario. `seed` comes from the scenario through
faults.launch.py. It governs this node's noise and dropout only: nav2_amcl
exposes no seed, so a run is repeatable in its sensor corruption and not in
its particle filter.

HONESTY NOTE, verbatim from fault_catalogue.yaml: this corrupts the LaserScan
stream between the sensor bridge and its consumers. The simulated lidar itself
stays healthy - Gazebo Harmonic cannot change SDF sensor noise at runtime.

The degradation maths is pure and unit-tested in test/test_scan_degradation.py.
What cannot be tested there - that /scan has exactly one publisher and that it
is this node - is asserted against a running system by checks 2 and 3 of the
proof harness.
"""

import random

from ral_faults.scan_degradation import Degradation, degrade_ranges
from ral_interfaces.srv import SetScanDegradation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# A PRIVATE name ('~/'): it resolves against the NODE name, so it is
# /scan_fault_relay/set_degradation and cannot collide with the manager's
# operator-facing /faults/set_scan_degradation. A bare relative name would
# resolve against the namespace instead and land at /set_degradation.
SERVICE_NAME = '~/set_degradation'


class ScanFaultRelay(Node):
    """Republishes the raw lidar stream, degrading it when a fault is in force."""

    def __init__(self) -> None:
        super().__init__('scan_fault_relay')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        # The scenario's seed. Integer: a launch file that passes a bare
        # LaunchConfiguration passes a STRING and this declaration rejects it,
        # which is why faults.launch.py wraps it in ParameterValue.
        self.declare_parameter('seed', 0)

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._seed = self.get_parameter('seed').get_parameter_value().integer_value

        self._config = None          # Degradation in force, or None
        self._rng = random.Random(self._seed)
        self._scans_degraded = 0

        # Subscribe best-effort (qos_profile_sensor_data). A best-effort
        # subscription is compatible with the ros_gz bridge's reliable
        # publisher and would also accept a genuinely best-effort source, so
        # swapping the sensor's origin later does not silently produce an
        # incompatible-QoS pair with no messages and no error.
        #
        # Publish RELIABLE: reliable satisfies both reliable and best-effort
        # subscribers, whereas a best-effort publisher is invisible to a
        # reliable subscriber. Sensor-data subscribers (Nav2's) are served
        # either way; the host's rviz2, which subscribes reliably unless its
        # display is reconfigured, is not - and a scan that cannot be drawn
        # during a fault run is a diagnostic lost for no gain.
        self._pub = self.create_publisher(
            LaserScan, self._output_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._sub = self.create_subscription(
            LaserScan, self._input_topic, self._on_scan, qos_profile_sensor_data)
        self._service = self.create_service(
            SetScanDegradation, SERVICE_NAME, self._on_set_degradation)

        self.get_logger().info(
            f'scan relay: {self._input_topic} -> {self._output_topic}, '
            f'pass-through; degradation service {self._service.srv_name}, '
            f'seed {self._seed}')

    # -- the fault ------------------------------------------------------------
    def _on_set_degradation(self, request, response):
        """Put a degradation in force, or remove the one that is."""
        # The id the caller sent, echoed on every reply: the srv's response
        # carries the id the request was served under, and this relay never
        # mints one of its own - the fault manager owns the id vocabulary.
        response.fault_id = request.fault_id
        if not request.enabled:
            if self._config is None:
                # Removing what was never there is a failure, not a no-op -
                # the same rule ClearObstacle.srv states. A silent success
                # would let a run report a cleared fault that never existed.
                response.success = False
                response.message = 'no scan degradation is in force'
                return response
            previous = self._config
            self._config = None
            response.success = True
            response.message = (
                f'scan degradation removed after {self._scans_degraded} degraded '
                f'scan(s); /scan is a faithful republish of {self._input_topic} '
                f'again (was: noise {previous.range_noise_stddev_m} m, max range '
                f'{previous.max_range_m} m, blind sector '
                f'{previous.blind_sector_start_deg} -> '
                f'{previous.blind_sector_end_deg} deg, dropout '
                f'{previous.dropout_probability})')
            self._scans_degraded = 0
            self.get_logger().warn(response.message)
            return response

        self._config = Degradation(
            range_noise_stddev_m=float(request.range_noise_stddev_m),
            max_range_m=float(request.max_range_m),
            blind_sector_start_deg=float(request.blind_sector_start_deg),
            blind_sector_end_deg=float(request.blind_sector_end_deg),
            dropout_probability=float(request.dropout_probability))
        # Reseed on every enable, so the corrupted stream depends only on the
        # seed and on how many scans have passed SINCE ACTIVATION - not on how
        # many scans the node happened to see before the fault was requested,
        # which varies with bring-up timing and would make a rerun of the same
        # scenario produce different corruption.
        self._rng = random.Random(self._seed)
        self._scans_degraded = 0
        response.success = True
        response.message = (
            f'scan degradation in force on {self._output_topic}: additive Gaussian '
            f'noise sigma {self._config.range_noise_stddev_m} m, ranges truncated '
            f'above {self._config.max_range_m} m, blind sector '
            f'{self._config.blind_sector_start_deg} -> '
            f'{self._config.blind_sector_end_deg} deg (CCW), per-beam dropout '
            f'{self._config.dropout_probability}; RNG reseeded with seed '
            f'{self._seed}, so the corrupted stream repeats for this seed. '
            f'{self._input_topic} stays undegraded for comparison')
        self.get_logger().warn(response.message)
        return response

    def _on_scan(self, msg: LaserScan) -> None:
        if self._config is None:
            # The received object, unchanged. No field copying, no restamping,
            # no filtering: see the module docstring.
            self._pub.publish(msg)
            return

        degraded = LaserScan()
        # Everything the device reported about ITSELF is carried over
        # verbatim, including the stamp: the fault is in the ranges, and a
        # re-stamped scan would also be a (false) timing fault and would break
        # the exact-stamp join metrics.scan_divergence uses to compare the two
        # streams.
        degraded.header = msg.header
        degraded.angle_min = msg.angle_min
        degraded.angle_max = msg.angle_max
        degraded.angle_increment = msg.angle_increment
        degraded.time_increment = msg.time_increment
        degraded.scan_time = msg.scan_time
        degraded.range_min = msg.range_min
        degraded.range_max = msg.range_max
        degraded.intensities = msg.intensities
        degraded.ranges = degrade_ranges(
            msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
            self._config, self._rng)
        self._scans_degraded += 1
        self._pub.publish(degraded)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFaultRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # `ros2 launch` SIGINTs the whole process group; rclpy surfaces that as
        # either of these, depending on where the signal lands.
        pass
    finally:
        # The signal can also arrive during teardown; letting it escape turns
        # every clean shutdown into `process has died, exit code -2`, which
        # destroys the value of that message as a failure signal.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
