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

In Phase 2 this node is a faithful pass-through: whatever arrives on
/scan_raw is republished on /scan unchanged. Phase 4.2 adds a
SetScanDegradation service HERE, at this boundary, so that a sensor fault
changes the data a real lidar driver would deliver to its consumers - not a
dashboard field and not a number the UI prints. A fault that only moves UI
state is forbidden by ENGINEERING.md and by CLAUDE.md's non-negotiables.

The disabled path must stay a byte-identical republish of the incoming
message. A relay that restamps, reorders, re-synthesises or rounds ranges
corrupts every non-fault run in the project silently: the baseline the fault
runs are compared against would already be wrong, and the comparison would
attribute the relay's own distortion to the injected fault. The check that
enforces this end-to-end is check 3 in scripts/checks/roomba_nav_check.py
(same header.stamp on both topics => identical ranges and frame_id).

There is deliberately no unit test here. "The republished copy equals the
input" is a tautology when the implementation is `pub.publish(msg)`; the
property that actually matters - /scan has exactly one publisher and it is
this node, and its content matches /scan_raw on the live graph - can only be
asserted against a running system, which is what checks 2 and 3 of the proof
harness do.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFaultRelay(Node):
    """Republishes the raw lidar stream on the topic the robot's stack consumes."""

    def __init__(self) -> None:
        super().__init__('scan_fault_relay')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

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

        self.get_logger().info(
            f'scan relay: {self._input_topic} -> {self._output_topic}, '
            'pass-through (no degradation implemented in Phase 2)')

    def _on_scan(self, msg: LaserScan) -> None:
        # The received object, unchanged. No field copying, no restamping, no
        # filtering: see the module docstring.
        self._pub.publish(msg)


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
