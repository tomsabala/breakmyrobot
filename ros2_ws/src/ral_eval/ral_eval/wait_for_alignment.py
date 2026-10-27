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
Block until /frame_alignment carries one message, then exit 0.

This exists because the car's global planner may not be ACTIVATED before the
EKF origin has been measured: `map -> ekf_local` does not exist until
ekf_local_tf publishes it (+49.4 / +49.7 / +55.1 s wall across three measured
runs, car.yaml:148-156), and a costmap activated earlier aborts with "Timed
out waiting for transform".

It replaces `ros2 topic echo --once /frame_alignment`, which was tried first
and proved unreliable: MEASURED in logs/stack_car_20261026T150610Z.log, the
CLI returned non-zero while the topic was healthy, the lifecycle manager was
never started, and the whole stack then failed bring-up with "the global
planner never activated". A subscriber with the publisher's own QoS
(TRANSIENT_LOCAL, so a late subscriber still receives the single latched
message) has no such failure mode.

Exit codes: 0 one message received; 1 the timeout expired; 2 arguments.
"""

import argparse
import sys
import time

from ral_interfaces.msg import FrameAlignment
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main(argv=None) -> int:
    """Wait for one /frame_alignment message, or report the timeout."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/frame_alignment')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='WALL seconds. The measurement happens before the '
                             'simulation is useful, so a sim-time limit would not '
                             'fire.')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('wait_for_frame_alignment')
    received = []
    # The publisher's own QoS: ekf_local_tf latches this message
    # (ekf_local_tf.py:208-212), and a VOLATILE subscriber that joins after it
    # was published would wait for a second message that never comes.
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(FrameAlignment, args.topic,
                             lambda msg: received.append(msg), qos)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if received:
        message = received[0]
        print(f'--- {args.topic}: {message.parent_frame} -> {message.child_frame} '
              f'at ({message.x:.3f}, {message.y:.3f}, {message.yaw:.3f})', flush=True)
        return 0
    print(f'--- {args.topic}: no message within {args.timeout:.0f} s wall',
          file=sys.stderr, flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
