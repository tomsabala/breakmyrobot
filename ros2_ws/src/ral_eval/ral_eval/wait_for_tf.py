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
Block until a TF chain exists AND the clock is advancing, then exit 0.

This gates Nav2's start on a MEASURED precondition instead of a wall timer.
Both halves are required and both were observed failing:

* The timer was too long. MEASURED, logs/stack_roomba_20261026T204117Z.log:
  the robot was spawned at +0.7 s and the Nav2 container was not started
  until +12.2 s - 11.5 s of idle waiting in a ~30 s bring-up.
* The timer was also too SHORT, because it measures nothing. MEASURED,
  logs/stack_roomba_20261026T205614Z.log: it fired into an incomplete TF
  tree, the costmap looped on "Could not find a connection between 'map' and
  'base_link' ... two or more unconnected trees", the container then ignored
  SIGTERM for 10 s and was SIGKILLed (exit -9), and the gateway failed the
  stack with "no messages on /map within 90 s wall".

A node rather than a `ros2 run tf2_ros tf2_echo` or a `TimerAction`: tf2_echo
has no exit condition and a timer asserts nothing. The transform is asked for
at rclpy.time.Time() - "the latest available" - which is exactly what a
costmap asks for, so this gate passes when the consumer would pass.

/clock is checked for TWO messages, not one: one message proves only that a
publisher exists, two prove the simulation is actually advancing. Without it
the gate can pass against a static TF tree while gz is paused, and Nav2 then
starts into a clock that never ticks.

use_sim_time stays FALSE and --timeout is WALL seconds, because at t = 0 sim
time may not exist at all and a sim-time limit could never fire.

Exit codes: 0 both preconditions hold; 1 the timeout expired (stderr names
WHICH half is missing); 2 arguments.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import tf2_ros


def main(argv=None) -> int:
    """Wait for the clock to advance and the TF chain to resolve."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--parent', default='odom',
                        help='the frame the transform is expressed IN')
    parser.add_argument('--child', default='base_link',
                        help='the frame being located')
    parser.add_argument('--clock-samples', type=int, default=2,
                        help='messages on /clock before the clock counts as '
                             'advancing; 1 proves only that a publisher exists')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='WALL seconds. Sim time may not exist yet when this '
                             'starts, so a sim-time limit could never fire.')
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node('wait_for_tf')
    clock_messages = []
    # The gate runs on WALL time by construction (see the module docstring),
    # so it must not adopt sim time even while everything it waits for does.
    node.create_subscription(Clock, '/clock',
                             lambda msg: clock_messages.append(msg), 10)
    buffer = tf2_ros.Buffer()
    # The listener is held so it is not garbage collected while spinning.
    listener = tf2_ros.TransformListener(buffer, node)
    deadline = time.monotonic() + args.timeout
    clock_ready = False
    tf_ready = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            clock_ready = len(clock_messages) >= args.clock_samples
            if not clock_ready:
                continue
            # rclpy.time.Time() means "the latest available transform", which
            # is what a costmap itself asks for.
            tf_ready = buffer.can_transform(args.parent, args.child,
                                            rclpy.time.Time())
            if tf_ready:
                break
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()

    if clock_ready and tf_ready:
        print(f'--- {args.parent} -> {args.child} resolves and /clock carried '
              f'{len(clock_messages)} messages', flush=True)
        return 0
    missing = []
    if not clock_ready:
        missing.append(f'/clock carried {len(clock_messages)} messages, '
                       f'{args.clock_samples} required')
    if not tf_ready:
        missing.append(f'{args.parent} -> {args.child} does not resolve')
    print(f'--- gate expired after {args.timeout:.0f} s wall: '
          + '; '.join(missing), file=sys.stderr, flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
