"""
The assertions both proof harnesses make, in one copy.

`roomba_nav_check.py` and `car_nav_check.py` bring up two entirely different
autonomy stacks — Nav2 and AMCL for the Roomba, ArduRover SITL and MAVROS for
the car — but several of the properties they assert belong to THIS PROJECT
rather than to either stack: that simulator truth is consumed by nobody but the
harness and the fault injector, that `/scan` has exactly one publisher and it is
the fault relay, that the TF tree contains the frames the robot's own estimate
produces and nothing else. Those assertions live here so there is exactly one
implementation of each. Two copies would drift, and the copy that drifted would
be the one that quietly stopped being able to fail.

What stays in each harness is everything robot-specific: its constants, its
numbered `record()` calls, its mission, its doc text, its limitations. What is
shared is only the mechanism — start the stack, read the live graph, compare,
tear down, write the document.

Each check helper returns `(passed, detail)` and records nothing itself: the
harness owns the numbering, and a number means the same thing across the two
documents only because the harness, not this module, assigns it.

This is NOT a test module. Nothing in `colcon test` imports it and pytest does
not collect it: every helper here needs a running simulator. The build-time
guards on the same properties are `ral_bringup`'s `test_bridge_contract.py` and
`test_nav2_contract.py`, which need no simulator at all.
"""

import os
import re
import signal
import subprocess

import yaml

# Nodes that must never subscribe to simulator truth (ENGINEERING.md section
# 4). Matched as substrings against live node names. The list names the Nav2
# stack because that is the stack whose nodes are numerous enough to hide one;
# a platform whose autonomy runs elsewhere (the car's runs inside the
# autopilot) is covered by the allowlist half of the same check.
NAV_NODE_PATTERNS = ('amcl', 'controller_server', 'planner_server', 'bt_navigator',
                     'costmap', 'velocity_smoother', 'slam_toolbox', 'behavior_server',
                     'smoother_server', 'waypoint_follower')


def launch_stack(launch_file, log_path, extra_args=()):
    """
    Start one `ral_bringup` launch file in its own process group.

    Returns `(process, log_handle)`. The caller keeps the handle open for the
    lifetime of the stack and closes it afterwards, so the launch log is
    complete on disk before the document that points at it is written.

    `start_new_session=True` is what makes `teardown` able to signal the whole
    tree: `ros2 launch` forks gz, the bridge, the nodes and — for the car —
    the autopilot binary, none of which die with the parent.
    """
    log = open(log_path, 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'ral_bringup', launch_file, *extra_args],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return launch, log


def teardown(launch):
    """
    Stop a launched stack: SIGINT to the group, then SIGKILL if it will not go.

    SIGINT first because `ros2 launch` shuts its children down cleanly on it,
    which is what flushes the launch log and lets gz release the UDP ports the
    SITL instance holds. The kill is the deadlock guard, not the plan.
    """
    try:
        os.killpg(os.getpgid(launch.pid), signal.SIGINT)
        launch.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(launch.pid), signal.SIGKILL)
        launch.wait(timeout=10)
    except ProcessLookupError:
        pass


def measure_rtf(world):
    """Read the real-time factor gz reports for this world."""
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{world}/stats', '-n', '3'],
            capture_output=True, text=True, timeout=30).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return f'unavailable ({type(exc).__name__})'
    values = re.findall(r'real_time_factor:\s*([0-9.]+)', out)
    if not values:
        return 'unavailable (no real_time_factor in /world stats)'
    return ', '.join(values)


def bumper_liveness_check(node, world, model):
    """
    (alive, detail) for /bumper: a ROS publisher AND the gz contact topic.

    A contact sensor publishes only on contact, so `0 reports` cannot be
    told from a dead bridge by counting. This is the same proof
    ral_eval's run_scenario.bumper_liveness makes, re-implemented here
    because a verifier that imports the code under test shares its bugs.
    """
    publishers = sorted(info.node_name
                        for info in node.get_publishers_info_by_topic('/bumper'))
    gz_topic = f'/world/{world}/model/{model}/link/base_link/sensor/bumper/contact'
    try:
        listed = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                                text=True, timeout=30).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return False, (f'/bumper ROS publishers = {publishers}; gz topic -l is '
                       f'unavailable ({type(exc).__name__}), so the gz half of the '
                       'contact path could not be proven')
    gz_present = gz_topic in listed
    if gz_present:
        gz_detail = f'gz contact topic {gz_topic} is listed by `gz topic -l`'
    else:
        matches = [line.strip() for line in listed.splitlines()
                   if line.strip().endswith('/sensor/bumper/contact')]
        gz_detail = (f'gz contact topic {gz_topic} is NOT listed by `gz topic -l`; '
                     f'contact topics found: {matches or "none"}')
    alive = bool(publishers) and gz_present
    return alive, (f'/bumper ROS publishers = {publishers}; {gz_detail}. Both halves '
                   'are required: a contact sensor is silent until something touches '
                   'it, so a count of zero reports is only evidence when the path '
                   'that would have carried them is shown to exist')


def scan_publisher_check(node):
    """
    `/scan` has exactly one publisher and it is `scan_fault_relay`.

    A second publisher is the subtle version of routing a lidar consumer
    around the fault boundary: the stack comes up normally and every lidar
    fault reaches only some of the consumers. `/scan_raw` is checked for a
    single publisher too — the sensor itself, through the bridge.
    """
    scan_pubs = node.get_publishers_info_by_topic('/scan')
    raw_pubs = node.get_publishers_info_by_topic('/scan_raw')
    scan_names = sorted(pub.node_name for pub in scan_pubs)
    raw_names = sorted(pub.node_name for pub in raw_pubs)
    passed = scan_names == ['scan_fault_relay'] and len(raw_names) == 1
    return passed, (
        f'/scan publishers = {scan_names}, /scan_raw publishers = {raw_names}; '
        'a second publisher or a different node means lidar faults would not '
        'reach every consumer')


def truth_subscriber_check(node, allowlist):
    """
    Only the allowlisted nodes subscribe to `/ground_truth/odom`.

    An allowlist, not just a nav-pattern scan: a pattern list only catches the
    offenders somebody thought of, and anything outside the allowlist is a new
    truth consumer nobody declared. Both halves are reported, because a
    navigation component reading truth is a different failure from an
    undeclared one and the detail has to say which happened.
    """
    truth_subs = node.get_subscriptions_info_by_topic('/ground_truth/odom')
    truth_sub_names = sorted(sub.node_name for sub in truth_subs)
    offenders = [name for name in truth_sub_names
                 if any(pattern in name for pattern in NAV_NODE_PATTERNS)]
    unexpected = [name for name in truth_sub_names if name not in allowlist]
    passed = not offenders and not unexpected
    return passed, (
        f'/ground_truth/odom subscribers = {truth_sub_names}; navigation '
        f'offenders = {offenders}; not allowlisted = {unexpected}')


def tf_topology_check(node, expected_frames, topology):
    """
    The WHOLE TF frame set, plus the parent of every frame that must have one.

    Asserting the whole set is what makes this a truth-leak detector: simulator
    truth bridged onto `/tf` arrives under the gz PosePublisher's own names
    (`<world>` -> `<model>`), which an absence test on a known truth frame name
    would never see. `expected_frames` is therefore a closed set, and a frame
    nobody declared fails the check even if the declared tree is intact.

    `topology` maps child -> required parent.
    """
    frames_yaml = yaml.safe_load(node.tf_buffer.all_frames_as_yaml()) or {}
    parents = {name: info.get('parent') for name, info in frames_yaml.items()}
    present = set(parents) | {p for p in parents.values() if p}
    unexpected = present - expected_frames
    missing_frames = expected_frames - present
    wrong = {child: (parents.get(child), parent)
             for child, parent in topology.items()
             if parents.get(child) != parent}
    passed = not unexpected and not missing_frames and not wrong
    return passed, (
        f'frames = {sorted(present)}; unexpected = {sorted(unexpected)}, '
        f'missing = {sorted(missing_frames)}, wrong parents (got, want) = '
        f'{wrong or "none"}')


def write_doc(path, lines):
    """Write a generated document and say where it went."""
    with open(path, 'w') as handle:
        handle.write('\n'.join(lines))
    print(f'--- wrote {path}', flush=True)
