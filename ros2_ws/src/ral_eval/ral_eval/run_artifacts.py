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
The writers that make a directory under runs/ a RUN.

Both producers use these: `run_scenario`, which runs a committed scenario
file, and the dashboard gateway, which runs a mission an operator typed. They
have to share, not duplicate: `metrics.write_metrics` hard-depends on the
exact key set of env.json (metrics.py:1010-1057 reads eleven of them by name
and refuses a directory missing any), so two assemblies of that dict would
drift and `write_metrics` would start raising on dashboard runs only - the
runs nobody regenerates.

Nothing here spins a node or talks to ROS: it is file and process work, so it
is importable from the gateway's HTTP thread as well as from the runner.
"""

import hashlib
import json
import os
import signal
import subprocess

from ral_eval.frames import frames_path_for_map, grid_sha256

# Written by docker/build_overlay.sh at image build time: the ArduPilot commit
# the SITL binary in this image was built from. docker/ardupilot.repos pins a
# BRANCH, so without this file a run can say which branch it meant but not
# which code it flew.
FIRMWARE_VERSION_PATH = '/opt/ardupilot/VERSION.txt'


def write_json(path: str, payload: dict) -> None:
    """Atomic write: a reader never sees half an artifact."""
    temporary = path + '.partial'
    with open(temporary, 'w') as handle:
        json.dump(payload, handle, indent=2, sort_keys=False)
        handle.write('\n')
    os.replace(temporary, path)


def sha256_file(path: str) -> str:
    """Hash a file whole; used for every provenance field env.json carries."""
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def firmware_sha() -> 'str | None':
    """
    Read the ArduPilot commit this image was built from, or None when absent.

    None is not a failure: an image built without the autopilot overlay has no
    firmware, and env.json carries the source path beside this value so a
    reader can tell "no autopilot here" from "the file was not written".
    """
    try:
        with open(FIRMWARE_VERSION_PATH) as handle:
            return handle.read().strip() or None
    except OSError:
        return None


def git_sha(cwd: str) -> str:
    """Report HEAD of the repository holding the run, or a stated unknown."""
    try:
        result = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=cwd,
                                capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return 'unknown (git unavailable)'
    if result.returncode != 0:
        return 'unknown (git unavailable)'
    return result.stdout.strip() or 'unknown (git unavailable)'


def git_dirty(cwd: str) -> 'bool | None':
    """
    Report whether the tree has uncommitted changes; None when git is absent.

    A commit SHA recorded beside uncommitted changes is a false provenance
    claim: the code that produced the run is not the code at that commit.
    """
    try:
        result = subprocess.run(['git', 'status', '--porcelain'], cwd=cwd,
                                capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return bool(result.stdout.strip())


def git_diff_sha256(cwd: str) -> 'str | None':
    """
    Fingerprint the uncommitted delta, so a dirty run is still identifiable.

    `git_sha` alone names a commit the code did not come from. This hashes
    `git diff HEAD` concatenated with `git status --porcelain`, which together
    cover tracked modifications and untracked paths. None when git is absent
    or either command fails - stated, never guessed.
    """
    parts = []
    for command in (['git', 'diff', 'HEAD'], ['git', 'status', '--porcelain']):
        try:
            result = subprocess.run(command, cwd=cwd, capture_output=True,
                                    text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if result.returncode != 0:
            return None
        parts.append(result.stdout)
    return hashlib.sha256(''.join(parts).encode()).hexdigest()


def make_run_dir(runs_dir: str, experiment_id: str, started) -> str:
    """
    Create runs/<experiment_id>_<YYYYmmdd>T<HHMMSS>Z and return it.

    NOT exist_ok: the directory name resolves to one second, and a second run
    in the same second would leave the previous run's metrics.json and
    plot.png beside the new run.json. The caller gets FileExistsError and must
    refuse the run rather than mixing two runs' artifacts.

    `dashboard_api._started_key` parses this name, so the format is a
    contract, not a convenience.
    """
    run_dir = os.path.join(runs_dir,
                           f'{experiment_id}_{started.strftime("%Y%m%dT%H%M%SZ")}')
    os.makedirs(run_dir)
    return run_dir


def build_env(*, repo_root: str, image_ref: str, world: str, robot: str, seed: int,
              map_yaml: str, profile: dict, scenario_path: str,
              launch_command: list, record_topics, bag_qos_overrides_path: str,
              nav2_params_path: 'str | None', autopilot_parm_path: 'str | None',
              gate_reference: str, created) -> dict:
    """
    Assemble env.json for one run: the 29 keys metrics.py and the Inspector read.

    Every value is read from the files the run ACTUALLY used - the alignment
    that belongs to this map, the profile text as it was at launch - so a file
    edited afterwards cannot change what the run claims to be.
    """
    alignment_path = frames_path_for_map(map_yaml)
    with open(alignment_path) as handle:
        alignment_text = handle.read()
    with open(profile['profile_path']) as handle:
        profile_text = handle.read()
    dirty = git_dirty(repo_root)
    sha = git_sha(repo_root)
    return {
        'image_ref': image_ref,
        'git_sha': f'{sha}+dirty' if dirty else sha,
        'git_dirty': dirty,
        'git_diff_sha256': git_diff_sha256(repo_root) if dirty else None,
        'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', 'unset'),
        'world': world,
        'robot': robot,
        'seed': seed,
        'map_yaml': map_yaml,
        'map_sha256': grid_sha256(map_yaml),
        'alignment': alignment_text,
        'alignment_path': alignment_path,
        # The profile's own TEXT, as `alignment` carries the alignment file's:
        # metrics.py and plot_run.py read the robot's description out of the
        # RUN, never out of the repository, so a profile edited after a run
        # cannot change what that run's numbers claim to be.
        'robot_profile': profile_text,
        'robot_profile_path': profile['profile_path'],
        'nav2_params_path': nav2_params_path,
        'nav2_params_sha256': sha256_file(nav2_params_path) if nav2_params_path else None,
        # The autopilot half of the same provenance: which parameter file was
        # fed to ardurover, and which ArduPilot source the firmware in this
        # image was built from. docker/ardupilot.repos tracks a floating
        # branch on purpose, so the SHA recorded here is what makes a run
        # traceable to a build rather than to a branch name.
        'autopilot_parm_path': autopilot_parm_path,
        'autopilot_parm_sha256': (sha256_file(autopilot_parm_path)
                                  if autopilot_parm_path else None),
        'firmware_sha': firmware_sha(),
        'firmware_sha_source': (f'{FIRMWARE_VERSION_PATH}, written by '
                                'docker/build_overlay.sh'),
        'scenario_path': scenario_path,
        'scenario_sha256': sha256_file(scenario_path),
        'launch_command': list(launch_command),
        'record_topics': list(record_topics),
        'bag_qos_overrides_path': bag_qos_overrides_path,
        'bag_qos_overrides_sha256': sha256_file(bag_qos_overrides_path),
        'gate_reference': gate_reference,
        'created_utc': created.strftime('%Y-%m-%dT%H:%M:%SZ'),
    }


def start_recorder(run_dir: str, topics, qos_overrides_path: str, log_handle):
    """
    Start `ros2 bag record` into run_dir/bag, writing its output to log_handle.

    Its own session, so the recorder can be signalled without signalling the
    stack: stop_recorder SIGINTs the GROUP, and a shared group would take the
    simulation down with it.
    """
    return subprocess.Popen(
        ['ros2', 'bag', 'record', '-s', 'mcap', '-o', os.path.join(run_dir, 'bag'),
         '--use-sim-time', '--include-hidden-topics',
         '--qos-profile-overrides-path', qos_overrides_path, *topics],
        stdout=log_handle, stderr=subprocess.STDOUT, start_new_session=True)


def stop_recorder(proc, log_handle) -> None:
    """
    SIGINT the recorder and wait for it to finalise the mcap; SIGKILL at 30 s.

    SIGINT is what makes `ros2 bag record` write metadata.yaml; a SIGKILL
    leaves a truncated mcap with no metadata, which metrics.py reports as a
    failed run rather than silently scoring a partial bag.
    """
    if proc is not None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=10)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
        except ProcessLookupError:
            pass
    if log_handle is not None:
        log_handle.close()
