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
The only reader of a robot profile.

A profile is ral_bringup/config/robots/<robot>.yaml: one file per platform,
carrying the gz model name, the spawn pose, the autonomy stack that drives it,
what its position estimate IS, the plant limits, the footprint, the faults it
supports and the topics a run records. It replaced ral_faults'
config/capabilities.yaml, which carried the capability half of this and left
the other half spread across module constants in four packages.

The profiles live in ral_bringup, not here, because they are read by the
launch layer, the scenario runner, the metrics and the plot as well as by the
fault manager - ral_bringup is the package that owns bring-up configuration.

The LOADER lives in ral_eval because the dependency graph leaves it nowhere
else: ral_bringup exec_depends both ral_eval and ral_faults, ral_faults
already depends on ral_eval (ral_faults/package.xml declares
`<exec_depend>ral_eval</exec_depend>` for fault_manager's own import of this
module, and `<test_depend>ral_eval</test_depend>` for the catalogue test that
pins against run_scenario), and ral_eval depends on neither. Putting it in
ral_faults would require ral_eval -> ral_faults, which closes a cycle colcon
refuses to order.

The consequence is the rule below: this module NEVER resolves ral_bringup's
share directory. A caller passes the path, and a node gets it from a launch
parameter - `profile_path` is the resolver for a caller that already knows
that share directory.

Every lookup RAISES on a missing key rather than defaulting. An absent key is
a configuration error: defaulting would either address the wrong gz entity,
label a metric with an estimator that did not produce it, or let a platform be
subjected to a fault nobody declared it supports. The refusal idiom is
ral_faults' catalogue.py's: name the file and the missing key, and do not
guess.
"""

import os

import yaml

# Every key a profile must carry, for every robot.
REQUIRED_KEYS = ('robot', 'model', 'description_model', 'stack_launch', 'bridge_config',
                 'spawn', 'estimate', 'command', 'mission_driver', 'plant', 'footprint',
                 'goal_tolerance_m', 'kidnap_z_m', 'bringup_wall_limit_s',
                 'bringup_topics', 'record_topics', 'faults', 'estimator_label')

# Keys a profile MAY carry. The list is closed and checked: `unsupported` and
# `recorder_gate_topics` used to be read with `doc.get(...) or {}`, so a
# misspelling silently disabled the capability text or the recorder gate with
# no error at all. Anything outside REQUIRED_KEYS + OPTIONAL_KEYS is a typo
# until the schema says otherwise.
#   unsupported                  a robot that supports every fault has nothing
#                                to explain
#   recorder_gate_topics         a platform may have no pre-mission topic worth
#                                gating on
#   mobility_note                a plant whose mobility response has not been
#                                measured must say so in its own words rather
#                                than inherit another plant's numbers
#   arrival_estimate_tolerance_m required for mission_driver mavros_guided
#                                (see below), meaningless for a stack whose
#                                own goal checker owns arrival
#   autopilot_parm               the autopilot parameter file this platform is
#                                started with, hashed into env.json
OPTIONAL_KEYS = ('unsupported', 'recorder_gate_topics', 'mobility_note',
                 'arrival_estimate_tolerance_m', 'autopilot_parm')

REQUIRED_SPAWN_KEYS = ('x', 'y', 'z', 'yaw')
REQUIRED_ESTIMATE_KEYS = ('topic', 'type', 'frame', 'label', 'tf_source', 'uncertainty')
REQUIRED_COMMAND_KEYS = ('kind', 'topic', 'label')
REQUIRED_PLANT_KEYS = ('max_linear_m_s', 'max_angular_rad_s', 'min_turn_radius_m')


def profile_path(bringup_share: str, robot: str) -> str:
    """
    Return <bringup_share>/config/robots/<robot>.yaml.

    No existence check: the caller is about to open it, and a "does it exist"
    guess here would only turn one clear OSError into two error paths.
    """
    return os.path.join(bringup_share, 'config', 'robots', f'{robot}.yaml')


def load_profile(path: str) -> dict:
    """
    One robot's profile, fully validated.

    `path` is required - this module cannot resolve ral_bringup's share
    directory (see the module docstring), so there is no default to fall back
    to. Raises ValueError naming the file and the missing key for anything
    incomplete, and for a `robot:` key that disagrees with the file stem: the
    stem is what every caller looks the file up by, so a disagreement means
    one of the two is addressing a platform that is not the one described.
    """
    with open(path) as handle:
        doc = yaml.safe_load(handle) or {}
    if not isinstance(doc, dict):
        raise ValueError(f'{path}: expected a mapping at the top level, got '
                         f'{type(doc).__name__}')

    missing = [key for key in REQUIRED_KEYS if key not in doc]
    if missing:
        raise ValueError(f'{path}: robot profile is missing {missing}; every key is '
                         'required for every robot, because an absent one would be '
                         'filled in by a guess about the platform')

    unknown = [key for key in doc if key not in REQUIRED_KEYS + OPTIONAL_KEYS]
    if unknown:
        raise ValueError(f'{path}: unknown top-level key(s) {sorted(unknown)}; the '
                         'schema is closed on purpose. The optional keys used to be '
                         'read with `.get(...) or <empty>`, so a misspelled '
                         '`unsupported` or `recorder_gate_topics` silently disabled '
                         'a capability refusal or the recorder gate. Known keys: '
                         f'{sorted(REQUIRED_KEYS + OPTIONAL_KEYS)}')

    stem = os.path.splitext(os.path.basename(path))[0]
    if str(doc['robot']) != stem:
        raise ValueError(f'{path}: the `robot` key is {doc["robot"]!r} but the file '
                         f'stem is {stem!r}; callers look this profile up by stem, so '
                         'the two naming different platforms means one of them is '
                         'addressing the wrong robot')

    _require_sub(path, doc, 'spawn', REQUIRED_SPAWN_KEYS)
    _require_sub(path, doc, 'estimate', REQUIRED_ESTIMATE_KEYS)
    _require_sub(path, doc, 'command', REQUIRED_COMMAND_KEYS)
    _require_sub(path, doc, 'plant', REQUIRED_PLANT_KEYS)
    # `source` is read by run_scenario when it records what the footprint IS,
    # so an absent one is a run that cannot cite its own clearance budget.
    _require_sub(path, doc, 'footprint', ('radius_m', 'source'))

    uncertainty = doc['estimate']['uncertainty']
    if not isinstance(uncertainty, dict) or 'kind' not in uncertainty:
        raise ValueError(f'{path}: estimate.uncertainty has no `kind`; metrics.py '
                         'decides which uncertainty metric this platform can report '
                         'from it and will not infer one from the message type')
    # The subkeys the drivers and metrics actually read. Without these the run
    # dies as a bare KeyError AFTER the run directory exists, which looks like
    # a crash rather than the configuration error it is.
    #
    # `topic` is required for estimator_status only: MavrosGuidedDriver
    # subscribes to it by name and metrics.py reads the flags from it. A
    # covariance_trace platform has no separate uncertainty topic to require -
    # the covariance rides on the estimate message itself, which is what
    # metrics.py's est_cov_trace_max reads.
    if uncertainty['kind'] == 'estimator_status' and 'topic' not in uncertainty:
        raise ValueError(
            f'{path}: estimate.uncertainty.kind is \'estimator_status\' but carries '
            'no `topic`; the mission driver subscribes to it by name and the '
            'recorder records it')
    if uncertainty['kind'] == 'covariance_trace' and 'converged_below' not in uncertainty:
        raise ValueError(
            f'{path}: estimate.uncertainty.kind is \'covariance_trace\' but carries '
            'no `converged_below`; Nav2Driver decides whether localization has '
            'converged from it and will not invent a threshold')

    if doc['mission_driver'] == 'mavros_guided':
        if 'arrival_estimate_tolerance_m' not in doc:
            raise ValueError(
                f'{path}: a mavros_guided profile must carry '
                'arrival_estimate_tolerance_m; the autopilot reports its distance to '
                'target in whole metres, so the runner cannot tell arrival from a '
                'truncation without a gate on the robot\'s own estimate')
        if 'autopilot_parm' not in doc:
            raise ValueError(
                f'{path}: a mavros_guided profile must carry autopilot_parm; the '
                'autopilot is started with --defaults <that file>, so a run that '
                'cannot name and hash it cannot say which configuration flew it')

    if not isinstance(doc['faults'], list):
        raise ValueError(f'{path}: `faults` is not a list; an absent or malformed '
                         'list is not "supports everything"')

    doc['unsupported'] = dict(doc.get('unsupported') or {})
    doc['recorder_gate_topics'] = list(doc.get('recorder_gate_topics') or [])
    doc['profile_path'] = path
    return doc


def _require_sub(path: str, doc: dict, key: str, required) -> None:
    block = doc[key]
    if not isinstance(block, dict):
        raise ValueError(f'{path}: `{key}` is not a mapping')
    missing = [name for name in required if name not in block]
    if missing:
        raise ValueError(f'{path}: `{key}` is missing {missing}')


def supports(profile: dict, kind: str) -> 'str | None':
    """
    None when this robot supports `kind`; otherwise the refusal reason.

    The reason comes VERBATIM from the profile's `unsupported` block when it
    has one - that text is written for an operator and is what the FaultEvent
    and the service reply carry. A kind simply absent from `faults`, with no
    stated reason, still refuses: the profile is the declaration, and silence
    in it is not consent.
    """
    if kind in profile['faults']:
        return None
    stated = profile['unsupported'].get(kind)
    if stated:
        return str(stated)
    basename = os.path.basename(profile.get('profile_path', 'the robot profile'))
    return (f'{kind} is not listed under {profile["robot"]} in {basename}; no reason '
            'is recorded, and silence in the table is not consent')
