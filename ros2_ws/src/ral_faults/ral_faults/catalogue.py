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
The only reader of fault_catalogue.yaml.

The file lives in this package's own config/ and is resolved through its
share directory, so a node reads the installed copy that belongs to the build
it is running - never a path relative to a source tree that may not exist in
the container. It lives HERE, in the package whose code reads it: resolving
another package's share directory at runtime is a runtime dependency on it,
and that edge cannot be declared, because ral_bringup already exec_depends
ral_faults and colcon refuses a cycle.

What a ROBOT is - its gz model, its faults, its estimate, its plant limits -
is NOT here. It is in ral_bringup/config/robots/<robot>.yaml, read by
ral_eval/ral_eval/robot_profile.py, whose path a caller supplies. The loader
sits in ral_eval, not in this package, because ral_faults already depends on
ral_eval (package.xml declares both an exec_depend and a test_depend) while
ral_eval depends on neither ral_faults nor ral_bringup: hosting it here would
need ral_eval -> ral_faults and close a cycle. This file describes the fault
VOCABULARY, which is platform-independent; the profile describes one platform.

Every lookup here RAISES on a missing key rather than defaulting. A fault kind
absent from the catalogue is a configuration error: defaulting would publish a
FaultEvent whose injection_point is a guess. The refusal idiom is
robot_profile.load_profile's: name the file and the missing key, and do not
guess.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory

import yaml

CATALOGUE_FILE = 'fault_catalogue.yaml'


def config_path(filename: str) -> str:
    """Absolute path of one config file in this package's share directory."""
    return os.path.join(get_package_share_directory('ral_faults'), 'config', filename)


def _load_yaml(path: str) -> dict:
    with open(path) as handle:
        doc = yaml.safe_load(handle) or {}
    if not isinstance(doc, dict):
        raise ValueError(f'{path}: expected a mapping at the top level, got '
                         f'{type(doc).__name__}')
    return doc


def load_catalogue(path: str = '') -> dict:
    """
    Return {kind: metadata} from fault_catalogue.yaml.

    `path` empty resolves the installed ral_faults copy. Every block must
    carry the full field set; a partially specified kind raises naming the
    field, because the manager puts these strings straight into a FaultEvent
    and a missing one would be published as an empty injection_point.
    """
    resolved = path or config_path(CATALOGUE_FILE)
    doc = _load_yaml(resolved)
    faults = doc.get('faults')
    if not isinstance(faults, dict) or not faults:
        raise ValueError(f'{resolved} has no `faults:` mapping; there is no fault '
                         'vocabulary to serve and this loader will not invent one')
    required = ('category', 'service', 'srv_type', 'injection_point', 'target_rule',
                'lifetime', 'cleared_by', 'alters_truth', 'honesty_note',
                'param_ranges')
    for kind, entry in faults.items():
        if not isinstance(entry, dict):
            raise ValueError(f'{resolved}: fault {kind!r} is not a mapping')
        missing = [field for field in required if field not in entry]
        if missing:
            raise ValueError(f'{resolved}: fault {kind!r} is missing {missing}')
    return faults


def check_params(catalogue: dict, kind: str, params: dict) -> 'str | None':
    """
    None when every value is finite and inside the catalogue's range, else why not.

    The returned string is the refusal an operator reads: it is published as
    the FaultEvent `rejected` detail and answered as the service message.

    Two separate facts are checked. FINITENESS always: a NaN or infinite
    force, pose or sensor parameter is not a request gz or the relay can
    apply, and `max_range_m: 0.0` - the proto default a partially filled
    request carries - turns every finite beam into a no-return and reports it
    as a successful activation. RANGE only where the kind declares one in
    `param_ranges`; an empty block means the kind states no bounds, not that
    finiteness goes unchecked.
    """
    ranges = kind_entry(catalogue, kind).get('param_ranges') or {}
    for name, value in params.items():
        if not math.isfinite(value):
            return (f'{name} is {value!r}, which is not a finite number; a wrench, '
                    'pose or sensor parameter that is NaN or infinite is not a '
                    'request gz or the relay can apply')
        bounds = ranges.get(name)
        if not bounds:
            continue
        low, high = bounds
        if (low is not None and value < low) or (high is not None and value > high):
            return (f'{name} is {value}; fault_catalogue.yaml declares '
                    f'{kind}.param_ranges.{name} as [{low}, {high}] and a request '
                    'outside it is refused rather than applied')
    return None


def kind_entry(catalogue: dict, kind: str) -> dict:
    """One kind's metadata, or a ValueError naming the kinds that do exist."""
    try:
        return catalogue[kind]
    except KeyError:
        raise ValueError(
            f'{kind!r} is not in the fault catalogue; known kinds are '
            f'{sorted(catalogue)}') from None


def target_for(catalogue: dict, kind: str, params: dict, model: str) -> str:
    """
    FaultEvent.target for one request, by the catalogue's `target_rule`.

    This reproduces run_scenario._target_of; test_mission_vocabulary pins the
    two against each other so the manager and the runner cannot label the same
    fault differently in the same bag.
    """
    rule = kind_entry(catalogue, kind)['target_rule']
    if rule == 'params.name':
        return str(params['name'])
    if rule == 'model':
        return model
    if rule.startswith('literal:'):
        return rule.split(':', 1)[1]
    raise ValueError(f'{kind}: unknown target_rule {rule!r} in the fault catalogue')
