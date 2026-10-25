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
The only reader of fault_catalogue.yaml and capabilities.yaml.

Both files live in this package's own config/ and are resolved through its
share directory, so a node reads the installed copy that belongs to the build
it is running - never a path relative to a source tree that may not exist in
the container. They live HERE, in the package whose code reads them: resolving
another package's share directory at runtime is a runtime dependency on it,
and that edge cannot be declared, because ral_bringup already exec_depends
ral_faults and colcon refuses a cycle.

Every lookup here RAISES on a missing key rather than defaulting. A fault kind
absent from the catalogue, or a robot absent from the capability table, is a
configuration error: defaulting would either publish a FaultEvent whose
injection_point is a guess, or let a platform be subjected to a fault nobody
declared it supports. The refusal idiom is run_scenario.read_robot_radius's
(:835-857): name the file and the missing key, and do not guess.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory

import yaml

CATALOGUE_FILE = 'fault_catalogue.yaml'
CAPABILITIES_FILE = 'capabilities.yaml'


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


def load_capabilities(path: str = '') -> dict:
    """
    Return {robot: {'model', 'faults', 'unsupported'}} from capabilities.yaml.

    `unsupported` is normalised to a dict so callers never have to handle its
    absence; `model` and `faults` are required, because the manager addresses
    gz by model name and gates every request on the fault list.
    """
    resolved = path or config_path(CAPABILITIES_FILE)
    doc = _load_yaml(resolved)
    table = {}
    for robot, entry in doc.items():
        if not isinstance(entry, dict):
            raise ValueError(f'{resolved}: robot {robot!r} is not a mapping')
        if 'model' not in entry:
            raise ValueError(f'{resolved}: robot {robot!r} has no `model`; the fault '
                             'manager addresses gz entities by model name and will '
                             'not guess one')
        faults = entry.get('faults')
        if not isinstance(faults, list):
            raise ValueError(f'{resolved}: robot {robot!r} has no `faults` list; an '
                             'absent list is not "supports everything"')
        table[robot] = {
            'model': str(entry['model']),
            'faults': [str(kind) for kind in faults],
            'unsupported': dict(entry.get('unsupported') or {}),
        }
    if not table:
        raise ValueError(f'{resolved} declares no robots')
    return table


def supports(capabilities: dict, robot: str, kind: str) -> 'str | None':
    """
    None when `robot` supports `kind`; otherwise the refusal reason.

    The reason comes VERBATIM from the robot's `unsupported` block when it has
    one - that text is written for an operator and is what the FaultEvent and
    the service reply carry. A kind that is simply absent from the list, with
    no stated reason, still refuses: the table is the declaration, and silence
    in it is not consent.
    """
    entry = capabilities.get(robot)
    if entry is None:
        return (f'robot {robot!r} is not in capabilities.yaml (known: '
                f'{sorted(capabilities)}); no fault can be gated for a platform '
                'whose capabilities are undeclared')
    if kind in entry['faults']:
        return None
    stated = entry['unsupported'].get(kind)
    if stated:
        return str(stated)
    return (f'{kind} is not listed under {robot} in capabilities.yaml, and no '
            'reason is recorded there; an undeclared fault is refused rather '
            'than attempted')


def model_of(capabilities: dict, robot: str) -> str:
    """Return the gz model name for `robot`, or raise naming the file."""
    entry = capabilities.get(robot)
    if entry is None:
        raise ValueError(f'robot {robot!r} is not in capabilities.yaml; known robots '
                         f'are {sorted(capabilities)}')
    return entry['model']


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
