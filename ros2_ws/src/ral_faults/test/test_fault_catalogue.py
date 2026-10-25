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
The fault catalogue and the capability table, pinned against their consumers.

config/fault_catalogue.yaml is what fault_manager publishes every FaultEvent
from, and config/capabilities.yaml is the gate every request passes. The
runner is what sends the request. A drift between them does not raise: it
sends a fault to a service nobody serves, or labels an injection point that
was not the one altered - and both read as a working run. This file is where
that drift fails instead.

It lives in ral_faults because that is the package whose code READS these
files and whose share directory installs them. The edge it needs is one-way
and acyclic: ral_faults test_depends ral_eval, and ral_eval depends on
neither ral_faults nor ral_bringup. The reverse - ral_eval importing
ral_faults - would make colcon refuse to order the workspace at all ("Unable
to order packages topologically"), because ral_bringup already exec_depends
both.

The files are read with plain yaml, by SOURCE path rather than through
ral_faults.catalogue: a loader that silently defaulted a missing field could
not hide the drift this test exists to catch.
"""

import os

from ral_eval import run_scenario

from ral_eval.scenario import INTERVENTION_PARAM_RANGES, INTERVENTION_PARAMS

from ral_interfaces.msg import FaultEvent

import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'config')


def _config(filename: str) -> dict:
    with open(os.path.join(CONFIG_DIR, filename)) as handle:
        return yaml.safe_load(handle) or {}


def test_the_fault_catalogue_covers_exactly_the_kinds_the_runner_dispatches():
    catalogue = _config('fault_catalogue.yaml')['faults']
    kinds = {getattr(FaultEvent, name) for name in dir(FaultEvent)
             if name.startswith('KIND_')}
    assert kinds, 'FaultEvent exposes no KIND_* constants'
    assert set(catalogue) == kinds
    assert set(catalogue) == set(INTERVENTION_PARAMS)


def test_the_catalogue_agrees_with_the_runner_on_service_and_injection_point():
    """One drift here sends a fault to a service nobody serves, or mislabels it."""
    catalogue = _config('fault_catalogue.yaml')['faults']
    for kind, entry in catalogue.items():
        assert entry['service'] == run_scenario.DISPATCH[kind][0], kind
        assert entry['injection_point'].format(world='ral_apartment') \
            == run_scenario.injection_point(kind, 'ral_apartment'), kind


def test_every_catalogue_entry_carries_the_fields_the_manager_publishes():
    """A missing field would be published as an empty FaultEvent string."""
    catalogue = _config('fault_catalogue.yaml')['faults']
    required = ('category', 'service', 'srv_type', 'injection_point', 'target_rule',
                'lifetime', 'cleared_by', 'alters_truth', 'honesty_note',
                'param_ranges')
    for kind, entry in catalogue.items():
        assert set(required) <= set(entry), (kind, sorted(set(required) - set(entry)))
        assert entry['lifetime'] in ('keyed_set', 'singleton', 'oneshot'), kind
        assert entry['category'] in ('environment', 'dynamics', 'state', 'perception'), \
            kind
        # A one-shot changes the world once; offering a "clear" control for it
        # would promise an undo that does not exist.
        if entry['lifetime'] == 'oneshot':
            assert entry['cleared_by'] is None, kind


def test_the_catalogue_bounds_agree_with_the_scenario_schemas_bounds():
    """
    One table of bounds, read twice.

    scenario.py refuses an out-of-range value when a scenario FILE is loaded;
    the manager refuses it at the SERVICE, which is where every non-scenario
    caller arrives. Two tables that disagreed would accept a run the other
    rejects, so the catalogue is the source and this pins the copy.
    """
    catalogue = _config('fault_catalogue.yaml')['faults']
    for kind, entry in catalogue.items():
        declared = entry['param_ranges']
        assert isinstance(declared, dict), kind
        expected = INTERVENTION_PARAM_RANGES.get(kind, {})
        assert set(declared) == set(expected), kind
        for field, (low, high) in expected.items():
            assert tuple(declared[field]) == (low, high), (kind, field)


def test_the_catalogue_target_rules_reproduce_the_runners_target_field():
    """FaultEvent.target must read the same whoever published it."""
    catalogue = _config('fault_catalogue.yaml')['faults']
    model = _config('capabilities.yaml')['roomba']['model']
    params = {'name': 'blocker_1'}
    for kind, entry in catalogue.items():
        rule = entry['target_rule']
        if rule == 'params.name':
            expected = params['name']
        elif rule == 'model':
            expected = model
        else:
            expected = rule.split(':', 1)[1]
        assert expected == run_scenario._target_of(kind, params), kind


def test_every_fault_a_robot_claims_to_support_exists_in_the_catalogue():
    """A capability naming a kind with no catalogue entry would be unservable."""
    catalogue = _config('fault_catalogue.yaml')['faults']
    capabilities = _config('capabilities.yaml')
    assert capabilities, 'capabilities.yaml declares no robots'
    for robot, entry in capabilities.items():
        assert entry.get('model'), f'{robot} has no gz model name'
        for kind in entry['faults']:
            assert kind in catalogue, f'{robot} claims {kind}, which is not catalogued'
        for kind in (entry.get('unsupported') or {}):
            assert kind in catalogue, f'{robot} refuses {kind}, which is not catalogued'
            assert kind not in entry['faults'], \
                f'{robot} both supports and refuses {kind}'


def test_a_refused_fault_states_a_reason_an_operator_can_read():
    """The dashboard and the FaultEvent carry this string verbatim."""
    capabilities = _config('capabilities.yaml')
    catalogue = _config('fault_catalogue.yaml')['faults']
    for robot, entry in capabilities.items():
        for kind in set(catalogue) - set(entry['faults']):
            reason = (entry.get('unsupported') or {}).get(kind)
            assert reason, (
                f'{robot} does not support {kind} and records no reason; an operator '
                'is told "no" with nothing to read')
