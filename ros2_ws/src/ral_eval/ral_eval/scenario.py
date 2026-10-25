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
The one place a scenario file is interpreted.

Pure Python - yaml and math, no rclpy - so the schema can be exercised
without a ROS graph or a simulator, which is what makes the rejection cases
testable at all.

Everything here refuses rather than repairs. A scenario is the tracked input
that makes a run reproducible, so a value this module cannot understand is an
error naming the file, the key and the offending value; it is never coerced
into something plausible. The specific failure that justifies the strictness
is `world:` - falling back to ral_apartment for an unknown world would score
a run against a map that does not belong to it, and every estimate-error
number would be a frame offset reported as localization error.

Poses in a scenario are in the gz WORLD frame. This module does not convert
them: `mission_poses` returns BOTH forms so the runner can record the input
verbatim beside the values it actually sent.
"""

import math
import os

from ral_eval.frames import world_pose_to_map

import yaml

# One legal world until a second one is authored. A second world is not just
# another SDF: it needs its own bridge_<robot>_<world>.yaml (three gz topic
# paths embed the world name), its own slam_toolbox map and its own
# <world>_frames.yaml alignment.
ALLOWED_WORLDS = ('ral_apartment',)

# The robots that are actually INTEGRATED: each has a profile in
# ral_bringup/config/robots/<robot>.yaml, a gz model, a bridge config and a
# launch file that brings its own autonomy stack up. Accepting a name with no
# profile would run one platform and label the result as another.
ALLOWED_ROBOTS = ('roomba', 'car')

# type -> {param name: required Python type}. This table is also the set of
# intervention types the runner's DISPATCH must cover; test_mission_vocabulary
# asserts the two agree, and that both agree with FaultEvent's KIND_*
# constants.
INTERVENTION_PARAMS = {
    'spawn_obstacle': {'name': str, 'pose': dict, 'size': dict},
    'clear_obstacle': {'name': str},
    'scan_degradation': {'enabled': bool, 'range_noise_stddev_m': float,
                         'max_range_m': float, 'blind_sector_start_deg': float,
                         'blind_sector_end_deg': float, 'dropout_probability': float},
    'mobility_fault': {'enabled': bool, 'force_x_n': float, 'force_y_n': float,
                       'torque_z_nm': float},
    'kidnap': {'pose': dict},
}

# Physically meaningful bounds for the numeric fault parameters, as
# (minimum, maximum); None means unbounded on that side. A type check alone
# accepts `dropout_probability: 5.0` and `range_noise_stddev_m: -3.0`, which
# no fault manager can apply. Bounds come from the modelled hardware: the
# RPLIDAR A1 as modelled on ral_roomba
# (ral_description/models/ral_roomba/model.sdf:109-113 gives it a 0.15-12.0 m
# range, and :105-106 a full -pi..pi scan). They apply to the only platform
# that can receive the fault, but NOT because this module checks: schema
# validation here never sees the robot, and `_validate_interventions` takes
# no profile. The gate is ral_faults/fault_manager.py's, at RUNTIME, and
# deliberately so - refusing there is what produces the evidence, a `rejected`
# FaultEvent in the bag carrying the profile's verbatim reason. A scenario
# that asks for scan_degradation on the car is therefore accepted, dispatched
# and refused on the record, which is the honest outcome; the car's profile
# does not list the fault because ral_car carries the same lidar but nothing
# in its stack consumes it.
INTERVENTION_PARAM_RANGES = {
    'scan_degradation': {
        'range_noise_stddev_m': (0.0, None),
        'max_range_m': (0.15, 12.0),
        'blind_sector_start_deg': (-180.0, 180.0),
        'blind_sector_end_deg': (-180.0, 180.0),
        'dropout_probability': (0.0, 1.0),
    },
}
# Deliberately unbounded: mobility_fault's force and torque (any finite wrench
# is a wrench gz can apply, and an implausibly large one is a visible result
# rather than an invalid request) and spawn_obstacle.size, which keeps only its
# existing `> 0.0` check for the same reason. Do not add bounds here by
# guesswork; a bound belongs here only when a modelled device states it.

REQUIRED_KEYS = ('experiment_id', 'robot', 'world', 'seed', 'initial_pose', 'goal',
                 'timeout_sec')

# doc.get() cannot distinguish an absent key from `interventions:` with no
# value, and those are different statements about the run.
_ABSENT = object()

# experiment_id becomes a directory name under runs/, so it may not carry
# spaces, slashes or case that differs only by shell completion.
_ID_ALLOWED = set('abcdefghijklmnopqrstuvwxyz0123456789_')
_ID_LEADING = set('abcdefghijklmnopqrstuvwxyz0123456789')


class ScenarioError(ValueError):
    """A scenario that cannot be run as written. The message names the key."""


def _is_finite_number(value) -> bool:
    # bool is an int subclass; a YAML `yaw: true` is not an angle. NaN and
    # infinity are numbers that cannot be executed: `timeout_sec: .nan` makes
    # `elapsed >= timeout_sec` permanently false and the mission loop never
    # ends; a NaN goal reaches int(math.floor(...)) in preflight_goal and
    # raises there, after the run directory and the recorder already exist.
    return (not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value))


def _entity_name(value, source: str, where: str) -> str:
    """Check a gz entity name: it ends in an SDF string and a FaultEvent target."""
    if not isinstance(value, str) or not value or len(value) > 64 \
            or not set(value) <= _ID_ALLOWED or value[0] not in _ID_LEADING:
        raise ScenarioError(
            f'{source}: {where} is {value!r}; a gz entity name must be 1-64 '
            'characters of [a-z0-9_] starting with [a-z0-9].')
    return value


def _pose(doc: dict, key: str, source: str) -> dict:
    value = doc.get(key)
    if not isinstance(value, dict):
        raise ScenarioError(
            f'{source}: `{key}` is {value!r}, which is not a mapping. It must be '
            '{x: <m>, y: <m>, yaw: <rad>} in the gz WORLD frame.')
    out = {}
    for axis in ('x', 'y', 'yaw'):
        if axis not in value:
            raise ScenarioError(
                f'{source}: `{key}` has no `{axis}`. A pose needs x, y and yaw; '
                'a missing component cannot be assumed to be zero.')
        if not _is_finite_number(value[axis]):
            raise ScenarioError(
                f'{source}: `{key}.{axis}` is {value[axis]!r}, which is not a finite '
                'number.')
        out[axis] = float(value[axis])
    extra = sorted(set(value) - {'x', 'y', 'yaw'})
    if extra:
        raise ScenarioError(
            f'{source}: `{key}` has unexpected key(s) {extra}. A pose is x, y, yaw '
            'only; an ignored key is a value the author believes is in force.')
    return out


def _size(value, source: str, where: str) -> dict:
    if not isinstance(value, dict):
        raise ScenarioError(
            f'{source}: {where} is {value!r}, which is not a mapping. It must be '
            '{x: <m>, y: <m>, z: <m>}, the full extent of the box.')
    out = {}
    for axis in ('x', 'y', 'z'):
        if not _is_finite_number(value.get(axis)):
            raise ScenarioError(
                f'{source}: {where}.{axis} is {value.get(axis)!r}, which is not a '
                'finite number.')
        if float(value[axis]) <= 0.0:
            raise ScenarioError(
                f'{source}: {where}.{axis} is {value[axis]!r}; a box extent must be '
                'positive.')
        out[axis] = float(value[axis])
    extra = sorted(set(value) - {'x', 'y', 'z'})
    if extra:
        raise ScenarioError(
            f'{source}: {where} has unexpected key(s) {extra}. A box extent is '
            'x, y, z only; an ignored key is a value the author believes is in force.')
    return out


def _coerce_param(name: str, want: type, value, source: str, where: str,
                  limits: 'tuple[float | None, float | None]' = None):
    """One parameter, type-checked against INTERVENTION_PARAMS."""
    if want is bool:
        if not isinstance(value, bool):
            raise ScenarioError(
                f'{source}: {where}.{name} is {value!r}, which is not a boolean.')
        return value
    if want is float:
        if not _is_finite_number(value):
            raise ScenarioError(
                f'{source}: {where}.{name} is {value!r}, which is not a finite number.')
        number = float(value)
        if limits is not None:
            low, high = limits
            if (low is not None and number < low) or (high is not None and number > high):
                raise ScenarioError(
                    f'{source}: {where}.{name} is {number}; it must be in '
                    f'[{"-inf" if low is None else low}, '
                    f'{"inf" if high is None else high}]. A parameter outside the '
                    'range the modelled device supports is a fault nobody can apply.')
        return number
    if want is str:
        if name == 'name':
            return _entity_name(value, source, f'{where}.name')
        if not isinstance(value, str) or not value:
            raise ScenarioError(
                f'{source}: {where}.{name} is {value!r}, which is not a non-empty '
                'string.')
        return value
    if want is dict:
        if name == 'size':
            return _size(value, source, f'{where}.size')
        if not isinstance(value, dict):
            raise ScenarioError(
                f'{source}: {where}.{name} is {value!r}, which is not a mapping.')
        pose = {}
        for axis in ('x', 'y', 'yaw'):
            if not _is_finite_number(value.get(axis)):
                raise ScenarioError(
                    f'{source}: {where}.{name}.{axis} is {value.get(axis)!r}, which '
                    'is not a finite number. gz entity poses are WORLD frame x, y, '
                    'yaw.')
            pose[axis] = float(value[axis])
        extra = sorted(set(value) - {'x', 'y', 'yaw'})
        if extra:
            raise ScenarioError(
                f'{source}: {where}.{name} has unexpected key(s) {extra}.')
        return pose
    raise AssertionError(f'unhandled parameter type {want!r} for {name}')


def _validate_interventions(raw, timeout_sec: float, source: str) -> list:
    if raw is _ABSENT:
        return []
    if raw is None:
        raise ScenarioError(
            f'{source}: `interventions` is present but empty. Omit the key for a '
            'run without interventions; an empty key reads as a list the author '
            'meant to fill in.')
    if not isinstance(raw, list):
        raise ScenarioError(
            f'{source}: `interventions` is {raw!r}, which is not a list. Omit the '
            'key for a run without interventions.')

    parsed = []
    for index, entry in enumerate(raw):
        where = f'interventions[{index}]'
        if not isinstance(entry, dict):
            raise ScenarioError(f'{source}: {where} is {entry!r}, which is not a mapping.')
        if 'at_sec' not in entry:
            raise ScenarioError(
                f'{source}: {where} has no `at_sec`. An intervention without a time '
                'cannot be dispatched.')
        if not _is_finite_number(entry['at_sec']):
            raise ScenarioError(
                f'{source}: {where}.at_sec is {entry["at_sec"]!r}, which is not a '
                'finite number.')
        at_sec = float(entry['at_sec'])
        if at_sec < 0.0:
            raise ScenarioError(
                f'{source}: {where}.at_sec is {at_sec}; it is sim seconds SINCE GOAL '
                'ACCEPTANCE and cannot be negative.')
        if at_sec >= timeout_sec:
            raise ScenarioError(
                f'{source}: {where}.at_sec is {at_sec}, at or past `timeout_sec` '
                f'{timeout_sec}. The mission is cancelled by then, so the '
                'intervention would never fire and the run would silently be a '
                'baseline.')
        if 'type' not in entry:
            raise ScenarioError(f'{source}: {where} has no `type`.')
        kind = entry['type']
        if kind not in INTERVENTION_PARAMS:
            raise ScenarioError(
                f'{source}: {where}.type is {kind!r}; known types are '
                f'{sorted(INTERVENTION_PARAMS)}.')

        wanted = INTERVENTION_PARAMS[kind]
        params = entry.get('params', {})
        if not isinstance(params, dict):
            raise ScenarioError(
                f'{source}: {where}.params is {params!r}, which is not a mapping.')
        missing = sorted(set(wanted) - set(params))
        if missing:
            raise ScenarioError(
                f'{source}: {where} (type {kind}) is missing param(s) {missing}. '
                'Every parameter is required: a defaulted fault parameter is a fault '
                'nobody chose.')
        unexpected = sorted(set(params) - set(wanted))
        if unexpected:
            raise ScenarioError(
                f'{source}: {where} (type {kind}) has unexpected param(s) '
                f'{unexpected}; accepted are {sorted(wanted)}. An ignored parameter '
                'reads as applied.')
        extra_keys = sorted(set(entry) - {'at_sec', 'type', 'params'})
        if extra_keys:
            raise ScenarioError(
                f'{source}: {where} has unexpected key(s) {extra_keys}; an '
                'intervention is at_sec, type and params.')

        ranges = INTERVENTION_PARAM_RANGES.get(kind, {})
        checked = {name: _coerce_param(name, want, params[name], source, where,
                                       ranges.get(name))
                   for name, want in wanted.items()}
        parsed.append({'at_sec': at_sec, 'type': kind, 'params': checked})

    # Stable sort: two interventions at the same at_sec keep the author's order,
    # which is what makes `clear_obstacle` after `spawn_obstacle` well defined.
    parsed.sort(key=lambda item: item['at_sec'])

    spawned = {}
    for index, entry in enumerate(parsed, start=1):
        entry['id'] = f'{entry["type"]}_{index}'
        if entry['type'] == 'spawn_obstacle':
            name = entry['params']['name']
            if name in spawned:
                raise ScenarioError(
                    f'{source}: two spawn_obstacle interventions use the name '
                    f'{name!r}. gz entity names are unique, so the second spawn '
                    'would fail and a clear_obstacle could not say which it removed.')
            spawned[name] = entry['at_sec']
        elif entry['type'] == 'clear_obstacle':
            name = entry['params']['name']
            if name not in spawned:
                raise ScenarioError(
                    f'{source}: clear_obstacle for {name!r}, which no earlier '
                    'spawn_obstacle created. Removing an entity that does not exist '
                    'reports a cleared fault that never existed.')
            if entry['at_sec'] <= spawned[name]:
                raise ScenarioError(
                    f'{source}: clear_obstacle for {name!r} is at at_sec '
                    f'{entry["at_sec"]}, at or before the spawn_obstacle that '
                    f'created it ({spawned[name]}). The obstacle would exist for no '
                    'sim time at all and the run would record two interventions for '
                    'an environment that never changed.')
            spawned.pop(name)
    return parsed


def validate_scenario(doc: dict, source: str) -> dict:
    """
    Check one parsed scenario document and return it normalised.

    Raises ScenarioError naming `source`, the key and the offending value.
    Normalisation is: numbers coerced to float, `interventions` defaulted to
    [], sorted by at_sec and given stable `id`s of the form <type>_<1-based
    index after sorting> - the same id the fault manager will carry on its
    FaultEvents, so the two can be joined.
    """
    if not isinstance(doc, dict):
        raise ScenarioError(
            f'{source}: the document is {type(doc).__name__}, not a mapping of '
            'scenario keys.')

    missing = [key for key in REQUIRED_KEYS if key not in doc]
    if missing:
        raise ScenarioError(
            f'{source}: missing required key(s) {missing}. Required: '
            f'{list(REQUIRED_KEYS)}; `interventions` is optional.')

    unexpected = sorted(set(doc) - set(REQUIRED_KEYS) - {'interventions'})
    if unexpected:
        raise ScenarioError(
            f'{source}: unexpected top-level key(s) {unexpected}. A key this runner '
            'does not read is a setting the author believes is in force.')

    experiment_id = doc['experiment_id']
    if not isinstance(experiment_id, str) or not experiment_id \
            or not set(experiment_id) <= _ID_ALLOWED \
            or experiment_id[0] not in _ID_LEADING \
            or len(experiment_id) > 64:
        raise ScenarioError(
            f'{source}: `experiment_id` is {experiment_id!r}; it must be 1-64 '
            'characters of [a-z0-9_] starting with [a-z0-9], because it becomes a '
            'run directory name.')

    world = doc['world']
    if world not in ALLOWED_WORLDS:
        raise ScenarioError(
            f'{source}: `world` is {world!r}; the only authored world is '
            f'{ALLOWED_WORLDS[0]!r}. A second world needs its own '
            'bridge_roomba_<world>.yaml, its own slam_toolbox map and its own '
            '<world>_frames.yaml alignment. Falling back to '
            f'{ALLOWED_WORLDS[0]!r} would score this run against the wrong map, '
            'and every estimate-error metric would report a frame offset as '
            'localization error.')

    robot = doc['robot']
    if robot not in ALLOWED_ROBOTS:
        raise ScenarioError(
            f'{source}: `robot` is {robot!r}; the integrated robots are '
            f'{list(ALLOWED_ROBOTS)}. A robot is integrated when it has a profile in '
            'ral_bringup/config/robots/<robot>.yaml naming its gz model, its bridge '
            'config, its autonomy stack and the faults that are real for it.')

    seed = doc['seed']
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ScenarioError(
            f'{source}: `seed` is {seed!r}, which is not an integer.')

    timeout = doc['timeout_sec']
    if not _is_finite_number(timeout) or float(timeout) <= 0.0:
        raise ScenarioError(
            f'{source}: `timeout_sec` is {timeout!r}; it must be a positive, finite '
            'number of SIM seconds.')
    timeout = float(timeout)

    return {
        'experiment_id': experiment_id,
        'robot': robot,
        'world': world,
        'seed': int(seed),
        'initial_pose': _pose(doc, 'initial_pose', source),
        'goal': _pose(doc, 'goal', source),
        'timeout_sec': timeout,
        'interventions': _validate_interventions(doc.get('interventions', _ABSENT),
                                                 timeout, source),
    }


def load_scenario(path: str) -> dict:
    """Read and validate a scenario file. Raises ScenarioError naming the path."""
    try:
        with open(path, 'r') as handle:
            doc = yaml.safe_load(handle)
    except OSError as exc:
        raise ScenarioError(f'cannot read the scenario at {path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ScenarioError(f'{path} is not valid YAML: {exc}') from exc
    scenario = validate_scenario(doc, os.path.basename(path))
    scenario['source_path'] = os.path.abspath(path)
    return scenario


def mission_poses(scenario: dict, wtm: 'tuple[float, float, float]') -> dict:
    """
    Return the scenario's two poses in BOTH frames.

    Returns {'initial': {'world': (x, y, yaw), 'map': (x, y, yaw)}, 'goal': ...}.
    Built on ral_eval.frames.world_pose_to_map, never a second copy of the
    conversion: the launch file's display transform, the runner's /initialpose
    and the metrics' truth-vs-estimate comparison have to agree exactly, and a
    sign error in one of three copies surfaces as localization error.
    """
    out = {}
    for key in ('initial_pose', 'goal'):
        pose = scenario[key]
        world = (pose['x'], pose['y'], pose['yaw'])
        out['initial' if key == 'initial_pose' else 'goal'] = {
            'world': world,
            'map': world_pose_to_map(*world, wtm),
        }
    return out
