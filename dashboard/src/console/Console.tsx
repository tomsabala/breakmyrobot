import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { Screen } from '../App';
import type { Ral } from '../client/useRal';
import type { FaultResponse, RobotProfile } from '../client/types';
import { C, F } from '../theme/tokens';
import { useTips } from '../ui/primitives';
import type { Layers } from './draw';
import {
  INITIAL_FORMS, KIND_OF, SHEET_KEYS, SLOTS, buildParams, hasErrors, summarise,
  uniqueName, validate,
} from './faults';
import type { Forms, SheetKey } from './faults';
import type { ArmedRequest, ConsoleCtx, GoalForm, OpenSections, RequestRecord } from './context';
import { TopBar } from './TopBar';
import { WorldView } from './WorldView';
import { SidePanel } from './SidePanel';
import { Drawer } from './Drawer';
import { Timeline } from './Timeline';

const DEFAULT_LAYERS: Layers = {
  map: true, gcost: false, lcost: false, scan: true, raw: false, plan: true,
  est: true, truth: true, foot: true, world: true,
};

const DEFAULT_OPEN: OpenSections = {
  mission: true, mdetail: false, setup: true, div: true, params: false, hon: false,
  timeline: true, rows: {},
};

export function Console({ ral, go }: { ral: Ral; go: (screen: Screen) => void }) {
  const { source, state } = ral;
  const reference = ral.reference!;
  const uiState = state!;

  const [pop, setPop] = useState<string | null>(null);
  const [layers, setLayers] = useState<Layers>(DEFAULT_LAYERS);
  const [drawerOpen, setDrawerOpen] = useState(true);
  const [tab, setTab] = useState<'inject' | 'sensing' | 'metrics'>('inject');
  const [sheet, setSheet] = useState<SheetKey>('obs');
  const [open, setOpen] = useState<OpenSections>(DEFAULT_OPEN);
  const [forms, setFormsState] = useState<Forms>(INITIAL_FORMS);
  const [armed, setArmed] = useState<ArmedRequest[]>([]);
  const [requests, setRequests] = useState<RequestRecord[]>([]);
  const [confirm, setConfirm] = useState<string | null>(null);
  const [pick, setPick] = useState<'obs' | 'kid' | 'goal' | null>(null);
  const [selectedObstacle, setSelectedObstacle] = useState<string | null>(null);
  const [timeMode, setTimeMode] = useState<'abs' | 'rel'>('abs');
  const [selectedMarker, setSelectedMarker] = useState<
    { type: 'm' | 'e'; index: number } | null>(null);
  const [startRobot, setStartRobot] = useState<string>('roomba');
  const [lifecycle, setLifecycle] = useState<RequestRecord | null>(null);
  const confirmTimer = useRef<number | null>(null);

  const stackRobot = uiState.stack?.robot ?? null;
  const robot = stackRobot ?? startRobot;
  const profile: RobotProfile | undefined = reference.profiles[robot];

  const goalDefault = useMemo<GoalForm>(() => {
    const baseline = reference.scenarios.find(
      (s) => s.robot === robot && s.experiment_id === `baseline_${robot}`);
    const goal = baseline?.goal;
    return {
      x: goal ? String(goal.x) : '',
      y: goal ? String(goal.y) : '',
      yaw: goal ? String(goal.yaw) : '0.0',
      timeout: String(baseline?.timeout_sec ?? 180),
    };
  }, [reference, robot]);
  const [goal, setGoalState] = useState<GoalForm>(goalDefault);
  const lastGoalRobot = useRef(robot);
  useEffect(() => {
    if (lastGoalRobot.current !== robot) {
      lastGoalRobot.current = robot;
      setGoalState(goalDefault);
    }
  }, [robot, goalDefault]);

  const errors = useMemo(() => validate(forms, reference), [forms, reference]);

  const stack = uiState.stack;
  const blockReason = (() => {
    if (!uiState.linkUp) return 'dashboard link disconnected; requests cannot be sent.';
    if (!stack || stack.state === 'idle') return 'no stack is running.';
    if (stack.state === 'starting') return 'stack bring-up in progress.';
    if (stack.state === 'stopping') return 'the stack is stopping.';
    if (!uiState.active) return 'fault manager not running (/faults/active absent).';
    return '';
  })();

  const cardFor = useCallback(
    (key: string) => requests.filter((r) => r.key === key).pop(), [requests]);
  const pending = useCallback(
    (key: string) => cardFor(key)?.status === 'pending', [cardFor]);

  const sendFault = useCallback((key: string, kind: string,
    params: Record<string, unknown>, summary: string) => {
    if (blockReason) return;
    const service = kind === 'clear_all'
      ? '/faults/clear_all'
      : (reference.catalogue.faults[kind]?.service ?? `/faults/${kind}`);
    const record: RequestRecord = {
      key, kind, service, summary, sentSim: uiState.simNow, status: 'pending',
    };
    setRequests((list) => [...list, record]);
    source.post<FaultResponse>(`/api/faults/${kind}`, params)
      .then((response) => {
        setRequests((list) => list.map((item) => (item === record
          ? { ...item, status: 'resolved', response, atSim: uiState.simNow }
          : item)));
      })
      .catch((error: unknown) => {
        setRequests((list) => list.map((item) => (item === record
          ? { ...item, status: 'failed', error: String(error), atSim: uiState.simNow }
          : item)));
      });
  }, [blockReason, reference, source, uiState]);

  const usedNames = useCallback(() => new Set<string>([
    ...Object.keys(uiState.obstacles),
    ...armed.filter((a) => a.sheet === 'obs')
      .map((a) => String((a.params as { name?: string }).name ?? '')),
  ]), [uiState.obstacles, armed]);

  const bumpName = useCallback(() => {
    setFormsState((current) => ({
      ...current,
      obs: { ...current.obs, name: uniqueName(current.obs.name, usedNames()) },
    }));
  }, [usedNames]);

  const arm = useCallback(() => {
    if (blockReason || armed.length >= 4 || hasErrors(errors, sheet)) return;
    const kind = KIND_OF[sheet];
    const params = buildParams(sheet, forms);
    setArmed((list) => [...list, {
      slot: SLOTS[list.length], sheet, kind, params, summary: summarise(sheet, forms),
    }]);
    if (sheet === 'obs') bumpName();
  }, [armed.length, blockReason, bumpName, errors, forms, sheet]);

  const fire = useCallback((index: number) => {
    const entry = armed[index];
    if (!entry || blockReason || pending(entry.sheet)) return;
    setArmed((list) => list.filter((_, i) => i !== index)
      .map((item, i) => ({ ...item, slot: SLOTS[i] })));
    sendFault(entry.sheet, entry.kind, entry.params, entry.summary);
  }, [armed, blockReason, pending, sendFault]);

  const disarm = useCallback((index: number) => {
    setArmed((list) => list.filter((_, i) => i !== index)
      .map((item, i) => ({ ...item, slot: SLOTS[i] })));
  }, []);

  const twoStep = useCallback((name: string, run: () => void) => {
    if (confirm !== name) {
      setConfirm(name);
      if (confirmTimer.current) window.clearTimeout(confirmTimer.current);
      confirmTimer.current = window.setTimeout(() => setConfirm(null), 4000);
      return;
    }
    setConfirm(null);
    run();
  }, [confirm]);

  // -- lifecycle actions ------------------------------------------------------
  // Each follows R-I1: idle -> pending -> resolved, and resolves ONLY from the
  // gateway's own answer. `accepted: true` is never rendered as "done": the
  // stack and mission states come from /api/stack and /mission_state.
  const lifecycleCall = useCallback((label: string, path: string, body?: unknown) => {
    const record: RequestRecord = {
      key: 'lifecycle', kind: label, service: path, summary: label,
      sentSim: uiState.simNow, status: 'pending',
    };
    setLifecycle(record);
    source.post<{ accepted: boolean; message: string }>(path, body)
      .then((response) => setLifecycle({
        ...record, status: 'resolved',
        response: { fault_id: '', success: response.accepted, message: response.message },
        atSim: uiState.simNow,
      }))
      .catch((error: unknown) => setLifecycle({
        ...record, status: 'failed', error: String(error), atSim: uiState.simNow,
      }));
  }, [source, uiState]);

  // The robot is an ARGUMENT, not read from state: the robot popover sets
  // `startRobot` and starts in one click, and a callback closing over the
  // previous render's state would launch the robot selected before it.
  const startStack = useCallback((robot?: string) => {
    const target = robot ?? startRobot;
    lifecycleCall(`start ${target}`, '/api/stack/start', { robot: target });
  }, [lifecycleCall, startRobot]);

  const stopStack = useCallback(() => {
    lifecycleCall('stop stack', '/api/stack/stop');
  }, [lifecycleCall]);

  const startMission = useCallback(() => {
    const x = Number(goal.x);
    const y = Number(goal.y);
    const yaw = Number(goal.yaw);
    const timeout = Number(goal.timeout);
    if (![x, y, yaw, timeout].every(Number.isFinite)) return;
    lifecycleCall('start mission', '/api/mission/start',
      { goal: { x, y, yaw }, timeout_sec: timeout });
  }, [goal, lifecycleCall]);

  const cancelMission = useCallback(() => {
    lifecycleCall('cancel mission', '/api/mission/cancel');
  }, [lifecycleCall]);

  // -- keyboard ---------------------------------------------------------------
  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      const tag = (event.target as HTMLElement | null)?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
      if (event.metaKey || event.ctrlKey || event.altKey) return;
      const key = event.key;
      if (['1', '2', '3', '4'].includes(key)) {
        setSheet(SHEET_KEYS[Number(key) - 1]);
        setTab('inject');
        setDrawerOpen(true);
        setConfirm(null);
      } else if (key.length === 1 && 'qwer'.includes(key.toLowerCase())) {
        fire('qwer'.indexOf(key.toLowerCase()));
      } else if (key === 'f' || key === 'F') {
        setDrawerOpen((value) => !value);
      } else if (key === 'p' || key === 'P') {
        setPick((value) => (value ? null : (sheet === 'obs' ? 'obs' : (sheet === 'kid' ? 'kid' : null))));
      } else if (key === 'g' || key === 'G') {
        setPick((value) => (value === 'goal' ? null : 'goal'));
      } else if (key === 'Escape') {
        setPick(null);
        setPop(null);
        setSelectedObstacle(null);
        setConfirm(null);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [fire, sheet]);

  const { onTip, hideTip, layer } = useTips();

  if (!profile) {
    return (
      <div style={{ padding: 24, font: `12.5px ${F.mono}`, color: C.bad }}>
        {`/api/reference carries no profile for ${robot}; the console will not guess one.`}
      </div>
    );
  }

  const ctx: ConsoleCtx = {
    source, reference, state: uiState, go,
    profile, robot, isCar: profile.mission_driver !== 'nav2',
    scenarios: reference.scenarios,
    pop, setPop,
    layers,
    toggleLayer: (key) => setLayers((value) => ({ ...value, [key]: !value[key] })),
    drawerOpen, setDrawerOpen, tab, setTab, sheet, setSheet,
    open,
    toggle: (key) => setOpen((value) => ({ ...value, [key]: !value[key] })),
    toggleRow: (id) => setOpen((value) => ({
      ...value, rows: { ...value.rows, [id]: !value.rows[id] },
    })),
    forms,
    setForms: (update) => setFormsState((current) => update(current)),
    errors,
    armed, arm, fire, disarm,
    requests, cardFor, pending, sendFault,
    confirm, twoStep,
    pick, setPick, selectedObstacle, setSelectedObstacle,
    goal,
    setGoal: (update) => setGoalState((current) => update(current)),
    timeMode, setTimeMode, selectedMarker, setSelectedMarker,
    startRobot, setStartRobot, startStack, stopStack, startMission, cancelMission,
    lifecycle,
    blockReason,
  };

  return (
    <div
      onMouseOver={onTip}
      onFocus={onTip}
      onBlur={hideTip}
      onMouseLeave={hideTip}
      style={{
        display: 'grid', gridTemplateColumns: 'minmax(0,1fr)',
        gridTemplateRows: 'auto minmax(0,1fr) auto', height: '100vh',
        minHeight: 760, minWidth: 1280, fontSize: '12.5px', lineHeight: 1.4,
      }}
    >
      <TopBar ctx={ctx} />
      <div style={{ display: 'flex', minHeight: 0 }}>
        <WorldView ctx={ctx} />
        <SidePanel ctx={ctx} />
        {drawerOpen ? <Drawer ctx={ctx} /> : null}
      </div>
      <Timeline ctx={ctx} />
      {layer}
    </div>
  );
}
