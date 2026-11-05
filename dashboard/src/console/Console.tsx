import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { Screen } from '../App';
import type { Ral } from '../client/useRal';
import type { FaultResponse, RobotProfile } from '../client/types';
import { C, F } from '../theme/tokens';
import { useTips } from '../ui/primitives';
import { DEFAULT_CAMERA, zoomBy } from './draw';
import type { Camera, Layers, View } from './draw';
import {
  INITIAL_FORMS, KIND_OF, SHEET_KEYS, SLOTS, buildParams, hasErrors, summarise,
  uniqueName, validate,
} from './faults';
import type { Forms, SheetKey } from './faults';
import type {
  ArmedRequest, ConsoleCtx, GoalForm, OpenSections, PickKind, RequestRecord, StartForm,
} from './context';
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
  const [pick, setPick] = useState<PickKind | null>(null);
  const [selectedObstacle, setSelectedObstacle] = useState<string | null>(null);
  const [timeMode, setTimeMode] = useState<'abs' | 'rel'>('abs');
  const [selectedMarker, setSelectedMarker] = useState<
    { type: 'm' | 'e'; index: number } | null>(null);
  const [startRobot, setStartRobot] = useState<string>('roomba');
  const [lifecycle, setLifecycle] = useState<RequestRecord | null>(null);
  const confirmTimer = useRef<number | null>(null);
  // The camera is a REF, not state: the world-view render loop redraws every
  // animation frame regardless, so a camera change needs no re-render, and a
  // useState camera would re-render the whole console on every wheel tick.
  const cameraRef = useRef<Camera>(DEFAULT_CAMERA);
  const resetCamera = useCallback(() => { cameraRef.current = DEFAULT_CAMERA; }, []);
  // Written by WorldView's render loop; the only copies in the app.
  const viewRef = useRef<View | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const zoomKeyboard = useCallback((factor: number) => {
    const view = viewRef.current;
    const canvas = canvasRef.current;
    if (!view || !canvas) return;
    cameraRef.current = zoomBy(cameraRef.current, view, canvas, factor);
  }, []);

  const stackRobot = uiState.stack?.robot ?? null;
  // A recording has no /stack message, but it does know which platform it
  // recorded: showing a car run under the roomba's profile would put the
  // wrong footprint, plant limits and estimator label on real data.
  const recordedRobot = source.kind === 'replay' ? (source.replay?.robot() ?? null) : null;
  const robot = stackRobot ?? recordedRobot ?? startRobot;
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

  // The spawn pose the NEXT stack will be launched at, defaulted from the
  // profile the operator is about to start. It follows `startRobot`, not
  // `robot`: a running stack's spawn pose is history and cannot change.
  const startDefault = useMemo<StartForm>(() => {
    const spawn = reference.profiles[startRobot]?.spawn;
    return {
      x: spawn ? String(spawn.x) : '',
      y: spawn ? String(spawn.y) : '',
      yaw: spawn ? String(spawn.yaw) : '0.0',
    };
  }, [reference, startRobot]);
  const [start, setStartState] = useState<StartForm>(startDefault);
  const lastStartRobot = useRef(startRobot);
  useEffect(() => {
    if (lastStartRobot.current !== startRobot) {
      lastStartRobot.current = startRobot;
      setStartState(startDefault);
    }
  }, [startRobot, startDefault]);

  // The live reposition destination. Starts EMPTY: a default here would draw
  // a destination marker the operator never chose.
  const [repos, setReposState] = useState<StartForm>({ x: '', y: '', yaw: '0.0' });
  // Off by default: a bag costs ~20 MB per run and the car runs at RTF
  // 0.34-0.52 under lockstep, so recording is an explicit choice.
  const [recordBag, setRecordBag] = useState(false);

  const errors = useMemo(() => validate(forms, reference), [forms, reference]);

  const stack = uiState.stack;
  const blockReason = (() => {
    if (!uiState.linkUp) return 'dashboard link disconnected; requests cannot be sent.';
    if (!stack || stack.state === 'idle') return 'no stack is running.';
    if (stack.state === 'starting') return 'stack bring-up in progress.';
    if (stack.state === 'stopping') return 'the stack is stopping.';
    if (stack.state === 'crashed') {
      return 'the stack process exited on its own; nothing is running to act on.';
    }
    if (stack.state === 'orphaned') {
      return 'a stack from a previous gateway is running; this gateway has no mission '
        + 'node or fault clients for it. Stop it first.';
    }
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
    const x = Number(start.x);
    const y = Number(start.y);
    const yaw = Number(start.yaw);
    // The spawn pose is sent only when the form BELONGS to the robot being
    // started: the robot popover sets `startRobot` and starts in the same
    // click, so the form is still the previous robot's at this point, and
    // spawning robot B at robot A's pose would be a silent substitution.
    // Omitting `spawn` makes the gateway use that profile's own default.
    const spawn = target === startRobot && [x, y, yaw].every(Number.isFinite)
      ? { x, y, yaw }
      : undefined;
    lifecycleCall(`start ${target}`, '/api/stack/start',
      spawn ? { robot: target, spawn } : { robot: target });
  }, [lifecycleCall, start, startRobot]);

  const stopStack = useCallback((force?: boolean) => {
    lifecycleCall(force ? 'force stop stack' : 'stop stack', '/api/stack/stop',
      force ? { force: true } : undefined);
  }, [lifecycleCall]);

  // "Stop, then start X" has to be exactly that. Issuing both POSTs at once
  // left the start refused by the running stack's own /clock, so the label
  // promised a relaunch that never happened.
  const stopThenStart = useCallback(async (robot: string) => {
    const record: RequestRecord = {
      key: 'lifecycle', kind: `stop, then start ${robot}`,
      service: '/api/stack/stop + /api/stack/start', summary: `relaunch as ${robot}`,
      sentSim: uiState.simNow, status: 'pending',
    };
    setLifecycle(record);
    try {
      const stopped = await source.post<{ accepted: boolean; message: string }>(
        '/api/stack/stop');
      if (!stopped.accepted) {
        setLifecycle({
          ...record, status: 'resolved', atSim: uiState.simNow,
          response: { fault_id: '', success: false,
            message: `the stack was not stopped, so ${robot} was not started: `
              + stopped.message },
        });
        return;
      }
      setStartRobot(robot);
      const started = await source.post<{ accepted: boolean; message: string }>(
        '/api/stack/start', { robot });
      setLifecycle({
        ...record, status: 'resolved', atSim: uiState.simNow,
        response: { fault_id: '', success: started.accepted,
          message: `stopped; ${started.message}` },
      });
    } catch (error) {
      setLifecycle({
        ...record, status: 'failed', error: String(error), atSim: uiState.simNow,
      });
    }
  }, [source, uiState]);

  const startMission = useCallback(() => {
    const x = Number(goal.x);
    const y = Number(goal.y);
    const yaw = Number(goal.yaw);
    const timeout = Number(goal.timeout);
    if (![x, y, yaw, timeout].every(Number.isFinite)) return;
    lifecycleCall('start mission', '/api/mission/start',
      { goal: { x, y, yaw }, timeout_sec: timeout, record: recordBag });
  }, [goal, lifecycleCall, recordBag]);

  const cancelMission = useCallback(() => {
    lifecycleCall('cancel mission', '/api/mission/cancel');
  }, [lifecycleCall]);

  // The live reposition. It is NOT a lifecycle call: it is an intervention
  // dispatched through /api/faults/<kind> like every catalogue kind, so it
  // lands in the REQUESTS list, /fault_events and run.json.interventions.
  const sendReposition = useCallback((x: number, y: number, yaw: number) => {
    sendFault('repos', 'reposition', { pose: { x, y, yaw } },
      `reposition to world (${x.toFixed(2)}, ${y.toFixed(2)}, ${yaw.toFixed(2)})`);
  }, [sendFault]);

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
      } else if (key === 's' || key === 'S') {
        setPick((value) => (value === 'start' ? null : 'start'));
      } else if (key === '+' || key === '=') {
        zoomKeyboard(1.25);
      } else if (key === '-' || key === '_') {
        zoomKeyboard(1 / 1.25);
      } else if (key === '0') {
        resetCamera();
      } else if (key === 'Escape') {
        setPick(null);
        setPop(null);
        setSelectedObstacle(null);
        setConfirm(null);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [fire, resetCamera, sheet, zoomKeyboard]);

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
    camera: cameraRef, resetCamera, viewRef, canvasRef, zoomKeyboard,
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
    start,
    setStart: (update) => setStartState((current) => update(current)),
    repos,
    setRepos: (update) => setReposState((current) => update(current)),
    sendReposition,
    recordBag, setRecordBag,
    timeMode, setTimeMode, selectedMarker, setSelectedMarker,
    startRobot, setStartRobot, startStack, stopStack, stopThenStart,
    startMission, cancelMission,
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
