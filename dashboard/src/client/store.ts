import type {
  ActiveMsg, AlignMsg, BumperMsg, CmdMsg, EstimateMsg, FaultEventMsg, FeedbackMsg,
  GridMsg, MavrosStateMsg, MissionMsg, PlanMsg, Pose2D, RtfMsg, ScanMsg, StackMsg,
  TruthMsg, WpMsg,
} from './types';
import { TERMINAL_STATES } from '../theme/tokens';

/** Truth and estimate are paired by stamp within this window, or dropped. */
export const STAMP_MATCH_S = 0.05;
/** Trail decimation: a point is kept when it is this far from the last one. */
export const TRAIL_MIN_M = 0.02;

export interface TrailPoint { x: number; y: number }
export interface Pair { t: number; e: number; win: boolean }
export interface Obstacle {
  x: number; y: number; sx: number; sy: number; id: string; t: number;
}
export interface Kidnap {
  id: string; t: number; from: TrailPoint | null; to: TrailPoint;
}
export interface Gap { from: number; to: number }
export interface Achieved { t: number; v: number }

export interface UiState {
  /** Bumped on every ingest, so React can re-render off one number. */
  version: number;

  simNow: number;
  lastClockWallMs: number;
  linkUp: boolean;
  linkDownSim: number | null;
  linkDownWallMs: number | null;
  gaps: Gap[];
  droppedFrames: number;

  worldToMap: Pose2D;

  truth: TruthMsg | null;
  truthHist: { t: number; x: number; y: number; yaw: number }[];
  trailTruth: (TrailPoint | null)[];
  trailEstimate: (TrailPoint | null)[];
  breakNext: boolean;
  teleportJump: boolean;

  estimate: EstimateMsg | null;
  pairs: Pair[];
  unpaired: number;
  covMax: number | null;

  scan: ScanMsg | null;
  scanRaw: ScanMsg | null;
  plan: PlanMsg | null;
  planTimes: number[];
  cmd: CmdMsg | null;
  feedback: FeedbackMsg | null;
  wp: WpMsg | null;
  mavros: MavrosStateMsg | null;
  align: AlignMsg | null;
  bumper: BumperMsg | null;
  bumperHits: { t: number; pair: string }[];

  map: GridMsg | null;
  costmap: GridMsg | null;

  mission: MissionMsg | null;
  missions: MissionMsg[];
  events: FaultEventMsg[];
  active: ActiveMsg | null;
  obstacles: Record<string, Obstacle>;
  kidnaps: Kidnap[];

  stack: StackMsg | null;
  rtf: RtfMsg | null;

  tAccept: number | null;
  tTerminal: number | null;
  pathLength: number;
  teleportExcluded: number;
  lastPathPoint: { x: number; y: number } | null;
  goalError: number | null;
  achieved: Achieved[];
  gridSamples: { t: number; x: number; y: number; yaw: number }[];
  lastGridKey: number | null;
  seen: Set<string>;
}

export function createState(worldToMap: Pose2D): UiState {
  return {
    version: 0,
    simNow: 0,
    lastClockWallMs: Date.now(),
    linkUp: false,
    linkDownSim: null,
    linkDownWallMs: null,
    gaps: [],
    droppedFrames: 0,
    worldToMap,
    truth: null,
    truthHist: [],
    trailTruth: [],
    trailEstimate: [],
    breakNext: false,
    teleportJump: false,
    estimate: null,
    pairs: [],
    unpaired: 0,
    covMax: null,
    scan: null,
    scanRaw: null,
    plan: null,
    planTimes: [],
    cmd: null,
    feedback: null,
    wp: null,
    mavros: null,
    align: null,
    bumper: null,
    bumperHits: [],
    map: null,
    costmap: null,
    mission: null,
    missions: [],
    events: [],
    active: null,
    obstacles: {},
    kidnaps: [],
    stack: null,
    rtf: null,
    tAccept: null,
    tTerminal: null,
    pathLength: 0,
    teleportExcluded: 0,
    lastPathPoint: null,
    goalError: null,
    achieved: [],
    gridSamples: [],
    lastGridKey: null,
    seen: new Set<string>(),
  };
}

// -- frame conversions --------------------------------------------------------
// The same arithmetic as ral_eval/frames.py. The offsets come from
// /api/reference (maps/apartment_frames.yaml), never from a constant here: a
// hardcoded world_to_map would put every drawn pose somewhere else the day the
// map is remade.

export function worldToMap(x: number, y: number, yaw: number, w: Pose2D): Pose2D {
  const dx = x - w.x;
  const dy = y - w.y;
  const c = Math.cos(w.yaw);
  const s = Math.sin(w.yaw);
  return { x: dx * c + dy * s, y: -dx * s + dy * c, yaw: wrapToPi(yaw - w.yaw) };
}

export function mapToWorld(x: number, y: number, yaw: number, w: Pose2D): Pose2D {
  const c = Math.cos(w.yaw);
  const s = Math.sin(w.yaw);
  return { x: w.x + x * c - y * s, y: w.y + x * s + y * c, yaw: wrapToPi(yaw + w.yaw) };
}

export function wrapToPi(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

/**
 * The estimate in the `map` frame, or null when it cannot be expressed there.
 *
 * The car's estimate arrives in `ekf_local`, whose offset from `world` is
 * MEASURED per run and published on /frame_alignment. Until that message
 * arrives the estimate cannot be drawn at all: assuming an identity transform
 * would place the body at an offset that reads as localization error (C19).
 */
export function estimateInMap(state: UiState): Pose2D | null {
  const est = state.estimate;
  if (!est) return null;
  if (est.frame === 'map') return { x: est.pose.x, y: est.pose.y, yaw: est.pose.yaw_rad };
  const a = state.align;
  if (!a) return null;
  const c = Math.cos(a.yaw);
  const s = Math.sin(a.yaw);
  const wx = a.x + c * est.pose.x - s * est.pose.y;
  const wy = a.y + s * est.pose.x + c * est.pose.y;
  return worldToMap(wx, wy, est.pose.yaw_rad + a.yaw, state.worldToMap);
}

function truthAt(state: UiState, t: number) {
  const h = state.truthHist;
  if (!h.length) return null;
  let lo = 0;
  let hi = h.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (h[mid].t < t) lo = mid + 1;
    else hi = mid;
  }
  let best = h[lo];
  if (lo > 0 && Math.abs(h[lo - 1].t - t) < Math.abs(best.t - t)) best = h[lo - 1];
  return best;
}

// `spawn_obstacle` activated details carry the gz entity pose as prose,
// because that pose exists nowhere else: the manager's message IS the record
// of what gz created. Parsing it is the only way to draw the real box; a box
// drawn from the REQUEST would be the box the operator asked for, not the one
// in the world.
const BOX_RE =
  /box '([^']+)' \(([\d.]+) x ([\d.]+) x [\d.]+ m\) at world \((-?[\d.]+), (-?[\d.]+),/;
const KIDNAP_RE = /to world \((-?[\d.]+), (-?[\d.]+),/;

/**
 * Drop everything that belonged to a stack that has stopped.
 *
 * The alignment, the trails, the mission history, the fault lanes and the
 * grids all describe ONE robot in ONE run of the world. Carrying them into
 * the next stack would show the previous robot's mission under the new one.
 * The link state and the frame conversion survive, because neither belongs
 * to the stack.
 */
function resetForNewStack(state: UiState): void {
  const fresh = createState(state.worldToMap);
  fresh.version = state.version;
  fresh.linkUp = state.linkUp;
  fresh.stack = state.stack;
  Object.assign(state, fresh);
}

export function ingest(state: UiState, topic: string, payload: unknown): UiState {
  state.version += 1;
  switch (topic) {
    case 'reset':
      resetForNewStack(state);
      break;
    case 'link': {
      const p = payload as { up: boolean };
      if (p.up === false && state.linkUp) {
        state.linkUp = false;
        state.linkDownSim = state.simNow;
        state.linkDownWallMs = Date.now();
      } else if (p.up === true) {
        if (!state.linkUp && state.linkDownSim !== null) {
          // A hole in the record, drawn as a hole: the trails break and the
          // timeline hatches the interval. Nothing is interpolated across it.
          state.gaps.push({ from: state.linkDownSim, to: state.simNow });
          state.trailTruth.push(null);
          state.trailEstimate.push(null);
          state.lastPathPoint = null;
        }
        state.linkUp = true;
        state.linkDownWallMs = null;
      }
      break;
    }
    case 'dropped': {
      const p = payload as { count: number };
      state.droppedFrames += p.count;
      break;
    }
    case 'hello':
      state.linkUp = true;
      break;
    case 'clock': {
      const p = payload as { sim_s: number };
      state.simNow = p.sim_s;
      state.lastClockWallMs = Date.now();
      break;
    }
    case 'stack':
      state.stack = payload as StackMsg;
      break;
    case 'rtf':
      state.rtf = payload as RtfMsg;
      break;
    case 'truth':
      ingestTruth(state, payload as TruthMsg);
      break;
    case 'est':
      ingestEstimate(state, payload as EstimateMsg);
      break;
    case 'scan':
      state.scan = payload as ScanMsg;
      break;
    case 'scan_raw':
      state.scanRaw = payload as ScanMsg;
      break;
    case 'plan': {
      const p = payload as PlanMsg;
      state.plan = p;
      state.planTimes.push(p.stamp_sim_s);
      break;
    }
    case 'cmd':
      state.cmd = payload as CmdMsg;
      break;
    case 'fb':
      state.feedback = payload as FeedbackMsg;
      break;
    case 'wp':
      state.wp = payload as WpMsg;
      break;
    case 'mavros_state':
      state.mavros = payload as MavrosStateMsg;
      break;
    case 'align':
      state.align = payload as AlignMsg;
      break;
    case 'active':
      state.active = payload as ActiveMsg;
      break;
    case 'map':
      state.map = payload as GridMsg;
      break;
    case 'costmap':
      state.costmap = payload as GridMsg;
      break;
    case 'bumper':
      ingestBumper(state, payload as BumperMsg);
      break;
    case 'mission':
      ingestMission(state, payload as MissionMsg);
      break;
    case 'fault_event':
      ingestFaultEvent(state, payload as FaultEventMsg);
      break;
    default:
      break;
  }
  return state;
}

function ingestTruth(state: UiState, msg: TruthMsg) {
  state.truth = msg;
  const w = msg.pose;
  state.truthHist.push({ t: msg.stamp_sim_s, x: w.x, y: w.y, yaw: w.yaw });
  if (state.truthHist.length > 12000) state.truthHist.splice(0, 2000);

  const m = worldToMap(w.x, w.y, w.yaw, state.worldToMap);
  if (state.breakNext) {
    // A teleport is not motion. The trail breaks rather than drawing a line
    // the body never travelled.
    state.trailTruth.push(null);
    state.breakNext = false;
    state.teleportJump = true;
  }
  const last = state.trailTruth[state.trailTruth.length - 1];
  if (!last || Math.hypot(last.x - m.x, last.y - m.y) >= TRAIL_MIN_M) {
    state.trailTruth.push({ x: m.x, y: m.y });
  }

  if (state.tAccept !== null && state.tTerminal === null) {
    if (state.lastPathPoint) {
      const d = Math.hypot(state.lastPathPoint.x - w.x, state.lastPathPoint.y - w.y);
      if (state.teleportJump) state.teleportExcluded += d;
      else state.pathLength += d;
    }
    state.lastPathPoint = { x: w.x, y: w.y };
    state.teleportJump = false;
  }

  // A 10 Hz grid of truth samples, for the achieved-speed estimate the
  // Sensing tab shows. Central difference over 0.2 s; labelled derived.
  const key = Math.round(msg.stamp_sim_s * 10);
  if (key !== state.lastGridKey && Math.abs(msg.stamp_sim_s * 10 - key) < 0.11) {
    state.lastGridKey = key;
    state.gridSamples.push({ t: key / 10, x: w.x, y: w.y, yaw: w.yaw });
    const n = state.gridSamples.length;
    if (n >= 3) {
      const a = state.gridSamples[n - 3];
      const b = state.gridSamples[n - 1];
      const c = state.gridSamples[n - 2];
      const vx = (b.x - a.x) / 0.2;
      const vy = (b.y - a.y) / 0.2;
      state.achieved.push({ t: c.t, v: vx * Math.cos(c.yaw) + vy * Math.sin(c.yaw) });
      if (state.achieved.length > 3000) state.achieved.splice(0, 500);
    }
    if (state.gridSamples.length > 50) state.gridSamples.splice(0, 40);
  }
}

function ingestEstimate(state: UiState, msg: EstimateMsg) {
  state.estimate = msg;
  const m = estimateInMap(state);
  if (m) {
    const last = state.trailEstimate[state.trailEstimate.length - 1];
    if (!last || Math.hypot(last.x - m.x, last.y - m.y) >= TRAIL_MIN_M) {
      state.trailEstimate.push({ x: m.x, y: m.y });
    }
  }
  if (msg.covariance_trace !== null && state.tAccept !== null && state.tTerminal === null) {
    state.covMax = Math.max(state.covMax ?? 0, msg.covariance_trace);
  }
  // Paired by STAMP, within 50 ms. An unpaired sample is DROPPED and counted;
  // extrapolating truth onto an estimate stamp would manufacture a residual.
  const truth = truthAt(state, msg.stamp_sim_s);
  if (m && truth && Math.abs(truth.t - msg.stamp_sim_s) <= STAMP_MATCH_S) {
    const truthMap = worldToMap(truth.x, truth.y, truth.yaw, state.worldToMap);
    state.pairs.push({
      t: msg.stamp_sim_s,
      e: Math.hypot(m.x - truthMap.x, m.y - truthMap.y),
      win: state.tAccept !== null && state.tTerminal === null,
    });
    if (state.pairs.length > 20000) state.pairs.splice(0, 5000);
  } else {
    state.unpaired += 1;
  }
}

function ingestBumper(state: UiState, msg: BumperMsg) {
  state.bumper = msg;
  // Debounced per colliding pair: a gz contact sensor publishes continuously
  // while two bodies touch, and counting every message would report hundreds
  // of "collisions" for one bump.
  for (const pair of msg.pairs) {
    const previous = state.bumperHits.filter((h) => h.pair === pair).pop();
    if (!previous || msg.stamp_sim_s - previous.t > 0.5) {
      state.bumperHits.push({ t: msg.stamp_sim_s, pair });
    }
  }
}

function ingestMission(state: UiState, msg: MissionMsg) {
  const key = `m${msg.stamp_sim_s}${msg.state}`;
  if (state.seen.has(key)) return;
  state.seen.add(key);
  state.missions.push(msg);
  state.mission = msg;
  if (msg.state === 'idle') {
    // A new mission is being set up. The previous one's Nav2 feedback, the
    // previous wp_dist and the previous plan belong to it, and showing them
    // under this mission would attribute a distance-to-go to a goal that has
    // not been sent - which is exactly what an operator would read as
    // progress.
    state.feedback = null;
    state.wp = null;
    state.plan = null;
    state.tAccept = null;
    state.tTerminal = null;
    state.goalError = null;
  }
  if (msg.state === 'goal_accepted') {
    state.tAccept = msg.stamp_sim_s;
    state.tTerminal = null;
    state.pathLength = 0;
    state.teleportExcluded = 0;
    state.lastPathPoint = null;
    state.covMax = null;
    state.goalError = null;
  }
  if (TERMINAL_STATES.has(msg.state)) {
    state.tTerminal = msg.stamp_sim_s;
    if (state.truth && msg.state !== 'unreachable') {
      const goalWorld = mapToWorld(msg.goal.x, msg.goal.y, msg.goal.yaw_rad,
        state.worldToMap);
      state.goalError = Math.hypot(state.truth.pose.x - goalWorld.x,
        state.truth.pose.y - goalWorld.y);
    }
  }
}

function ingestFaultEvent(state: UiState, msg: FaultEventMsg) {
  const key = `e${msg.id}${msg.state}${msg.stamp_sim_s}`;
  if (state.seen.has(key)) return;
  state.seen.add(key);
  state.events.push(msg);

  if (msg.kind === 'spawn_obstacle' && msg.state === 'activated') {
    const hit = BOX_RE.exec(msg.detail);
    if (hit) {
      state.obstacles[hit[1]] = {
        x: Number(hit[4]), y: Number(hit[5]),
        sx: Number(hit[2]), sy: Number(hit[3]),
        id: msg.id, t: msg.stamp_sim_s,
      };
    }
  }
  if (msg.kind === 'spawn_obstacle' && msg.state === 'cleared') {
    for (const name of Object.keys(state.obstacles)) {
      if (state.obstacles[name].id === msg.id) delete state.obstacles[name];
    }
  }
  if (msg.kind === 'kidnap' && msg.state === 'activated') {
    const hit = KIDNAP_RE.exec(msg.detail);
    const from = state.truth
      ? worldToMap(state.truth.pose.x, state.truth.pose.y, 0, state.worldToMap)
      : null;
    if (hit) {
      const to = worldToMap(Number(hit[1]), Number(hit[2]), 0, state.worldToMap);
      state.kidnaps.push({
        id: msg.id, t: msg.stamp_sim_s,
        from: from ? { x: from.x, y: from.y } : null,
        to: { x: to.x, y: to.y },
      });
    }
    state.breakNext = true;
  }
}

/** Decode an OccupancyGrid payload into the signed int8 cells it carries. */
export function decodeGrid(grid: GridMsg): Int8Array {
  const binary = atob(grid.data_b64);
  const cells = new Int8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) cells[i] = (binary.charCodeAt(i) << 24) >> 24;
  return cells;
}
