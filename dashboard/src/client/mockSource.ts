import referenceFixture from '../fixtures/reference.json';
import runsFixture from '../fixtures/runs.json';
import runDetailsFixture from '../fixtures/runDetails.json';
import type { RalSource, ReplayControls } from './source';
import type {
  FaultResponse, Reference, RobotProfile, RunDetail, RunEntry,
} from './types';

/**
 * The design fixture, for UI work with no container, no gz and no stack.
 *
 * It publishes the SAME view model the gateway does, so the console cannot
 * grow a code path that only works here. What it simulates is a plant, a
 * fault manager and a mission, scripted: the robot follows a fixed path, the
 * lidar is ray-cast against the world SDF's own boxes, and /faults/* answers
 * with the manager's real wordings. Everything it CANNOT fake - provenance,
 * metrics, bags, plots - comes from src/fixtures, which are recordings of a
 * real gateway (see src/fixtures/README.md).
 *
 * It is labelled MOCK in the top bar and is the only mode with replay
 * controls, because only a scripted fixture can honour a rewind.
 */

const reference = referenceFixture as unknown as Reference;
const runs = runsFixture as unknown as RunEntry[];
const runDetails = runDetailsFixture as unknown as Record<string, RunDetail>;

const SIM_STEP = 0.02;
const SCAN_BEAMS = 360;
const ANGLE_MIN = -Math.PI;
const ANGLE_INCREMENT = (2 * Math.PI) / SCAN_BEAMS;
const RANGE_MIN = 0.15;
const RANGE_MAX = 12.0;

type Publish = (topic: string, payload: unknown) => void;

interface Box { x: number; y: number; sx: number; sy: number }

function rng(seed: number) {
  let state = seed | 0;
  return () => {
    state = (state + 0x6D2B79F5) | 0;
    let t = Math.imul(state ^ (state >>> 15), 1 | state);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function gauss(random: () => number) {
  let u = 0;
  while (!u) u = random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * random());
}

function rayBox(ox: number, oy: number, dx: number, dy: number, b: Box) {
  const x0 = b.x - b.sx / 2;
  const x1 = b.x + b.sx / 2;
  const y0 = b.y - b.sy / 2;
  const y1 = b.y + b.sy / 2;
  let lo = -Infinity;
  let hi = Infinity;
  if (Math.abs(dx) < 1e-12) {
    if (ox < x0 || ox > x1) return Infinity;
  } else {
    let a = (x0 - ox) / dx;
    let c = (x1 - ox) / dx;
    if (a > c) [a, c] = [c, a];
    lo = Math.max(lo, a);
    hi = Math.min(hi, c);
  }
  if (Math.abs(dy) < 1e-12) {
    if (oy < y0 || oy > y1) return Infinity;
  } else {
    let a = (y0 - oy) / dy;
    let c = (y1 - oy) / dy;
    if (a > c) [a, c] = [c, a];
    lo = Math.max(lo, a);
    hi = Math.min(hi, c);
  }
  if (hi < lo || hi < 0 || lo < 0) return Infinity;
  return lo;
}

function wrap(angle: number) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function f3(value: number) { return value.toFixed(3); }

/** The scripted path per robot: spawn, then the committed baseline goal. */
function pathFor(profile: RobotProfile): [number, number][] {
  const baseline = reference.scenarios.find(
    (s) => s.robot === profile.robot && s.experiment_id === `baseline_${profile.robot}`);
  const start: [number, number] = [profile.spawn.x, profile.spawn.y];
  const goal = baseline?.goal;
  if (!goal) return [start, start];
  if (profile.mission_driver === 'nav2') {
    // A plausible route through the two doorways, so the lidar and the plan
    // have something to do. The REAL route is whatever Nav2 plans.
    const doors = reference.world_geometry.doorways;
    return [start, ...doors.map((d) => [d.x, d.y] as [number, number]), [goal.x, goal.y]];
  }
  return [start, [goal.x, goal.y]];
}

interface ActiveFault {
  kind: string;
  event: {
    stamp_sim_s: number; id: string; kind: string; injection_point: string;
    target: string; state: string; detail: string;
  };
}

class MockStack {
  t = 0;
  s = 0;
  yaw: number;
  truth: { x: number; y: number; yaw: number };
  closed = false;
  robot: string;
  profile: RobotProfile;
  path: { points: [number, number][]; cum: number[]; seg: number[]; total: number };
  goal: { x: number; y: number; yaw: number };
  mission: { idle?: boolean; accepted?: boolean; navigating?: boolean; terminal?: boolean;
    tAccept?: number; tTerminal?: number } = {};
  missions: unknown[] = [];
  events: ActiveFault['event'][] = [];
  active: ActiveFault[] = [];
  obstacles: Record<string, Box> = {};
  scanDegradation: {
    noise: number; maxr: number; bs: number; be: number; drop: number;
  } | null = null;
  degradedScans = 0;
  mobility: number | null = null;
  reprojections = 0;
  counters: Record<string, number> = {};
  queue: { at: number; run: () => void }[] = [];
  noise = rng(7);
  faultNoise = rng(1);
  kidnapped = false;
  nextScan = 0.1;
  nextEstimate = 0;
  nextPlan = 0;
  nextCmd = 0;
  nextFeedback = 0;
  nextWp = 0;
  nextActive = 1;
  command = 0;
  scenario: string;

  constructor(robot: string, scenario: string, private out: Publish) {
    this.robot = robot;
    this.scenario = scenario;
    this.profile = reference.profiles[robot];
    const points = pathFor(this.profile);
    const cum = [0];
    const seg: number[] = [];
    for (let i = 1; i < points.length; i += 1) {
      const length = Math.hypot(points[i][0] - points[i - 1][0],
        points[i][1] - points[i - 1][1]);
      seg.push(length);
      cum.push(cum[i - 1] + length);
    }
    this.path = { points, cum, seg, total: cum[cum.length - 1] };
    const baseline = reference.scenarios.find(
      (s) => s.robot === robot && s.experiment_id === `baseline_${robot}`);
    const occupied = reference.scenarios.find(
      (s) => s.experiment_id === 'control_occupied_goal');
    const chosen = scenario === 'occupied' ? occupied?.goal : baseline?.goal;
    this.goal = chosen ?? { x: this.profile.spawn.x, y: this.profile.spawn.y, yaw: 0 };
    this.yaw = this.profile.spawn.yaw;
    this.truth = { x: this.profile.spawn.x, y: this.profile.spawn.y, yaw: this.yaw };
    this.publish('active', { stamp_sim_s: 0, active: [] });
    this.publish('stack', this.stackPayload());
  }

  publish(topic: string, payload: unknown) { this.out(topic, payload); }

  stackPayload() {
    return {
      state: 'ready', robot: this.robot, world: reference.world, seed: 1,
      spawn: { x: this.profile.spawn.x, y: this.profile.spawn.y, yaw: this.profile.spawn.yaw },
      started_wall_utc: null, wall_elapsed_s: null,
      wall_limit_s: this.profile.bringup_wall_limit_s,
      bringup: this.profile.bringup_topics.map((entry) => ({
        topic: entry.topic, seen: true, at_wall_s: null,
      })),
      detail: 'mock fixture: no stack was launched',
      log_path: null, prepared: true, prepare_record: {},
      mission_active: Boolean(this.mission.accepted && !this.mission.terminal),
      bumper_available: false,
      bumper_reason: 'the mock fixture publishes no /bumper',
    };
  }

  worldBoxes(): Box[] {
    const boxes = reference.world_geometry.boxes
      .filter((b) => b.name !== 'ground_plane')
      .map((b) => ({ x: b.x, y: b.y, sx: b.sx, sy: b.sy }));
    return boxes.concat(Object.values(this.obstacles));
  }

  poseAt(s: number) {
    const p = this.path;
    const clamped = Math.max(0, Math.min(s, p.total));
    let i = 0;
    while (i < p.seg.length - 1 && clamped > p.cum[i + 1]) i += 1;
    const a = p.points[i];
    const b = p.points[i + 1] ?? a;
    const length = p.seg[i] ?? 1;
    const fraction = length ? (clamped - p.cum[i]) / length : 0;
    return {
      x: a[0] + (b[0] - a[0]) * fraction,
      y: a[1] + (b[1] - a[1]) * fraction,
      yaw: Math.atan2(b[1] - a[1], b[0] - a[0]),
      index: i,
    };
  }

  estimateError(t: number) {
    const k = this.robot === 'car' ? 1.1 : 1;
    return {
      x: k * (0.052 * Math.sin(0.29 * t + 0.9) + 0.021 * Math.sin(1.37 * t)),
      y: k * (0.047 * Math.cos(0.21 * t + 0.3) + 0.017 * Math.sin(1.11 * t + 2)),
    };
  }

  blocked(s: number) {
    const p = this.poseAt(s);
    const r = this.profile.footprint.radius_m;
    return Object.values(this.obstacles).some((o) => {
      const dx = Math.max(Math.abs(p.x - o.x) - o.sx / 2, 0);
      const dy = Math.max(Math.abs(p.y - o.y) - o.sy / 2, 0);
      return Math.hypot(dx, dy) < r;
    });
  }

  step(dt: number) {
    if (this.closed) return;
    this.t += dt;
    const t = this.t;
    if (this.queue.length) {
      const due = this.queue.filter((q) => q.at <= t);
      this.queue = this.queue.filter((q) => q.at > t);
      for (const q of due) q.run();
    }

    if (!this.mission.idle && t >= 4) {
      this.mission.idle = true;
      this.publishMission('idle', '',
        'stack up, goal not yet sent; `goal` is the goal that will be sent; this '
        + 'mission is driven by the dashboard and is NOT recorded as a run');
    }
    if (this.scenario === 'occupied') {
      if (this.mission.idle && !this.mission.terminal && t >= 5) {
        this.terminal('unreachable', 'runner:not_sent:goal_cell_occupied',
          'the mock fixture reproduces the committed control run: the goal footprint '
          + 'samples occupied cells, so the goal was never sent');
      }
    } else {
      if (!this.mission.accepted && t >= 8) {
        this.mission.accepted = true;
        this.mission.tAccept = t;
        this.publishMission('goal_accepted', '', 'goal_id mock-fixture');
        this.clearEvery('cleared at mission start');
      }
      if (this.mission.accepted && !this.mission.navigating && t >= 8.5) {
        this.mission.navigating = true;
        this.publishMission('navigating', '',
          `the stack was observed executing the goal at sim ${t.toFixed(2)} s`);
      }
    }

    this.command = 0;
    if (this.mission.navigating && !this.mission.terminal && !this.kidnapped) {
      const target = this.poseAt(this.s);
      const dy = wrap(target.yaw - this.yaw);
      if (Math.abs(dy) > 0.02) {
        this.yaw += Math.sign(dy)
          * Math.min(Math.abs(dy), this.profile.plant.max_angular_rad_s * dt);
      }
      if (Math.abs(dy) < 0.3) {
        const speed = this.profile.plant.max_linear_m_s * 0.8;
        this.command = speed;
        const deficit = this.mobility === null ? 0 : Math.min(0.95, Math.abs(this.mobility) / 120);
        const next = Math.max(0, this.s + speed * (1 - deficit) * dt);
        if (!this.blocked(next) || next < this.s) this.s = next;
      }
      const now = this.poseAt(this.s);
      this.truth = { x: now.x, y: now.y, yaw: this.yaw };
    }

    this.publish('clock', { sim_s: t });
    this.publish('truth', {
      stamp_sim_s: t, frame: 'world',
      pose: { ...this.truth },
      twist: { linear_x: this.command, angular_z: 0, known: true },
    });

    if (t >= this.nextEstimate) {
      this.nextEstimate = t + 0.1 - 1e-9;
      this.publishEstimate(t);
    }
    if (t >= this.nextScan) {
      this.nextScan = t + 0.1 - 1e-9;
      this.publishScan(t);
    }
    if (this.profile.mission_driver === 'nav2' && this.mission.navigating
      && !this.mission.terminal) {
      if (t >= this.nextPlan) {
        this.nextPlan = t + 1.0 - 1e-9;
        const here = this.poseAt(this.s);
        const w = reference.world_to_map;
        this.publish('plan', {
          stamp_sim_s: t, frame: 'map',
          poses: [[here.x, here.y] as [number, number]]
            .concat(this.path.points.slice(here.index + 1))
            .map(([x, y]) => ({ x: x - w.x, y: y - w.y })),
        });
      }
      if (t >= this.nextCmd) {
        this.nextCmd = t + 0.1 - 1e-9;
        this.publish('cmd', {
          stamp_sim_s: t, stamp_source: 'received',
          linear_x: this.command, angular_z: 0,
        });
      }
      if (t >= this.nextFeedback) {
        this.nextFeedback = t + 0.2 - 1e-9;
        this.publish('fb', {
          stamp_sim_s: t,
          distance_remaining: Math.max(0, this.path.total - this.s),
          number_of_recoveries: 0,
        });
      }
    }
    if (this.profile.mission_driver !== 'nav2' && this.mission.accepted
      && !this.mission.terminal && t >= this.nextWp) {
      this.nextWp = t + 0.2 - 1e-9;
      this.publish('wp', {
        stamp_sim_s: t, stamp_source: 'received',
        wp_dist: Math.floor(Math.hypot(this.truth.x - this.goal.x, this.truth.y - this.goal.y)),
      });
    }
    if (this.mobility !== null && Math.round(t * 50) % 5 === 0) this.reprojections += 1;
    if (t >= this.nextActive && this.mobility !== null) {
      this.nextActive = t + 1 - 1e-9;
      this.publishActive();
    }

    if (this.mission.navigating && !this.mission.terminal
      && this.s >= this.path.total - 1e-6) {
      const outcome = this.profile.mission_driver === 'nav2'
        ? 'nav2:STATUS_SUCCEEDED' : 'mavros:reached_wp_radius';
      this.terminal('succeeded', outcome,
        'mock fixture: the scripted path reached its end');
    }
    if (this.mission.tTerminal !== undefined && t >= this.mission.tTerminal + 8) {
      this.closed = true;
    }
  }

  publishEstimate(t: number) {
    const error = this.estimateError(t);
    const w = reference.world_to_map;
    const world = { x: this.truth.x + error.x, y: this.truth.y + error.y };
    if (this.profile.estimate.frame === 'map') {
      this.publish('est', {
        stamp_sim_s: t, frame: 'map',
        pose: { x: world.x - w.x, y: world.y - w.y, yaw_rad: this.truth.yaw + 0.3 * error.x },
        covariance_trace: 0.068 + 0.013 * Math.sin(0.4 * t),
        label: this.profile.estimate.label,
      });
      return;
    }
    const align = { x: 1.0, y: 4.0, yaw: 0.045 };
    if (t >= 1 && !this.alignPublished) {
      this.alignPublished = true;
      this.publish('align', {
        stamp_sim_s: t, parent_frame: 'world', child_frame: 'ekf_local',
        x: align.x, y: align.y, yaw: align.yaw, residual_m: 0.0015,
        residual_yaw_rad: 0.0004,
        source: 'mock fixture: /ground_truth/odom and the estimate, both stationary',
      });
    }
    const dx = world.x - align.x;
    const dy = world.y - align.y;
    const c = Math.cos(align.yaw);
    const s = Math.sin(align.yaw);
    this.publish('est', {
      stamp_sim_s: t, frame: 'ekf_local',
      pose: { x: c * dx + s * dy, y: -s * dx + c * dy, yaw_rad: this.truth.yaw - align.yaw },
      covariance_trace: null,
      label: this.profile.estimate.label,
    });
  }

  alignPublished = false;

  publishScan(t: number) {
    const boxes = this.worldBoxes();
    const raw: (number | null)[] = new Array(SCAN_BEAMS);
    const degraded: (number | null)[] = new Array(SCAN_BEAMS);
    const D = this.scanDegradation;
    for (let i = 0; i < SCAN_BEAMS; i += 1) {
      const beam = ANGLE_MIN + i * ANGLE_INCREMENT;
      const angle = this.truth.yaw + beam;
      const dx = Math.cos(angle);
      const dy = Math.sin(angle);
      let r = Infinity;
      for (const box of boxes) {
        const hit = rayBox(this.truth.x, this.truth.y, dx, dy, box);
        if (hit < r) r = hit;
      }
      const value = (r < RANGE_MIN || r > RANGE_MAX)
        ? null : r + gauss(this.noise) * 0.015;
      raw[i] = value;
      if (!D || value === null) { degraded[i] = value; continue; }
      const degrees = (beam * 180) / Math.PI;
      let blind = false;
      if (D.bs !== D.be) {
        blind = D.bs < D.be
          ? (degrees >= D.bs && degrees <= D.be)
          : (degrees >= D.bs || degrees <= D.be);
      }
      if (blind || this.faultNoise() < D.drop) { degraded[i] = null; continue; }
      const noisy = value + gauss(this.faultNoise) * D.noise;
      degraded[i] = noisy > D.maxr ? null : Math.max(0, noisy);
    }
    if (D) this.degradedScans += 1;
    const header = {
      frame: 'rplidar_link', angle_min: ANGLE_MIN, angle_increment: ANGLE_INCREMENT,
      range_min: RANGE_MIN, range_max: RANGE_MAX,
    };
    this.publish('scan_raw', { stamp_sim_s: t, ...header, ranges: raw });
    this.publish('scan', { stamp_sim_s: t, ...header, ranges: degraded });
  }

  publishMission(state: string, outcome: string, detail: string) {
    const w = reference.world_to_map;
    const message = {
      stamp_sim_s: this.t,
      experiment_id: `mock_${this.robot}`,
      state,
      goal: {
        frame_id: 'map', x: this.goal.x - w.x, y: this.goal.y - w.y,
        yaw_rad: this.goal.yaw,
      },
      outcome,
      detail,
    };
    this.missions.push(message);
    this.publish('mission', message);
    this.publish('stack', this.stackPayload());
  }

  terminal(state: string, outcome: string, detail: string) {
    this.mission.terminal = true;
    this.mission.tTerminal = this.t;
    this.publishMission(state, outcome, detail);
  }

  mintId(kind: string) {
    this.counters[kind] = (this.counters[kind] ?? 0) + 1;
    return `${kind}_ui_${this.counters[kind]}`;
  }

  event(id: string, kind: string, target: string, state: string, detail: string) {
    const entry = reference.catalogue.faults[kind];
    const message = {
      stamp_sim_s: this.t, id, kind,
      injection_point: (entry?.injection_point ?? '').replace('{world}', reference.world),
      target, state, detail,
    };
    this.events.push(message);
    this.publish('fault_event', message);
    return message;
  }

  publishActive() {
    this.publish('active', {
      stamp_sim_s: this.t,
      active: this.active.map((a) => (a.kind === 'mobility_fault'
        ? { ...a.event, detail: `${a.event.detail} [${this.reprojections} re-projection(s) since]` }
        : a.event)),
    });
  }

  removeEffect(entry: ActiveFault): string {
    if (entry.kind === 'spawn_obstacle') {
      delete this.obstacles[entry.event.target];
      return `gz removed '${entry.event.target}' from the world; the costmaps clear it `
        + 'as the lidar sweeps the space again';
    }
    if (entry.kind === 'scan_degradation') {
      const D = this.scanDegradation!;
      this.scanDegradation = null;
      return `scan degradation removed after ${this.degradedScans} degraded scan(s); `
        + '/scan is a faithful republish of /scan_raw again (was: noise '
        + `${f3(D.noise)} m, max range ${f3(D.maxr)} m, blind sector `
        + `${D.bs.toFixed(1)} -> ${D.be.toFixed(1)} deg, dropout ${f3(D.drop)})`;
    }
    if (entry.kind === 'mobility_fault') {
      this.mobility = null;
      return `persistent wrench cleared on /world/${reference.world}/wrench/clear`;
    }
    return '';
  }

  clearEvery(reason: string): number {
    if (!this.active.length) return 0;
    const count = this.active.length;
    for (const entry of this.active) {
      this.removeEffect(entry);
      this.event(entry.event.id, entry.kind, entry.event.target, 'cleared', reason);
    }
    this.active = [];
    this.publishActive();
    return count;
  }

  call(kind: string, params: Record<string, unknown>): Promise<FaultResponse> {
    const { promise, resolve } = Promise.withResolvers<FaultResponse>();
    // One service round trip, in SIM time, so a `pending` card is visible for
    // as long as a real one would be at this replay speed.
    this.queue.push({ at: this.t + 0.12, run: () => resolve(this.apply(kind, params)) });
    return promise;
  }

  apply(kind: string, params: Record<string, unknown>): FaultResponse {
    const model = this.profile.model;
    if (kind === 'clear_all') {
      const count = this.clearEvery('cleared by /faults/clear_all');
      return {
        fault_id: '', success: true,
        message: `${count} fault(s) cleared, 0 could not be cleared`,
      };
    }
    if (!this.profile.faults.includes(kind)) {
      const id = this.mintId(kind);
      const reason = this.profile.unsupported?.[kind]
        ?? `${kind} is not listed under ${this.robot} in ${this.robot}.yaml`;
      this.event(id, kind, model, 'rejected', reason);
      return { fault_id: id, success: false, message: reason };
    }
    if (params.enabled === false) {
      const entry = this.active.find((a) => a.kind === kind);
      if (!entry) {
        const id = this.mintId(kind);
        const message = `no ${kind} is in force; disabling one that was never activated `
          + 'would report a cleared fault that never existed';
        this.event(id, kind, kind === 'scan_degradation' ? '/scan' : model, 'rejected',
          message);
        return { fault_id: id, success: false, message };
      }
      const message = this.removeEffect(entry);
      this.event(entry.event.id, kind, entry.event.target, 'cleared', message);
      this.active = this.active.filter((a) => a !== entry);
      this.publishActive();
      return { fault_id: entry.event.id, success: true, message };
    }

    const id = this.mintId(kind);
    const pose = params.pose as { x: number; y: number; yaw: number } | undefined;
    const size = params.size as { x: number; y: number; z: number } | undefined;
    const name = params.name as string | undefined;
    const target = (kind === 'spawn_obstacle' || kind === 'clear_obstacle')
      ? (name ?? '') : (kind === 'scan_degradation' ? '/scan' : model);

    if (kind === 'spawn_obstacle' && name && pose && size) {
      this.event(id, kind, target, 'requested',
        `name='${name}', world pose (${pose.x}, ${pose.y}), size (${size.x}, ${size.y}, `
        + `${size.z}) m`);
      if (this.obstacles[name]) {
        const message = `obstacle '${name}' is already spawned in this run; gz entity `
          + 'names are unique and a second spawn would fail';
        this.event(id, kind, target, 'rejected', message);
        return { fault_id: id, success: false, message };
      }
      this.obstacles[name] = { x: pose.x, y: pose.y, sx: size.x, sy: size.y };
      const message = `gz created static box '${name}' (${f3(size.x)} x ${f3(size.y)} x `
        + `${f3(size.z)} m) at world (${f3(pose.x)}, ${f3(pose.y)}, z=${f3(size.z / 2)} `
        + 'set to half the box height so it rests on the floor); the lidar sees it from '
        + 'the next sweep';
      const event = this.event(id, kind, target, 'activated', message);
      this.active.push({ kind, event });
      this.publishActive();
      return { fault_id: id, success: true, message };
    }

    if (kind === 'clear_obstacle' && name) {
      const entry = this.active.find(
        (a) => a.kind === 'spawn_obstacle' && a.event.target === name);
      this.event(id, kind, target, 'requested', `name='${name}'`);
      if (!entry) {
        const message = `'${name}' was never spawned by this manager; removing an entity `
          + 'it did not create would report a cleared fault that never existed';
        this.event(id, kind, target, 'rejected', message);
        return { fault_id: id, success: false, message };
      }
      const message = this.removeEffect(entry);
      this.event(id, kind, target, 'activated', message);
      this.event(entry.event.id, 'spawn_obstacle', entry.event.target, 'cleared', message);
      this.active = this.active.filter((a) => a !== entry);
      this.publishActive();
      return { fault_id: id, success: true, message };
    }

    if (kind === 'scan_degradation') {
      const noise = Number(params.range_noise_stddev_m);
      const maxr = Number(params.max_range_m);
      const bs = Number(params.blind_sector_start_deg);
      const be = Number(params.blind_sector_end_deg);
      const drop = Number(params.dropout_probability);
      this.event(id, kind, target, 'requested',
        `enabled=True, range_noise_stddev_m ${noise}, max_range_m ${maxr}, blind sector `
        + `${bs} -> ${be} deg, dropout_probability ${drop}`);
      this.supersede(kind, id);
      this.scanDegradation = { noise, maxr, bs, be, drop };
      this.degradedScans = 0;
      this.faultNoise = rng(1);
      const message = `scan degradation in force on /scan: additive Gaussian noise sigma `
        + `${f3(noise)} m, ranges truncated above ${f3(maxr)} m, blind sector `
        + `${bs.toFixed(1)} -> ${be.toFixed(1)} deg (CCW), per-beam dropout ${f3(drop)}; `
        + 'RNG reseeded with seed 1, so the corrupted stream repeats for this seed. '
        + '/scan_raw stays undegraded for comparison';
      const event = this.event(id, kind, target, 'activated', message);
      this.active.push({ kind, event });
      this.publishActive();
      return { fault_id: id, success: true, message };
    }

    if (kind === 'mobility_fault') {
      const fx = Number(params.force_x_n);
      const fy = Number(params.force_y_n);
      const tz = Number(params.torque_z_nm);
      this.event(id, kind, target, 'requested',
        `enabled=True, body-frame force (${fx}, ${fy}) N, torque ${tz} N.m`);
      this.supersede(kind, id);
      this.mobility = fx;
      this.reprojections = 0;
      const c = Math.cos(this.truth.yaw);
      const s = Math.sin(this.truth.yaw);
      const entry = reference.catalogue.faults[kind];
      const message = `body-frame force (${f3(fx)}, ${f3(fy)}) N and torque ${f3(tz)} N.m `
        + `published on /world/${reference.world}/wrench/persistent as world-frame `
        + `(${f3(fx * c - fy * s)}, ${f3(fx * s + fy * c)}) N at yaw ${f3(this.truth.yaw)} `
        + `rad, after clearing the previous persistent wrench. ${entry?.honesty_note ?? ''} `
        + `On ${this.robot}: ${this.profile.mobility_note ?? ''}`;
      const event = this.event(id, kind, target, 'activated', message);
      this.active.push({ kind, event });
      this.publishActive();
      return { fault_id: id, success: true, message };
    }

    if (kind === 'kidnap' && pose) {
      this.event(id, kind, target, 'requested',
        `new_pose world (${pose.x}, ${pose.y}, yaw ${pose.yaw})`);
      this.kidnapped = true;
      this.truth = { x: pose.x, y: pose.y, yaw: pose.yaw };
      const entry = reference.catalogue.faults[kind];
      const message = `gz moved ${model} to world (${f3(pose.x)}, ${f3(pose.y)}, `
        + `z=${f3(this.profile.kidnap_z_m)} substituted by the manager). The body moved `
        + 'with no actuation and no inertial signature, so '
        + `${this.profile.estimator_label} is now wrong by the teleport distance and must `
        + `recover alone. ${entry?.honesty_note ?? ''}`;
      this.event(id, kind, target, 'activated', message);
      return { fault_id: id, success: true, message };
    }

    const message = `the mock fixture cannot apply ${kind} with those parameters`;
    this.event(id, kind, target, 'rejected', message);
    return { fault_id: id, success: false, message };
  }

  supersede(kind: string, newId: string) {
    const previous = this.active.find((a) => a.kind === kind);
    if (!previous) return;
    this.removeEffect(previous);
    this.event(previous.event.id, kind, previous.event.target, 'cleared',
      `superseded by ${newId}`);
    this.active = this.active.filter((a) => a !== previous);
  }
}

export function createMockSource(): RalSource {
  const listeners = new Set<Publish>();
  const emit: Publish = (topic, payload) => {
    for (const fn of listeners) fn(topic, payload);
  };

  let robot = 'roomba';
  let scenario = 'baseline';
  let stack = new MockStack(robot, scenario, emit);
  let playing = true;
  let speed = 4;
  let accumulator = 0;
  let last = performance.now();
  let frame = 0;

  const reset = () => {
    stack = new MockStack(robot, scenario, emit);
    accumulator = 0;
    emit('hello', {
      world: reference.world, repo_root: 'mock', ros_domain_id: 'mock', source: 'mock',
    });
    emit('link', { up: true });
  };

  const loop = () => {
    frame = window.requestAnimationFrame(loop);
    const now = performance.now();
    const dt = Math.min(0.1, (now - last) / 1000);
    last = now;
    if (!playing || stack.closed) return;
    accumulator += dt * speed;
    let steps = 0;
    while (accumulator >= SIM_STEP && steps < 400 && !stack.closed) {
      stack.step(SIM_STEP);
      accumulator -= SIM_STEP;
      steps += 1;
    }
  };

  const replay: ReplayControls = {
    scenarios: () => [
      { value: 'baseline', label: `baseline_${robot}`, disabled: false },
      {
        value: 'occupied',
        label: robot === 'roomba'
          ? 'control_occupied_goal' : 'control_occupied_goal (roomba only)',
        disabled: robot !== 'roomba',
      },
    ],
    scenario: () => scenario,
    pick: (value) => { scenario = value; reset(); },
    restart: () => reset(),
    seekAccept: () => {
      reset();
      let guard = 0;
      while (stack.t < 7.5 && guard < 20000 && !stack.closed) {
        stack.step(SIM_STEP);
        guard += 1;
      }
    },
    playing: () => playing,
    togglePlay: () => { playing = !playing; },
    step: (seconds) => {
      let n = Math.round(seconds / SIM_STEP);
      while (n > 0) { stack.step(SIM_STEP); n -= 1; }
    },
    speed: () => speed,
    setSpeed: (value) => { speed = value; },
    closed: () => stack.closed,
    setRobot: (value) => { robot = value; scenario = 'baseline'; reset(); },
  };

  return {
    kind: 'mock',
    subscribe(fn) {
      listeners.add(fn);
      if (listeners.size === 1) {
        last = performance.now();
        frame = window.requestAnimationFrame(loop);
        reset();
      }
      return () => {
        listeners.delete(fn);
        if (listeners.size === 0) window.cancelAnimationFrame(frame);
      };
    },
    async post<T>(path: string, body?: unknown): Promise<T> {
      if (path.startsWith('/api/faults/')) {
        const kind = path.slice('/api/faults/'.length);
        const response = await stack.call(kind, (body ?? {}) as Record<string, unknown>);
        return response as unknown as T;
      }
      // The mock has no stack to start, stop or drive. It says so rather than
      // pretending the action succeeded.
      return {
        accepted: false,
        message: `${path} is unavailable in mock mode: there is no container, no gz and `
          + 'no autonomy stack here. Run scripts/dashboard.sh and reload without '
          + '?source=mock.',
      } as unknown as T;
    },
    reference: async () => reference,
    runs: async () => runs,
    run: async (id: string) => {
      const found = runDetails[id];
      if (found) return found;
      throw new Error(`${id} is not in the mock fixture; only `
        + `${Object.keys(runDetails).join(', ')} were captured.`);
    },
    mapImage: async () => null,
    replay,
  };
}
