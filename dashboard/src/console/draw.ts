import { C } from '../theme/tokens';
import { decodeGrid, estimateInMap, worldToMap } from '../client/store';
import type { UiState } from '../client/store';
import type { GridMsg, Pose2D, Reference, RobotProfile } from '../client/types';

export interface Layers {
  map: boolean; gcost: boolean; lcost: boolean; scan: boolean; raw: boolean;
  plan: boolean; est: boolean; truth: boolean; foot: boolean; world: boolean;
}

export interface View { x0: number; y1: number; s: number; ox: number; oy: number }

export interface DraftBox { name: string; x: number; y: number; sx: number; sy: number }
export interface DraftPose { x: number; y: number; yaw: number }

export interface DrawInput {
  state: UiState;
  reference: Reference;
  profile: RobotProfile;
  layers: Layers;
  selectedObstacle: string | null;
  draftBox: DraftBox | null;
  draftKidnap: DraftPose | null;
  draftGoal: DraftPose | null;
  armed: { slot: string; kind: string; params: Record<string, unknown> }[];
  linkUp: boolean;
  goalToleranceM: number;
}

/** Cache of decoded occupancy grids, keyed by the base64 payload itself. */
const gridCache = new Map<string, { canvas: HTMLCanvasElement; grid: GridMsg }>();

function gridCanvas(grid: GridMsg, costmap: boolean): HTMLCanvasElement {
  const key = `${costmap ? 'c' : 'm'}${grid.data_b64.length}:${grid.data_b64.slice(0, 64)}`
    + `:${grid.data_b64.slice(-64)}`;
  const hit = gridCache.get(key);
  if (hit) return hit.canvas;

  const cells = decodeGrid(grid);
  const canvas = document.createElement('canvas');
  canvas.width = grid.width;
  canvas.height = grid.height;
  const context = canvas.getContext('2d');
  if (!context) return canvas;
  const image = context.createImageData(grid.width, grid.height);
  for (let row = 0; row < grid.height; row += 1) {
    for (let col = 0; col < grid.width; col += 1) {
      const value = cells[row * grid.width + col];
      // The image is drawn top-down; the grid's row 0 is its BOTTOM row.
      const target = ((grid.height - 1 - row) * grid.width + col) * 4;
      let r = 0;
      let g = 0;
      let b = 0;
      let a = 255;
      if (costmap) {
        if (value < 0) { a = 0; } else if (value === 0) { a = 0; } else {
          // Nav2's inflation ramp, shown as it is: lethal (>= 99) solid, the
          // inflated skirt fading out. No threshold is invented.
          const t = Math.min(1, value / 100);
          r = Math.round(60 + 140 * t);
          g = Math.round(90 - 50 * t);
          b = Math.round(255 - 115 * t);
          a = Math.round(40 + 150 * t);
        }
      } else if (value < 0) {
        // UNKNOWN is never free: a distinct mid-grey, so an operator can see
        // how much of the map the robot has no information about.
        r = 70; g = 70; b = 78;
      } else if (value === 0) {
        r = 35; g = 35; b = 35;
      } else {
        const t = value / 100;
        r = Math.round(35 + 155 * t);
        g = Math.round(35 + 155 * t);
        b = Math.round(35 + 155 * t);
      }
      image.data[target] = r;
      image.data[target + 1] = g;
      image.data[target + 2] = b;
      image.data[target + 3] = a;
    }
  }
  context.putImageData(image, 0, 0);
  if (gridCache.size > 8) gridCache.clear();
  gridCache.set(key, { canvas, grid });
  return canvas;
}

/** The map-frame extent the view fits, from the grid the robot actually has. */
function extent(reference: Reference) {
  const origin = reference.map.origin;
  const resolution = reference.map.resolution;
  const x0 = origin[0];
  const y0 = origin[1];
  return {
    x0: x0 - 0.4,
    x1: x0 + reference.map.width * resolution + 0.4,
    y0: y0 - 0.4,
    y1: y0 + reference.map.height * resolution + 0.4,
  };
}

export function drawMap(canvas: HTMLCanvasElement, input: DrawInput): View | null {
  const {
    state, reference, profile, layers, selectedObstacle, draftBox, draftKidnap,
    draftGoal, armed, linkUp, goalToleranceM,
  } = input;
  const box = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const W = Math.max(10, box.width);
  const H = Math.max(10, box.height);
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
  }
  const g = canvas.getContext('2d');
  if (!g) return null;
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);

  const { x0, x1, y0, y1 } = extent(reference);
  const pad = 24;
  const s = Math.min((W - 2 * pad) / (x1 - x0), (H - 2 * pad) / (y1 - y0));
  const ox = (W - (x1 - x0) * s) / 2;
  const oy = (H - (y1 - y0) * s) / 2;
  const view: View = { x0, y1, s, ox, oy };
  const X = (v: number) => ox + (v - x0) * s;
  const Y = (v: number) => oy + (y1 - v) * s;

  const isCar = profile.mission_driver !== 'nav2';
  const wtm = reference.world_to_map;

  g.fillStyle = C.canvas;
  g.fillRect(0, 0, W, H);
  g.font = "10px 'IBM Plex Mono', monospace";

  // 1. The occupancy grid the robot localizes against, or - for a platform
  //    that has none - the committed grid as a BACKGROUND, labelled as such.
  if (layers.map) {
    const grid = state.map;
    if (grid) {
      const image = gridCanvas(grid, false);
      g.imageSmoothingEnabled = false;
      const w = grid.width * grid.resolution * s;
      const h = grid.height * grid.resolution * s;
      g.globalAlpha = isCar ? 0.45 : 1;
      g.drawImage(image, X(grid.origin.x), Y(grid.origin.y + grid.height * grid.resolution),
        w, h);
      g.globalAlpha = 1;
    }
  }

  // 2. The world SDF's static boxes, as WORLD GEOMETRY. Always distinguishable
  //    from the grid above: this is what gz has, not what the robot believes.
  if (layers.world) {
    g.strokeStyle = 'rgba(255,255,255,0.16)';
    g.lineWidth = 1;
    for (const b of reference.world_geometry.boxes) {
      if (b.name === 'ground_plane') continue;
      const m = worldToMap(b.x, b.y, 0, wtm);
      g.strokeRect(X(m.x - b.sx / 2), Y(m.y + b.sy / 2), b.sx * s, b.sy * s);
    }
    g.fillStyle = C.text5;
    g.textAlign = 'center';
    for (const door of reference.world_geometry.doorways) {
      const m = worldToMap(door.x, door.y, 0, wtm);
      g.fillText(`${door.label} ${door.width_m.toFixed(2)} m`, X(m.x), Y(m.y) - 6);
    }
    g.textAlign = 'left';
  }

  // 3. The global costmap, when the platform publishes one.
  if (layers.gcost && state.costmap) {
    const image = gridCanvas(state.costmap, true);
    g.imageSmoothingEnabled = false;
    const grid = state.costmap;
    g.drawImage(image, X(grid.origin.x), Y(grid.origin.y + grid.height * grid.resolution),
      grid.width * grid.resolution * s, grid.height * grid.resolution * s);
  }

  // axes, in map metres
  g.fillStyle = '#9a9a9a';
  g.textAlign = 'center';
  for (let i = Math.ceil(x0); i <= Math.floor(x1); i += 1) g.fillText(String(i), X(i), Y(y0) + 13);
  g.textAlign = 'right';
  for (let j = Math.ceil(y0); j <= Math.floor(y1); j += 1) g.fillText(String(j), X(x0) - 6, Y(j) + 3);
  g.textAlign = 'left';

  const rect = (
    o: { x: number; y: number; sx: number; sy: number },
    fill: string | null, stroke: string, dash: number[] = [],
  ) => {
    const m = worldToMap(o.x, o.y, 0, wtm);
    g.setLineDash(dash);
    if (fill) {
      g.fillStyle = fill;
      g.fillRect(X(m.x - o.sx / 2), Y(m.y + o.sy / 2), o.sx * s, o.sy * s);
    }
    g.strokeStyle = stroke;
    g.lineWidth = 1.3;
    g.strokeRect(X(m.x - o.sx / 2), Y(m.y + o.sy / 2), o.sx * s, o.sy * s);
    g.setLineDash([]);
  };

  // 4. Injected obstacles, from the manager's own `activated` detail.
  for (const name of Object.keys(state.obstacles)) {
    const o = state.obstacles[name];
    const selected = selectedObstacle === name;
    rect(o, 'rgba(255,0,0,0.18)', selected ? '#ffffff' : C.truth);
    const m = worldToMap(o.x, o.y, 0, wtm);
    g.fillStyle = selected ? '#ffffff' : C.obstacleText;
    g.fillText(name, X(m.x - o.sx / 2), Y(m.y + o.sy / 2) - 4);
  }

  // 5. Armed-but-unsent requests, dashed: nothing in the world yet.
  armed.forEach((entry, index) => {
    if (entry.kind === 'spawn_obstacle') {
      const p = entry.params as unknown as {
        name: string; pose: { x: number; y: number }; size: { x: number; y: number };
      };
      const shape = { x: p.pose.x, y: p.pose.y, sx: p.size.x, sy: p.size.y };
      rect(shape, null, C.command, [2, 3]);
      const m = worldToMap(shape.x, shape.y, 0, wtm);
      g.fillStyle = C.draftText;
      g.fillText(`${entry.slot} armed '${p.name}'`,
        X(m.x - shape.sx / 2), Y(m.y - shape.sy / 2) + 12);
    }
    if (entry.kind === 'kidnap') {
      const p = entry.params as unknown as { pose: { x: number; y: number } };
      const m = worldToMap(p.pose.x, p.pose.y, 0, wtm);
      g.strokeStyle = C.command;
      g.setLineDash([2, 3]);
      g.beginPath();
      g.arc(X(m.x), Y(m.y), profile.footprint.radius_m * s, 0, Math.PI * 2);
      g.stroke();
      g.setLineDash([]);
      g.fillStyle = C.draftText;
      g.fillText(`${entry.slot} armed kidnap`,
        X(m.x) + profile.footprint.radius_m * s + 4, Y(m.y) + 3);
    }
    void index;
  });

  // 6. Drafts: what WOULD be sent. Dashed orange, and labelled "not sent".
  if (draftBox && Number.isFinite(draftBox.x) && draftBox.sx > 0 && draftBox.sy > 0) {
    rect(draftBox, 'rgba(255,140,0,0.08)', C.command, [5, 4]);
    const m = worldToMap(draftBox.x, draftBox.y, 0, wtm);
    g.fillStyle = C.draftText;
    g.fillText(`draft '${draftBox.name}' · not sent`,
      X(m.x - draftBox.sx / 2), Y(m.y - draftBox.sy / 2) + 12);
  }
  if (draftKidnap && Number.isFinite(draftKidnap.x)) {
    const m = worldToMap(draftKidnap.x, draftKidnap.y, draftKidnap.yaw, wtm);
    const px = X(m.x);
    const py = Y(m.y);
    g.strokeStyle = C.command;
    g.lineWidth = 1.4;
    g.setLineDash([5, 4]);
    g.beginPath();
    g.arc(px, py, profile.footprint.radius_m * s, 0, Math.PI * 2);
    g.stroke();
    g.setLineDash([]);
    g.beginPath();
    g.moveTo(px, py);
    g.lineTo(px + Math.cos(m.yaw) * 0.45 * s, py - Math.sin(m.yaw) * 0.45 * s);
    g.stroke();
    g.fillStyle = C.draftText;
    g.fillText('kidnap draft · not sent', px + profile.footprint.radius_m * s + 4, py + 3);
  }

  const estimate = estimateInMap(state);

  // 7. The lidar. Drawn from the ESTIMATE's pose, because that is the pose the
  //    robot believes it took the sweep from; drawing it from truth would make
  //    a badly localized robot look well localized.
  const drawScan = (scan: typeof state.scan, raw: boolean) => {
    if (!scan || !estimate) return;
    for (let i = 0; i < scan.ranges.length; i += 1) {
      const r = scan.ranges[i];
      if (r === null) continue;
      const a = estimate.yaw + scan.angle_min + i * scan.angle_increment;
      const px = X(estimate.x + r * Math.cos(a));
      const py = Y(estimate.y + r * Math.sin(a));
      if (raw) {
        g.strokeStyle = 'rgba(255,255,0,0.45)';
        g.lineWidth = 1;
        g.strokeRect(px - 1.5, py - 1.5, 3, 3);
      } else {
        g.fillStyle = C.scan;
        g.fillRect(px - 1.2, py - 1.2, 2.4, 2.4);
      }
    }
  };
  if (layers.raw) drawScan(state.scanRaw, true);
  if (layers.scan) drawScan(state.scan, false);

  // 8. The plan, where one exists.
  if (layers.plan && state.plan && state.plan.poses.length) {
    g.strokeStyle = C.plan;
    g.lineWidth = 1.6;
    g.setLineDash([5, 4]);
    g.beginPath();
    state.plan.poses.forEach((p, i) => (i ? g.lineTo(X(p.x), Y(p.y)) : g.moveTo(X(p.x), Y(p.y))));
    g.stroke();
    g.setLineDash([]);
  }

  // 9. The goal. The one that was SENT when a mission is live, else the draft.
  const goalMission = state.mission?.goal;
  const goal = goalMission
    ? { x: goalMission.x, y: goalMission.y }
    : (draftGoal ? worldToMap(draftGoal.x, draftGoal.y, draftGoal.yaw, wtm) : null);
  if (goal) {
    const sent = Boolean(goalMission);
    g.strokeStyle = C.command;
    g.lineWidth = 1.5;
    g.setLineDash(sent ? [] : [4, 3]);
    g.beginPath();
    g.arc(X(goal.x), Y(goal.y), Math.max(3, goalToleranceM * s), 0, Math.PI * 2);
    g.stroke();
    g.beginPath();
    g.moveTo(X(goal.x) - 5, Y(goal.y));
    g.lineTo(X(goal.x) + 5, Y(goal.y));
    g.moveTo(X(goal.x), Y(goal.y) - 5);
    g.lineTo(X(goal.x), Y(goal.y) + 5);
    g.stroke();
    g.setLineDash([]);
    g.fillStyle = C.draftText;
    g.fillText(sent ? 'goal' : 'goal · not sent', X(goal.x) + 8, Y(goal.y) - 6);
  }

  // 10. The two trails. `null` is a BREAK, never a join.
  const trail = (points: ({ x: number; y: number } | null)[], colour: string,
    dash: number[]) => {
    g.strokeStyle = colour;
    g.lineWidth = 2;
    g.setLineDash(dash);
    g.beginPath();
    let pen = false;
    for (const p of points) {
      if (!p) { pen = false; continue; }
      if (!pen) { g.moveTo(X(p.x), Y(p.y)); pen = true; } else g.lineTo(X(p.x), Y(p.y));
    }
    g.stroke();
    g.setLineDash([]);
  };
  if (layers.truth) trail(state.trailTruth, C.truth, []);
  if (layers.est) trail(state.trailEstimate, C.estimate, [7, 4]);

  // 11. Kidnaps: where the body was, where it appeared, and that the trail
  //     between the two is a break rather than a path.
  for (const k of state.kidnaps) {
    g.fillStyle = C.fault;
    g.strokeStyle = C.fault;
    if (k.from) {
      g.beginPath();
      g.arc(X(k.from.x), Y(k.from.y), 5, 0, Math.PI * 2);
      g.stroke();
    }
    const tx = X(k.to.x);
    const ty = Y(k.to.y);
    g.beginPath();
    g.moveTo(tx, ty - 12);
    g.lineTo(tx - 5, ty - 4);
    g.lineTo(tx + 5, ty - 4);
    g.closePath();
    g.fill();
    g.fillStyle = C.faultText;
    g.fillText(`${k.id} ${k.t.toFixed(2)} s · trail break`, tx + 8, ty - 6);
  }

  const pose = (p: Pose2D, colour: string, dash: number[]) => {
    const px = X(p.x);
    const py = Y(p.y);
    if (layers.foot) {
      g.strokeStyle = colour;
      g.lineWidth = 1.2;
      g.setLineDash(dash);
      g.beginPath();
      g.arc(px, py, profile.footprint.radius_m * s, 0, Math.PI * 2);
      g.stroke();
      g.setLineDash([]);
    }
    g.strokeStyle = colour;
    g.lineWidth = 2.2;
    g.beginPath();
    g.moveTo(px, py);
    g.lineTo(px + Math.cos(p.yaw) * 0.4 * s, py - Math.sin(p.yaw) * 0.4 * s);
    g.stroke();
    g.fillStyle = colour;
    g.beginPath();
    g.arc(px, py, 2.6, 0, Math.PI * 2);
    g.fill();
  };
  if (layers.truth && state.truth) {
    const t = state.truth.pose;
    pose(worldToMap(t.x, t.y, t.yaw, wtm), C.truth, []);
  }
  if (layers.est && estimate) {
    pose(estimate, C.estimate, [4, 3]);
    const cov = state.estimate?.covariance_trace;
    if (cov !== null && cov !== undefined) {
      g.strokeStyle = 'rgba(0,170,255,0.6)';
      g.lineWidth = 1;
      g.setLineDash([2, 3]);
      g.beginPath();
      g.arc(X(estimate.x), Y(estimate.y), 2 * Math.sqrt(cov * 0.45) * s, 0, Math.PI * 2);
      g.stroke();
      g.setLineDash([]);
    }
  }
  if (state.estimate && state.estimate.frame !== 'map' && !state.align) {
    g.fillStyle = '#cfe9ff';
    g.fillText('estimate not drawn: /frame_alignment not yet received '
      + '(no implicit identity transform)', X(x0) + 6, Y(y1) - 8);
  }

  if (!linkUp) {
    g.fillStyle = 'rgba(20,20,20,0.55)';
    g.fillRect(0, 0, W, H);
    g.fillStyle = C.warn;
    g.font = "600 12px 'IBM Plex Mono', monospace";
    g.textAlign = 'center';
    g.fillText(`dashboard link disconnected · data received up to sim `
      + `${state.simNow.toFixed(2)} s, not current`, W / 2, H / 2);
    g.textAlign = 'left';
  }
  return view;
}

/** |estimate − truth| over the last 60 s of sim time. The axis is never clipped. */
export function drawDivergence(canvas: HTMLCanvasElement, state: UiState,
  startPoseResidual: number | null) {
  const box = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const W = Math.max(10, box.width);
  const H = Math.max(10, box.height);
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr);
    canvas.height = Math.round(H * dpr);
  }
  const g = canvas.getContext('2d');
  if (!g) return;
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);

  const now = state.simNow;
  const t1 = Math.max(now, 10);
  const t0 = Math.max(0, t1 - 60);
  const points = state.pairs.filter((p) => p.t >= t0);
  // reduce, not Math.max(...array): spreading a long window of pairs as
  // arguments overflows the call stack. The axis is never clipped, so the
  // top is always the largest residual in view.
  const ymax = points.reduce((best, p) => (p.e * 1.15 > best ? p.e * 1.15 : best), 0.15);
  const l = 32;
  const rt = 8;
  const tp = 8;
  const bt = 16;
  const X = (t: number) => l + ((t - t0) / (t1 - t0)) * (W - l - rt);
  const Y = (v: number) => tp + (1 - v / ymax) * (H - tp - bt);

  g.strokeStyle = C.lineSoft;
  g.lineWidth = 1;
  g.beginPath();
  g.moveTo(l, tp);
  g.lineTo(l, H - bt);
  g.lineTo(W - rt, H - bt);
  g.stroke();

  g.fillStyle = '#9a9a9a';
  g.font = "9.5px 'IBM Plex Mono', monospace";
  g.textAlign = 'right';
  g.fillText(`${ymax.toFixed(2)} m`, l - 3, tp + 8);
  g.fillText('0', l - 3, H - bt);
  g.textAlign = 'left';
  g.fillText(`${t0.toFixed(0)} s`, l, H - 4);
  g.textAlign = 'right';
  g.fillText(`${t1.toFixed(0)} s sim`, W - rt, H - 4);
  g.textAlign = 'left';

  if (startPoseResidual !== null) {
    g.strokeStyle = C.text4;
    g.setLineDash([4, 3]);
    g.beginPath();
    g.moveTo(l, Y(startPoseResidual));
    g.lineTo(W - rt, Y(startPoseResidual));
    g.stroke();
    g.setLineDash([]);
  }
  if (state.tAccept !== null) {
    const a = Math.max(t0, state.tAccept);
    g.fillStyle = 'rgba(255,255,255,0.05)';
    g.fillRect(X(a), tp, X(state.tTerminal ?? now) - X(a), H - tp - bt);
  }
  g.strokeStyle = C.text;
  g.lineWidth = 1.3;
  g.beginPath();
  let previous: { t: number; e: number } | null = null;
  for (const p of points) {
    // A gap longer than a second is a gap, not a line across it.
    if (!previous || p.t - previous.t > 1.0) g.moveTo(X(p.t), Y(p.e));
    else g.lineTo(X(p.t), Y(p.e));
    previous = p;
  }
  g.stroke();
  g.fillStyle = C.text;
  for (const p of points) g.fillRect(X(p.t) - 1, Y(p.e) - 1, 2, 2);
}

/** Screen pixel -> map-frame metres, for the pick interactions. */
export function toMap(view: View, canvas: HTMLCanvasElement, clientX: number,
  clientY: number): Pose2D {
  const box = canvas.getBoundingClientRect();
  const px = clientX - box.left;
  const py = clientY - box.top;
  return {
    x: view.x0 + (px - view.ox) / view.s,
    y: view.y1 - (py - view.oy) / view.s,
    yaw: 0,
  };
}
