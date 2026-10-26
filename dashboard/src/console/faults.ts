import type { CatalogueEntry, Reference, RobotProfile, ScenarioSummary } from '../client/types';

/**
 * The four fault sheets, and the scenario field names each one sends.
 *
 * The form keys ARE the service's field names (ral_eval/scenario.py's
 * INTERVENTION_PARAMS), so the console, the scenarios and the catalogue speak
 * one vocabulary and the gateway can hand the body straight to the runner's
 * own request builders. Renaming anything here silently diverges the UI from
 * every committed scenario.
 */
export type SheetKey = 'obs' | 'scan' | 'mob' | 'kid';

export const KIND_OF: Record<SheetKey, string> = {
  obs: 'spawn_obstacle',
  scan: 'scan_degradation',
  mob: 'mobility_fault',
  kid: 'kidnap',
};

export const SHEET_KEYS: SheetKey[] = ['obs', 'scan', 'mob', 'kid'];
export const SLOTS = ['Q', 'W', 'E', 'R'];

export interface Forms {
  obs: { name: string; x: string; y: string; yaw: string; sx: string; sy: string; sz: string };
  scan: { noise: string; maxr: string; bs: string; be: string; drop: string };
  mob: { fx: string; fy: string; tz: string };
  kid: { x: string; y: string; yaw: string };
}

export const INITIAL_FORMS: Forms = {
  obs: { name: 'blocker_1', x: '6.0', y: '3.0', yaw: '0.0', sx: '1.2', sy: '0.2', sz: '0.6' },
  scan: { noise: '0.08', maxr: '2.5', bs: '90', be: '150', drop: '0.2' },
  mob: { fx: '-60', fy: '0', tz: '0' },
  kid: { x: '2.0', y: '6.5', yaw: '0.0' },
};

export function num(value: string): number | null {
  const text = value.trim();
  if (text === '') return null;
  const parsed = Number(text);
  return Number.isFinite(parsed) ? parsed : null;
}

/** The request body, in the service's own field names. */
export function buildParams(sheet: SheetKey, forms: Forms): Record<string, unknown> {
  if (sheet === 'obs') {
    const f = forms.obs;
    return {
      name: f.name,
      pose: { x: num(f.x), y: num(f.y), yaw: num(f.yaw) },
      size: { x: num(f.sx), y: num(f.sy), z: num(f.sz) },
    };
  }
  if (sheet === 'scan') {
    const f = forms.scan;
    return {
      enabled: true,
      range_noise_stddev_m: num(f.noise),
      max_range_m: num(f.maxr),
      blind_sector_start_deg: num(f.bs),
      blind_sector_end_deg: num(f.be),
      dropout_probability: num(f.drop),
    };
  }
  if (sheet === 'mob') {
    const f = forms.mob;
    return {
      enabled: true,
      force_x_n: num(f.fx),
      force_y_n: num(f.fy),
      torque_z_nm: num(f.tz),
    };
  }
  const f = forms.kid;
  return { pose: { x: num(f.x), y: num(f.y), yaw: num(f.yaw) } };
}

export function summarise(sheet: SheetKey, forms: Forms): string {
  const show = (value: number | null) => (value === null ? '?' : String(value));
  if (sheet === 'obs') {
    const f = forms.obs;
    return `'${f.name}' at world (${show(num(f.x))}, ${show(num(f.y))}) · `
      + `${show(num(f.sx))} × ${show(num(f.sy))} × ${show(num(f.sz))} m`;
  }
  if (sheet === 'scan') {
    const f = forms.scan;
    return `σ ${show(num(f.noise))} m · max ${show(num(f.maxr))} m · `
      + `blind ${show(num(f.bs))} → ${show(num(f.be))}° · dropout ${show(num(f.drop))}`;
  }
  if (sheet === 'mob') {
    const f = forms.mob;
    return `body force (${show(num(f.fx))}, ${show(num(f.fy))}) N · `
      + `torque ${show(num(f.tz))} N·m`;
  }
  const f = forms.kid;
  return `to world (${show(num(f.x))}, ${show(num(f.y))}) · yaw ${show(num(f.yaw))} rad`;
}

/** The catalogue field each form input carries, for the bounds check. */
const FIELD_OF: Record<string, [SheetKey, string]> = {
  range_noise_stddev_m: ['scan', 'noise'],
  max_range_m: ['scan', 'maxr'],
  blind_sector_start_deg: ['scan', 'bs'],
  blind_sector_end_deg: ['scan', 'be'],
  dropout_probability: ['scan', 'drop'],
};

/**
 * Client-side validation, from the CATALOGUE's declared ranges only.
 *
 * No bound is written here. A kind whose `param_ranges` is empty is checked
 * for finiteness and nothing else, exactly as ral_faults/catalogue.py does -
 * and the manager remains the authority either way: this only stops a request
 * that could not possibly be honoured from being sent at all.
 */
export function validate(forms: Forms, reference: Reference): Record<string, string> {
  const errors: Record<string, string> = {};
  const finite = (key: string, value: string) => {
    if (num(value) === null) errors[key] = 'must be a finite number';
  };

  if (!/^[A-Za-z0-9_][A-Za-z0-9_-]{0,63}$/.test(forms.obs.name)) {
    errors.obs_name = 'not [A-Za-z0-9_][A-Za-z0-9_-]{0,63}';
  }
  finite('obs_x', forms.obs.x);
  finite('obs_y', forms.obs.y);
  finite('obs_yaw', forms.obs.yaw);
  for (const key of ['sx', 'sy', 'sz'] as const) {
    const value = num(forms.obs[key]);
    if (value === null || value <= 0) errors[`obs_${key}`] = 'must be > 0';
  }
  for (const key of ['fx', 'fy', 'tz'] as const) finite(`mob_${key}`, forms.mob[key]);
  for (const key of ['x', 'y', 'yaw'] as const) finite(`kid_${key}`, forms.kid[key]);

  const ranges = reference.catalogue.faults.scan_degradation?.param_ranges ?? {};
  for (const [field, [sheet, key]] of Object.entries(FIELD_OF)) {
    const raw = forms[sheet][key as keyof Forms[typeof sheet]] as string;
    const value = num(raw);
    const errorKey = `${sheet}_${key}`;
    if (value === null) {
      errors[errorKey] = 'must be a finite number';
      continue;
    }
    const bound = ranges[field];
    if (!bound) continue;
    const [lo, hi] = bound;
    if ((lo !== null && value < lo) || (hi !== null && value > hi)) {
      errors[errorKey] = `outside [${lo ?? 'None'}, ${hi ?? 'None'}]`;
    }
  }
  return errors;
}

export function hasErrors(errors: Record<string, string>, prefix: string): boolean {
  return Object.keys(errors).some((key) => key.startsWith(`${prefix}_`) && errors[key]);
}

export interface Preset {
  label: string;
  sub: string;
  apply: (forms: Forms) => Forms;
  matches: (forms: Forms) => boolean;
}

const close = (a: number | null, b: number) => a !== null && Math.abs(a - b) < 1e-9;

/**
 * Obstacle presets, from the world's OWN doorways plus the committed
 * obstacle scenario. Every pose named here exists in an artifact.
 */
export function obstaclePresets(reference: Reference,
  scenarios: ScenarioSummary[]): Preset[] {
  const presets: Preset[] = [];
  for (const door of reference.world_geometry.doorways) {
    const name = `${door.id.replace('wall_', 'door_')}_block`;
    const sx = 0.3;
    const sy = Math.max(0.1, Number((door.width_m + 0.1).toFixed(2)));
    presets.push({
      label: `Across ${door.label}`,
      sub: `world (${door.x.toFixed(2)}, ${door.y.toFixed(2)}) · ${sx} × ${sy} × 0.6 m · `
        + `gap ${door.width_m.toFixed(2)} m between ${door.between.join(' and ')}`,
      apply: (forms) => ({
        ...forms,
        obs: {
          name, x: String(door.x), y: String(door.y), yaw: '0.0',
          sx: String(sx), sy: String(sy), sz: '0.6',
        },
      }),
      matches: (forms) => close(num(forms.obs.x), door.x) && close(num(forms.obs.y), door.y),
    });
  }
  for (const scenario of scenarios) {
    for (const intervention of scenario.interventions) {
      if (intervention.type !== 'spawn_obstacle') continue;
      const p = intervention.params as unknown as {
        name: string; pose: { x: number; y: number; yaw: number };
        size: { x: number; y: number; z: number };
      };
      presets.push({
        label: `${scenario.experiment_id ?? scenario.file} · ${p.name}`,
        sub: `world (${p.pose.x.toFixed(2)}, ${p.pose.y.toFixed(2)}) · `
          + `${p.size.x} × ${p.size.y} × ${p.size.z} m · the committed scenario's obstacle`,
        apply: (forms) => ({
          ...forms,
          obs: {
            name: p.name, x: String(p.pose.x), y: String(p.pose.y),
            yaw: String(p.pose.yaw), sx: String(p.size.x), sy: String(p.size.y),
            sz: String(p.size.z),
          },
        }),
        matches: (forms) => close(num(forms.obs.x), p.pose.x)
          && close(num(forms.obs.y), p.pose.y)
          && close(num(forms.obs.sx), p.size.x),
      });
    }
  }
  return presets;
}

/**
 * Wrench presets, from the FORCES NAMED IN THIS PLANT'S OWN `mobility_note`.
 *
 * The note is measured text in the robot profile and the manager republishes
 * it verbatim in every `activated` detail. Taking the numbers out of it means
 * the console can never offer a force whose effect on this plant nobody
 * measured - and the note itself is shown beside the buttons, so the effect
 * is read from the measurement rather than from a caption invented here.
 */
export function wrenchPresets(profile: RobotProfile): Preset[] {
  const note = profile.mobility_note ?? '';
  const forces: number[] = [];
  for (const match of note.matchAll(/(-?\d+(?:\.\d+)?)\s*N\b/g)) {
    const value = Number(match[1]);
    if (Number.isFinite(value) && !forces.includes(value)) forces.push(value);
  }
  return forces.map((force) => ({
    label: `force_x_n ${force} N`,
    sub: `named in ${profile.robot}.yaml mobility_note, quoted in full above`,
    apply: (forms) => ({ ...forms, mob: { fx: String(force), fy: '0', tz: '0' } }),
    matches: (forms) => close(num(forms.mob.fx), force) && close(num(forms.mob.fy), 0),
  }));
}

/** Kidnap presets: the destinations the committed kidnap scenarios used. */
export function kidnapPresets(scenarios: ScenarioSummary[], robot: string): Preset[] {
  const presets: Preset[] = [];
  for (const scenario of scenarios) {
    if (scenario.robot !== robot) continue;
    for (const intervention of scenario.interventions) {
      if (intervention.type !== 'kidnap') continue;
      const p = intervention.params as unknown as {
        pose: { x: number; y: number; yaw: number };
      };
      presets.push({
        label: `to world (${p.pose.x.toFixed(2)}, ${p.pose.y.toFixed(2)})`,
        sub: `the pose ${scenario.experiment_id ?? scenario.file} teleports to`,
        apply: (forms) => ({
          ...forms,
          kid: { x: String(p.pose.x), y: String(p.pose.y), yaw: String(p.pose.yaw) },
        }),
        matches: (forms) => close(num(forms.kid.x), p.pose.x)
          && close(num(forms.kid.y), p.pose.y),
      });
    }
  }
  return presets;
}

/**
 * The scan presets. Only the first is a measurement; the rest isolate ONE
 * parameter each, and the heading says their effect was not measured.
 */
export function scanPresets(): Preset[] {
  const sets: [number, number, number, number, number, string, string][] = [
    [0.08, 2.5, 90, 150, 0.2, 'σ 0.08 · max 2.5 m · blind 90 → 150° · dropout 0.2',
      "the parameters in the manager's own example detail"],
    [0, 12, -45, 45, 0, 'blind sector −45 → 45° only',
      'front 90° blanked, other parameters neutral'],
    [0, 12, 0, 0, 0.5, 'dropout 0.5 only', 'per-beam no-return probability 0.5'],
    [0, 1.0, 0, 0, 0, 'max range 1.0 m only', 'returns beyond 1.0 m become no-returns'],
  ];
  return sets.map(([noise, maxr, bs, be, drop, label, sub]) => ({
    label,
    sub,
    apply: (forms: Forms) => ({
      ...forms,
      scan: {
        noise: String(noise), maxr: String(maxr), bs: String(bs), be: String(be),
        drop: String(drop),
      },
    }),
    matches: (forms: Forms) => close(num(forms.scan.noise), noise)
      && close(num(forms.scan.maxr), maxr) && close(num(forms.scan.bs), bs)
      && close(num(forms.scan.be), be) && close(num(forms.scan.drop), drop),
  }));
}

export function entryFor(reference: Reference, kind: string): CatalogueEntry | undefined {
  return reference.catalogue.faults[kind];
}

/** A free obstacle name, so a second spawn cannot collide with a live entity. */
export function uniqueName(base: string, used: Set<string>): string {
  if (!used.has(base)) return base;
  const match = /^(.*?)_?(\d+)$/.exec(base);
  const stem = match ? match[1] : base;
  let index = match ? Number(match[2]) + 1 : 2;
  while (used.has(`${stem}_${index}`)) index += 1;
  return `${stem}_${index}`;
}
