import { C, F } from '../theme/tokens';
import { Tip, f2, f3 } from '../ui/primitives';
import type { ConsoleCtx } from './context';
import {
  KIND_OF, SHEET_KEYS, buildParams, hasErrors, kidnapPresets, num, obstaclePresets,
  scanPresets, summarise, uniqueName, wrenchPresets,
} from './faults';
import type { Forms, Preset, SheetKey } from './faults';

const TAB_LABEL: [ConsoleCtx['tab'], string][] = [
  ['inject', 'Inject'], ['sensing', 'Sensing'], ['metrics', 'Live metrics'],
];

const SHEET_TITLE: Record<SheetKey, string> = {
  obs: 'Obstacles', scan: 'Lidar', mob: 'Wrench', kid: 'Kidnap',
};

/** The form fields each sheet edits, with the service field each one carries. */
const FIELDS: Record<SheetKey, [string, string, string][]> = {
  obs: [
    ['name', 'name', 'The gz entity name. Substituted into SDF, so it must match ^[A-Za-z0-9_][A-Za-z0-9_-]{0,63}$.'],
    ['x', 'pose.x · world m', 'gz WORLD frame, not map: an entity pose is a world pose by definition and is never converted.'],
    ['y', 'pose.y · world m', 'gz WORLD frame, not map.'],
    ['yaw', 'pose.yaw · rad', 'Rotation about +Z, sent as a quaternion by the runner\u2019s own request builder.'],
    ['sx', 'size.x · m > 0', 'Full extent, not half.'],
    ['sy', 'size.y · m > 0', 'Full extent, not half.'],
    ['sz', 'size.z · m > 0', 'Full extent. The manager places the box at z = size.z / 2 so it rests on the floor.'],
  ],
  scan: [
    ['noise', 'range_noise_stddev_m', 'Additive zero-mean Gaussian per beam.'],
    ['maxr', 'max_range_m', 'The modelled RPLIDAR A1 range. 0.0 would blank every beam and still report success, which is why the catalogue bounds it.'],
    ['bs', 'blind_sector_start_deg', 'Scan frame, CCW start → end.'],
    ['be', 'blind_sector_end_deg', 'start > end wraps through ±180; start = end blanks nothing.'],
    ['drop', 'dropout_probability', 'Per beam. The relay\u2019s RNG is seeded by the scenario seed, so the corrupted stream repeats.'],
  ],
  mob: [
    ['fx', 'force_x_n · body N', 'BODY frame. The manager re-projects into the world frame at 10 Hz as the body rotates.'],
    ['fy', 'force_y_n · body N', 'BODY frame.'],
    ['tz', 'torque_z_nm · N·m', 'About +Z.'],
  ],
  kid: [
    ['x', 'pose.x · world m', 'gz WORLD frame.'],
    ['y', 'pose.y · world m', 'gz WORLD frame.'],
    ['yaw', 'pose.yaw · rad', 'Rotation about +Z.'],
  ],
};

export function Drawer({ ctx }: { ctx: ConsoleCtx }) {
  return (
    <div style={{
      width: 360, flex: 'none', display: 'flex', flexDirection: 'column',
      background: C.drawer, minHeight: 0,
    }}
    >
      <div style={{ display: 'flex', borderBottom: `1px solid ${C.line}` }}>
        {TAB_LABEL.map(([key, label]) => (
          <button
            key={key}
            type="button"
            onClick={() => ctx.setTab(key)}
            style={{
              flex: 1, background: ctx.tab === key ? C.row : 'transparent',
              color: ctx.tab === key ? C.text : C.text4, border: 0,
              borderBottom: `2px solid ${ctx.tab === key ? C.text : 'transparent'}`,
              font: `500 11.5px ${F.sans}`, padding: '9px 4px', cursor: 'pointer',
            }}
          >
            {label}
          </button>
        ))}
      </div>
      <div style={{
        flex: 1, overflowY: 'auto', padding: 12, display: 'grid', gap: 10,
        alignContent: 'start',
      }}
      >
        {ctx.tab === 'inject' ? <InjectTab ctx={ctx} /> : null}
        {ctx.tab === 'sensing' ? <SensingTab ctx={ctx} /> : null}
        {ctx.tab === 'metrics' ? <MetricsTab ctx={ctx} /> : null}
      </div>
    </div>
  );
}

function InjectTab({ ctx }: { ctx: ConsoleCtx }) {
  const { state, reference, profile } = ctx;
  const activeList = state.active?.active ?? [];
  const sheet = ctx.sheet;
  const kind = KIND_OF[sheet];
  const entry = reference.catalogue.faults[kind];
  const supported = profile.faults.includes(kind);
  const unsupportedReason = profile.unsupported?.[kind] ?? '';
  const inForce = activeList.find((a) => a.kind === kind);
  const obstacleCount = Object.keys(state.obstacles).length;
  const kidnapCount = state.events
    .filter((e) => e.kind === 'kidnap' && e.state === 'activated').length;
  const errors = ctx.errors;
  const invalid = hasErrors(errors, sheet);
  const blocked = Boolean(ctx.blockReason);
  const pending = ctx.pending(sheet);

  const presets: Preset[] = sheet === 'obs'
    ? obstaclePresets(reference, ctx.scenarios)
    : sheet === 'scan'
      ? scanPresets()
      : sheet === 'mob'
        ? wrenchPresets(profile)
        : kidnapPresets(ctx.scenarios, profile.robot);

  const presetTitle = sheet === 'obs'
    ? 'presets · world frame, from the world SDF and the committed scenarios'
    : sheet === 'scan'
      ? 'parameter sets · effect on this plant not measured'
      : sheet === 'mob'
        ? `forces named in ${profile.robot}.yaml mobility_note`
        : (presets.length
          ? 'presets · the committed kidnap destinations for this robot'
          : 'no committed kidnap pose for this robot · place on map');

  const card = ctx.cardFor(sheet === 'obs' && ctx.cardFor('clear')
    && (!ctx.cardFor('obs')
      || ctx.cardFor('clear')!.sentSim > ctx.cardFor('obs')!.sentSim) ? 'clear' : sheet);

  const send = () => {
    if (blocked || invalid || pending || !supported) return;
    const params = buildParams(sheet, ctx.forms);
    const run = () => {
      ctx.sendFault(sheet, kind, params, summarise(sheet, ctx.forms));
      if (sheet === 'obs') {
        ctx.setForms((f: Forms) => ({
          ...f,
          obs: {
            ...f.obs,
            name: uniqueName(f.obs.name, new Set([
              ...Object.keys(state.obstacles), f.obs.name,
            ])),
          },
        }));
      }
    };
    if (sheet === 'kid') ctx.twoStep('kid', run);
    else run();
  };

  const sendLabel = pending
    ? 'Pending…'
    : sheet === 'obs'
      ? 'Spawn'
      : sheet === 'kid'
        ? (ctx.confirm === 'kid' ? 'Confirm kidnap (irreversible)' : 'Kidnap…')
        : (inForce ? 'Send (supersedes)' : 'Apply');

  return (
    <>
      {blocked ? (
        <div style={{
          border: `1px dashed ${C.border}`, padding: '6px 9px', font: `11px ${F.sans}`,
          color: C.text2,
        }}
        >
          {`Disabled · ${ctx.blockReason}`}
        </div>
      ) : null}

      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6 }}>
        {SHEET_KEYS.map((key, index) => {
          const tileKind = KIND_OF[key];
          const tileSupported = profile.faults.includes(tileKind);
          const tileActive = activeList.find((a) => a.kind === tileKind);
          const status = !tileSupported
            ? `unsupported on ${profile.robot}`
            : ctx.pending(key)
              ? 'pending…'
              : key === 'obs'
                ? (obstacleCount ? `${obstacleCount} in world` : 'none in world')
                : key === 'kid'
                  ? (kidnapCount ? `sent ×${kidnapCount}` : 'one-shot')
                  : (tileActive ? '▲ in force' : 'not in force');
          return (
            <button
              key={key}
              type="button"
              onClick={() => ctx.setSheet(key)}
              data-tip={!tileSupported ? profile.unsupported?.[tileKind] : undefined}
              style={{
                textAlign: 'left', background: sheet === key ? C.header : C.dim,
                border: `1px solid ${sheet === key ? C.text3 : C.lineSoft}`, color: C.text,
                padding: '7px 8px', cursor: 'pointer', display: 'grid', gap: 2,
                opacity: tileSupported ? 1 : 0.7,
              }}
            >
              <span style={{
                display: 'flex', justifyContent: 'space-between', alignItems: 'baseline',
              }}
              >
                <span style={{ font: `600 12px ${F.sans}` }}>{SHEET_TITLE[key]}</span>
                <span style={{
                  font: `600 10px ${F.mono}`,
                  color: sheet === key ? C.ink : C.text2,
                  background: sheet === key ? C.text : C.line, padding: '0 4px',
                }}
                >
                  {index + 1}
                </span>
              </span>
              <span style={{
                font: `500 10px ${F.mono}`,
                color: tileActive ? C.faultText : C.text5,
              }}
              >
                {status}
              </span>
            </button>
          );
        })}
      </div>

      <div style={{
        background: C.tab, border: `1px solid ${C.lineSoft}`, padding: 10, display: 'grid',
        gap: 8,
      }}
      >
        <div style={{ display: 'grid', gap: 2 }}>
          <span style={{ font: `600 12.5px ${F.mono}` }}>
            {sheet === 'obs' ? 'spawn_obstacle · clear_obstacle' : kind}
          </span>
          <span style={{ font: `10.5px ${F.sans}`, color: C.text4 }}>
            {entry
              ? `${entry.category} · ${entry.lifetime} · `
                + `${entry.alters_truth ? 'changes the world' : 'does not change the world'}`
              : ''}
          </span>
        </div>
        <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
          <span style={{
            font: `10px ${F.mono}`, color: C.text4, overflow: 'hidden',
            textOverflow: 'ellipsis', whiteSpace: 'nowrap', minWidth: 0, flex: 1,
          }}
          >
            {entry?.injection_point.replace('{world}', reference.world) ?? ''}
          </span>
          <Tip
            text={entry
              ? `Service ${entry.service} (${entry.srv_type}). Target rule: `
                + `${entry.target_rule}. Removal: ${entry.cleared_by ?? 'none — one-shot'}.`
              : ''}
          />
        </div>
        {entry?.honesty_note ? (
          <div style={{ display: 'grid', gap: 2 }}>
            <div style={{
              font: `11px/1.45 ${F.sans}`, color: C.text3, fontStyle: 'italic',
              maxHeight: ctx.open.hon ? 'none' : '2.9em', overflow: 'hidden',
            }}
            >
              {entry.honesty_note}
            </div>
            {entry.honesty_note.length > 160 ? (
              <button
                type="button"
                onClick={() => ctx.toggle('hon')}
                style={{
                  all: 'unset', justifySelf: 'start', font: `10.5px ${F.sans}`,
                  color: C.link, cursor: 'pointer',
                }}
              >
                {ctx.open.hon ? 'Less' : 'More'}
              </button>
            ) : null}
          </div>
        ) : null}
        {sheet === 'mob' && profile.mobility_note ? (
          <div style={{
            font: `11px/1.45 ${F.sans}`, color: C.text3, fontStyle: 'italic',
            borderTop: `1px solid ${C.line}`, paddingTop: 6,
          }}
          >
            {`On ${profile.robot}: ${profile.mobility_note}`}
          </div>
        ) : null}
        {!supported ? (
          <div style={{
            border: `1px dashed ${C.borderMax}`, padding: '7px 9px',
            font: `11.5px ${F.sans}`, color: C.text,
          }}
          >
            <span style={{ font: `500 10.5px ${F.mono}` }}>
              {`unsupported on ${profile.robot}`}
            </span>
            {` · ${unsupportedReason}`}
          </div>
        ) : null}

        <div style={{ display: 'grid', gap: 4 }}>
          <span style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>{presetTitle}</span>
          {presets.map((preset) => {
            const matched = preset.matches(ctx.forms);
            return (
              <button
                key={preset.label}
                type="button"
                disabled={!supported}
                onClick={() => ctx.setForms(preset.apply)}
                style={{
                  textAlign: 'left', background: matched ? '#343434' : 'transparent',
                  border: `1px solid ${matched ? '#9a9a9a' : C.lineMid}`, color: C.text,
                  padding: '5px 8px', cursor: supported ? 'pointer' : 'default',
                  display: 'grid', gap: 1, opacity: supported ? 1 : 0.5,
                }}
              >
                <span style={{ font: `500 11px ${F.mono}` }}>{preset.label}</span>
                <span style={{ font: `10px ${F.mono}`, color: C.text4 }}>{preset.sub}</span>
              </button>
            );
          })}
          {sheet === 'obs' || sheet === 'kid' ? (
            <button
              type="button"
              disabled={blocked}
              onClick={() => ctx.setPick(ctx.pick === sheet ? null : sheet)}
              style={{
                textAlign: 'left',
                background: ctx.pick === sheet ? C.command : 'transparent',
                border: `1px dashed ${C.command}`,
                color: ctx.pick === sheet ? C.ink : C.draftText,
                padding: '6px 8px', cursor: 'pointer', font: `500 11px ${F.sans}`,
              }}
            >
              {ctx.pick === sheet
                ? (sheet === 'obs'
                  ? 'Drawing on map… drag a rectangle · Esc cancels'
                  : 'Placing… click the destination, drag to set yaw · Esc cancels')
                : (sheet === 'obs'
                  ? 'Draw on map (P) · drag a rectangle, or click to place at the current size'
                  : 'Place on map (P) · click the destination, drag to set yaw')}
            </button>
          ) : null}
        </div>

        <div style={{
          display: 'grid', gap: 5, background: C.sunkenAlt, border: `1px solid ${C.line}`,
          padding: '7px 8px',
        }}
        >
          <div style={{ display: 'flex', alignItems: 'baseline', gap: 6 }}>
            <span style={{
              font: `600 10px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
            }}
            >
              DRAFT
            </span>
            <span style={{ flex: 1 }} />
            <button
              type="button"
              onClick={() => ctx.toggle('params')}
              style={{
                all: 'unset', font: `10.5px ${F.sans}`, color: C.link, cursor: 'pointer',
              }}
            >
              {ctx.open.params ? 'Hide fields' : 'Edit'}
            </button>
          </div>
          <span style={{
            font: `10.5px ${F.mono}`, color: invalid ? C.bad : C.text,
            wordBreak: 'break-word',
          }}
          >
            {invalid
              ? `invalid: ${Object.keys(errors)
                .filter((key) => key.startsWith(`${sheet}_`) && errors[key])
                .map((key) => `${key.slice(sheet.length + 1)} ${errors[key]}`)
                .join('; ')}`
              : summarise(sheet, ctx.forms)}
          </span>
          {(sheet === 'obs' || sheet === 'kid') && !invalid ? (
            <span style={{ font: `10px ${F.mono}`, color: C.text4 }}>
              {(() => {
                const form = sheet === 'obs' ? ctx.forms.obs : ctx.forms.kid;
                const x = num(form.x);
                const y = num(form.y);
                if (x === null || y === null) return '';
                const w = reference.world_to_map;
                const mx = (x - w.x) * Math.cos(w.yaw) + (y - w.y) * Math.sin(w.yaw);
                const my = -(x - w.x) * Math.sin(w.yaw) + (y - w.y) * Math.cos(w.yaw);
                const extra = sheet === 'obs'
                  ? ` · z ${f3((num(ctx.forms.obs.sz) ?? 0) / 2)} m ◇ derived (size.z / 2)`
                  : ` · z substituted by the manager (kidnap_z_m ${profile.kidnap_z_m} m)`;
                return `map (${f3(mx)}, ${f3(my)}) via world_to_map${extra}`;
              })()}
            </span>
          ) : null}
          {ctx.open.params ? (
            <div style={{
              display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6, paddingTop: 4,
            }}
            >
              {FIELDS[sheet].map(([field, label, tip]) => {
                const key = `${sheet}_${field}`;
                const error = errors[key];
                const formValues = ctx.forms[sheet] as unknown as Record<string, string>;
                return (
                  <label key={field} style={{ display: 'grid', gap: 2, minWidth: 0 }}>
                    <span style={{
                      font: `10px ${F.mono}`, color: C.text4, overflow: 'hidden',
                      textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                    }}
                      data-tip={tip}
                    >
                      {label}
                    </span>
                    <input
                      value={formValues[field] ?? ''}
                      disabled={!supported}
                      onChange={(event) => {
                        const { value } = event.target;
                        ctx.setForms((forms) => ({
                          ...forms,
                          [sheet]: { ...forms[sheet], [field]: value },
                        }));
                      }}
                      style={{
                        background: C.ink, border: `1px solid ${error ? '#c07070' : C.lineMid}`,
                        color: C.text, fontSize: 11.5, padding: '4px 5px', width: '100%',
                      }}
                    />
                    <span style={{ font: `10px ${F.sans}`, color: C.bad }}>{error ?? ''}</span>
                  </label>
                );
              })}
            </div>
          ) : null}
        </div>

        {inForce && entry?.lifetime === 'singleton' ? (
          <div style={{
            font: `11px ${F.sans}`, color: C.text, background: C.warnBg, padding: '5px 8px',
          }}
          >
            {`${inForce.id} is in force. Sending again supersedes it: that instance is `
              + 'cleared with “superseded by <new id>”.'}
          </div>
        ) : null}
        {sheet === 'kid' ? (
          <div style={{
            font: `11px ${F.sans}`, color: C.text, background: C.warnBg, padding: '5px 8px',
          }}
          >
            {`z is substituted by the manager (kidnap_z_m ${profile.kidnap_z_m} m). Linear `
              + 'and angular velocity are not zeroed. Irreversible: no clear exists.'}
          </div>
        ) : null}

        <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>
          <button
            type="button"
            disabled={blocked || invalid || pending || !supported}
            onClick={send}
            style={{
              background: (sheet === 'kid' && ctx.confirm !== 'kid') ? 'transparent' : C.text,
              color: (sheet === 'kid' && ctx.confirm !== 'kid') ? C.text : C.ink,
              border: `1px solid ${C.text}`, font: `600 11.5px ${F.sans}`,
              padding: '5px 11px', cursor: 'pointer',
              opacity: blocked || invalid || pending || !supported ? 0.45 : 1,
            }}
          >
            {sendLabel}
          </button>
          <button
            type="button"
            disabled={blocked || invalid || !supported || ctx.armed.length >= 4}
            onClick={ctx.arm}
            style={{
              background: 'transparent', color: C.text, border: `1px dashed ${C.text5}`,
              font: `500 11.5px ${F.sans}`, padding: '5px 10px', cursor: 'pointer',
              opacity: blocked || invalid || !supported || ctx.armed.length >= 4 ? 0.45 : 1,
            }}
          >
            Arm · send later
          </button>
          {inForce && entry?.cleared_by === 'self_disable' ? (
            <button
              type="button"
              disabled={blocked || pending}
              onClick={() => ctx.sendFault(sheet, kind, { enabled: false }, 'enabled: false')}
              style={{
                background: 'transparent', color: C.text, border: `1px solid ${C.borderAlt}`,
                font: `500 11.5px ${F.sans}`, padding: '5px 10px', cursor: 'pointer',
              }}
            >
              Disable
            </button>
          ) : null}
        </div>

        {card ? (
          <div style={{
            display: 'grid', gap: 3, background: C.sunken, border: `1px solid ${C.line}`,
            padding: '6px 8px',
          }}
          >
            <span style={{
              font: `500 10.5px ${F.mono}`,
              color: card.status === 'resolved' && card.response && !card.response.success
                ? C.bad : C.text,
            }}
            >
              {card.status === 'pending'
                ? `pending · sent on ${card.service} at sim ${f2(card.sentSim)} s · `
                  + 'awaiting response'
                : card.status === 'failed'
                  ? 'no response: the request failed before the manager answered'
                  : `response · success=${card.response?.success} · `
                    + `${card.response?.fault_id || "fault_id ''"} · sim `
                    + `${f2(card.atSim ?? 0)} s`}
            </span>
            <span style={{
              font: `10.5px/1.45 ${F.mono}`,
              color: card.response?.success === false ? C.bad : C.text2,
              wordBreak: 'break-word', whiteSpace: 'pre-wrap', maxHeight: 140,
              overflow: 'auto',
            }}
            >
              {card.status === 'pending'
                ? 'Nothing in the world has changed yet.'
                : (card.error ?? card.response?.message ?? '')}
            </span>
          </div>
        ) : null}

        {sheet === 'obs' ? (
          <div style={{
            display: 'grid', gap: 4, borderTop: `1px solid ${C.line}`, paddingTop: 8,
          }}
          >
            <span style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>
              in the world · click one on the map to clear
            </span>
            {obstacleCount === 0 ? (
              <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>none</span>
            ) : null}
            {Object.keys(state.obstacles).map((name) => {
              const o = state.obstacles[name];
              const disabled = blocked || ctx.pending('clear');
              return (
                <div key={name} style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
                  <span style={{
                    font: `500 11px ${F.mono}`, flex: 1, overflow: 'hidden',
                    textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                  }}
                  >
                    {name}
                    <span style={{ color: C.text4 }}>
                      {` (${f2(o.x)}, ${f2(o.y)}) · ${f2(o.sx)}×${f2(o.sy)}`}
                    </span>
                  </span>
                  <button
                    type="button"
                    disabled={disabled}
                    onClick={() => ctx.sendFault('clear', 'clear_obstacle', { name },
                      `clear '${name}'`)}
                    style={{
                      background: 'transparent', border: `1px solid ${C.borderAlt}`,
                      color: C.text, font: `500 10.5px ${F.sans}`, padding: '2px 8px',
                      cursor: 'pointer', opacity: disabled ? 0.5 : 1,
                    }}
                  >
                    {ctx.pending('clear') ? 'pending…' : 'Clear'}
                  </button>
                </div>
              );
            })}
          </div>
        ) : null}
      </div>
    </>
  );
}

function SensingTab({ ctx }: { ctx: ConsoleCtx }) {
  const { state, profile, isCar } = ctx;
  const scan = state.scan;
  const raw = state.scanRaw;
  const returns = scan ? scan.ranges.filter((r) => r !== null).length : null;

  // Only beams from the SAME header stamp are comparable: /scan is a republish
  // of /scan_raw, and comparing two different sweeps would count the robot's
  // own motion as degradation.
  let differing: number | null = null;
  if (scan && raw && scan.stamp_sim_s === raw.stamp_sim_s) {
    differing = 0;
    for (let i = 0; i < Math.min(scan.ranges.length, raw.ranges.length); i += 1) {
      if (scan.ranges[i] !== raw.ranges[i]) differing += 1;
    }
  }

  const activeList = state.active?.active ?? [];
  const mobility = activeList.find((a) => a.kind === 'mobility_fault');
  const lastAchieved = state.achieved[state.achieved.length - 1];

  let deficit = 'no mobility_fault in force';
  if (mobility) {
    const before = state.achieved.filter((a) => a.t < mobility.stamp_sim_s
      && a.t >= mobility.stamp_sim_s - 10);
    const after = state.achieved.filter((a) => a.t >= mobility.stamp_sim_s + 0.2);
    if (!after.length) {
      deficit = 'no samples since activation yet';
    } else if (!before.length) {
      deficit = 'no pre-activation samples: the wrench was applied before the body moved';
    } else {
      const meanBefore = before.reduce((s, a) => s + a.v, 0) / before.length;
      const meanAfter = after.reduce((s, a) => s + a.v, 0) / after.length;
      deficit = meanBefore === 0
        ? `mean ${f3(meanAfter)} vs ${f3(meanBefore)} m/s: no pre-activation speed to `
          + 'compare against'
        : `${((1 - meanAfter / meanBefore) * 100).toFixed(1)}% · mean ${f3(meanAfter)} `
          + `vs ${f3(meanBefore)} m/s over ${after.length}/${before.length} samples`;
    }
  }

  const bumperAvailable = state.stack?.bumper_available;
  const contactRow = bumperAvailable === false
    ? state.stack?.bumper_reason ?? 'contact decoding unavailable'
    : state.bumperHits.length
      ? `${state.bumperHits.length} contact(s): `
        + `${state.bumperHits.slice(-3).map((h) => `${h.pair} @ ${f2(h.t)} s`).join(', ')}`
      : 'no contact reported on /bumper';

  return (
    <>
      <div style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
        LIDAR
      </div>
      <div style={{
        display: 'grid', gridTemplateColumns: '118px minmax(0,1fr)', gap: '4px 8px',
        fontSize: 11,
      }}
      >
        <span style={{ color: C.text4 }}>returns</span>
        <span style={{ fontFamily: F.mono }}>
          {returns === null ? 'no /scan yet' : `${returns} / ${scan!.ranges.length} ●`}
        </span>
        <span style={{ color: C.text4 }}>no-returns</span>
        <span style={{ fontFamily: F.mono }}>
          {returns === null ? '—' : String(scan!.ranges.length - returns)}
        </span>
        <span style={{ color: C.text4 }}>/scan ≠ /scan_raw</span>
        <span style={{ fontFamily: F.mono, display: 'flex', gap: 6, alignItems: 'center' }}>
          {differing === null
            ? 'no matching stamps yet'
            : `${differing} of ${scan!.ranges.length}`}
          <span style={{ color: C.text4 }}>◇</span>
          <Tip text="Counted only when the two messages carry the SAME header stamp. Two different sweeps would differ because the robot moved, not because the stream was degraded." />
        </span>
      </div>

      <div style={{
        font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4, marginTop: 6,
      }}
      >
        COMMANDED VS ACHIEVED
      </div>
      <div style={{
        display: 'grid', gridTemplateColumns: '118px minmax(0,1fr)', gap: '5px 8px',
        fontSize: 11, alignItems: 'baseline',
      }}
      >
        <span style={{ color: C.text4 }}>
          {isCar ? 'command' : `${profile.command.topic} linear.x`}
        </span>
        <span style={{ fontFamily: F.mono, display: 'flex', gap: 6, alignItems: 'center' }}>
          {isCar
            ? `position setpoint · cruise ${profile.plant.max_linear_m_s} m/s`
            : (state.cmd ? `${f3(state.cmd.linear_x)} m/s` : 'no command yet')}
          <span style={{ color: C.command }}>▶</span>
          <Tip
            text={isCar
              ? `Operator command on ${profile.command.topic}. There is no velocity command; cruise is the autopilot's CRUISE_SPEED / WP_SPEED, per ${profile.plant.source}.`
              : 'Operator command, velocity_smoother output. DiffDrive holds the last command indefinitely, so no new command does not mean stopped.'}
          />
        </span>
        <span style={{ color: C.text4 }}>achieved</span>
        <span style={{ fontFamily: F.mono, display: 'flex', gap: 6, alignItems: 'center' }}>
          {lastAchieved
            ? `${f3(lastAchieved.v)} m/s @ ${f2(lastAchieved.t)} s`
            : 'not enough samples'}
          <span style={{ color: C.truth }}>■</span>
          <Tip text="Central difference over 0.2 s of /ground_truth/odom positions, projected on the body heading. Derived from truth, so it is evaluation-only." />
        </span>
        <span style={{ color: C.text4 }}>mobility deficit</span>
        <span style={{ fontFamily: F.mono, display: 'flex', gap: 6, alignItems: 'center' }}>
          {deficit}
          <span style={{ color: C.text4 }}>◇</span>
          <Tip text="Mean achieved speed after activation against the 10 s before it, from the same stream. A live comparison, not metrics.json's windowed speed_vs_target." />
        </span>
      </div>

      <div style={{
        font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4, marginTop: 6,
      }}
      >
        CONTACT
      </div>
      <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
        <span style={{ font: `11px ${F.mono}`, flex: 1 }}>{contactRow}</span>
        <Tip text="/bumper is event-driven: a gz contact sensor publishes only while two bodies touch, so silence is the normal case. A zero COUNT needs run.json.bumper_liveness, which exists only after a recorded run." />
      </div>
    </>
  );
}

function MetricsTab({ ctx }: { ctx: ConsoleCtx }) {
  const { state, profile, isCar } = ctx;
  const inWindow = state.tAccept !== null;
  const windowPairs = state.pairs.filter((p) => p.win);
  const afterRun = 'computed after the run from the bag';
  const noWindow = state.tTerminal !== null && !inWindow
    ? 'no mission window: the goal was never accepted'
    : 'no goal_accepted yet';

  const rows: [string, string, string][] = [];
  const value = (key: string, text: string, meta: string) => rows.push([key, text, meta]);
  const unavailable = (key: string, why: string) => rows.push([key, 'unavailable', why]);

  if (inWindow) {
    value('mission_duration_s',
      f2((state.tTerminal ?? state.simNow) - state.tAccept!),
      's (sim) · /mission_state goal_accepted → terminal');
    value('path_length_m', f3(state.pathLength),
      `m · /ground_truth/odom in window; teleport_excluded_m ${f3(state.teleportExcluded)}`);
  } else {
    unavailable('mission_duration_s', noWindow);
    unavailable('path_length_m', noWindow);
  }
  if (state.tTerminal !== null && state.goalError !== null) {
    value('goal_pose_error_m', f3(state.goalError),
      'm · /ground_truth/odom at terminal vs the goal, in world');
  } else {
    unavailable('goal_pose_error_m', state.tTerminal !== null
      ? 'the goal was never sent' : 'available after a terminal state');
  }
  if (windowPairs.length) {
    const mean = windowPairs.reduce((s, p) => s + p.e, 0) / windowPairs.length;
    value('est_error_mean_m', f3(mean), `m · ${windowPairs.length} pairs within 50 ms`);
    // reduce, not Math.max(...array): a long mission pairs thousands of
    // samples, and spreading them as arguments overflows the call stack.
    const worst = windowPairs.reduce((best, p) => (p.e > best ? p.e : best), 0);
    value('est_error_max_m', f3(worst),
      `m · ${windowPairs.length} pairs within 50 ms`);
  } else {
    const why = inWindow ? 'no paired samples in window yet' : noWindow;
    unavailable('est_error_mean_m', why);
    unavailable('est_error_max_m', why);
  }
  if (profile.estimate.uncertainty.kind !== 'covariance_trace') {
    unavailable('est_cov_trace_max',
      `${profile.robot} publishes no numeric pose uncertainty; see the side panel`);
  } else if (state.covMax !== null) {
    value('est_cov_trace_max', f3(state.covMax),
      `${profile.estimate.topic} cov[0]+cov[7]+cov[35]`);
  } else {
    unavailable('est_cov_trace_max',
      inWindow ? `no ${profile.estimate.topic} in window yet` : noWindow);
  }
  if (isCar) {
    unavailable('plan_publications', 'this platform publishes no /plan');
    unavailable('recoveries', 'no Nav2 on this platform');
  } else if (inWindow) {
    value('plan_publications', String(state.planTimes.filter(
      (t) => t >= state.tAccept! && (state.tTerminal === null || t <= state.tTerminal),
    ).length), 'count · /plan publications, not replans');
    value('recoveries', state.feedback ? String(state.feedback.number_of_recoveries) : '—',
      "count · Nav2's own number_of_recoveries");
  } else {
    unavailable('plan_publications', noWindow);
    unavailable('recoveries', 'no Nav2 feedback yet');
  }
  unavailable('collision_count', state.stack?.bumper_available === false
    ? (state.stack.bumper_reason)
    : `${state.bumperHits.length} debounced contact(s) seen live; a COUNT needs `
      + 'run.json.bumper_liveness, which exists only after a recorded run');
  for (const key of ['fault_to_response_s', 'scan_degradation_divergence',
    'obstacle_min_clearance_m', 'est_error_at_start_pose_m',
    'localization_converged_sim_s']) {
    unavailable(key, `${afterRun}; a dashboard mission records no bag`);
  }

  return (
    <>
      <div style={{ font: `11px ${F.sans}`, color: C.text4 }}>
        ◇ derived (live) from the same streams metrics.py reads. A dashboard mission
        writes no run directory, so nothing here is replaced by a metrics.json.
      </div>
      {rows.map(([key, text, meta]) => (
        <div
          key={key}
          style={{
            borderBottom: '1px solid #2e2e2e', padding: '5px 0', display: 'flex', gap: 8,
            alignItems: 'center',
          }}
        >
          <span style={{ font: `500 11px ${F.mono}`, color: C.textBright, flex: 1 }}>
            {key}
          </span>
          <span style={{
            font: `600 11.5px ${F.mono}`, color: text === 'unavailable' ? C.text3 : C.text,
            textAlign: 'right',
          }}
          >
            {text}
          </span>
          <Tip text={meta} />
        </div>
      ))}
    </>
  );
}
