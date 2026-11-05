import { C, F } from '../theme/tokens';
import { CopyButton, Tip, f2 } from '../ui/primitives';
import { footprintLabel } from './draw';
import type { ConsoleCtx } from './context';

const linkTab = {
  font: `500 11.5px ${F.sans}`, padding: '4px 8px', color: C.text3,
} as const;

/**
 * The clock word.
 *
 * `stalled` is a measurement about /clock, not about the link: the browser
 * says "last received" when ITS connection is down, and only claims the
 * simulation stopped when the connection is up and no /clock arrived for
 * 2 s wall. Conflating the two would blame gz for a dropped socket - or, in
 * replay, for a paused playhead, which is why a recording reports its own
 * transport state instead of pretending to measure a simulator that is not
 * running.
 */
function clockWord(ctx: ConsoleCtx): [string, string, string] {
  const { state } = ctx;
  const recording = ctx.source.kind === 'replay' ? ctx.source.replay : undefined;
  if (recording) {
    if (recording.closed()) return ['■', 'recording ended', C.text2];
    if (!recording.playing()) return ['❙❙', 'recording paused', C.warn];
    return ['▶', `recorded /clock at ${recording.speed()}×`, C.text2];
  }
  if (!state.linkUp) return ['–', 'last received', C.warn];
  if (state.stack?.state === 'starting') return ['–', 'bring-up', C.warn];
  const since = Date.now() - state.lastClockWallMs;
  if (state.simNow === 0) return ['–', 'no /clock yet', C.warn];
  if (since > 2000) return ['■', 'stalled', C.warn];
  return ['▶', 'advancing', C.text2];
}

export function TopBar({ ctx }: { ctx: ConsoleCtx }) {
  const { state, reference, profile, source } = ctx;
  const [glyph, word, colour] = clockWord(ctx);
  const stack = state.stack;
  const mission = state.mission;
  const robotLabel = `${profile.model} · ${profile.robot}`;
  // Non-null only in replay mode: the identity of the RECORDING on screen.
  const rec = source.kind === 'replay' ? (source.replay?.info() ?? null) : null;

  const provRows: [string, string][] = [
    ...(rec ? ([
      ['replay run_id', rec.run_id],
      ['replay outcome', `${rec.outcome} · ${rec.headline}`],
      ['replay span', `sim ${rec.t_start.toFixed(2)} – ${rec.t_end.toFixed(2)} s · `
        + `${rec.frames} recorded frames`],
      ['replay bag', rec.provenance.bag ?? 'not recorded in the fixture'],
      ['replay git_sha', `${rec.provenance.git_sha ?? 'not recorded in the fixture'}`
        + `${rec.provenance.git_dirty ? ' · git_dirty: the tree had uncommitted changes'
          + ' when this run was recorded' : ''}`],
      ['replay image_ref', rec.provenance.image_ref ?? 'not recorded in the fixture'],
      ['replay exported', rec.provenance.exported_utc ?? 'not recorded in the fixture'],
      ['replay decimation', Object.entries(rec.provenance.decimation ?? {})
        .map(([topic, rule]) => `${topic}: ${rule}`).join(' · ') || 'none recorded'],
      ['replay topics', Object.entries(rec.provenance.topic_status ?? {})
        .filter(([, status]) => status !== 'read')
        .map(([topic, status]) => `${topic}: ${status}`).join(' · ')
        || 'every exported topic was read from the bag'],
      ['replay grid', rec.provenance.grid_source ?? 'from the bag'],
      ['replay reference', rec.reference_fallback
        ? 'FALLBACK: the fixture carried no /api/reference, so the committed design '
          + 'fixture is shown and may not match this run'
        : 'from the fixture, as /api/reference answered during the run'],
      ['replay control', 'none: this is a recording, no container, no gz, no autonomy '
        + 'stack. scripts/dashboard.sh runs the live console.'],
    ] as [string, string][]) : []),
    ['run / experiment', mission ? mission.experiment_id : 'no mission state yet'],
    ['world', reference.world],
    ['map_sha256', reference.map.map_sha256],
    ['map', `${reference.map.width} × ${reference.map.height} cells at `
      + `${reference.map.resolution} m · origin [${reference.map.origin.join(', ')}]`],
    ['world_to_map', `{x: ${reference.world_to_map.x}, y: ${reference.world_to_map.y}, `
      + `yaw: ${reference.world_to_map.yaw}} · ${reference.alignment_path}`],
    ['world_to_ekf_local', ctx.isCar
      ? (state.align
        ? `{x: ${state.align.x.toFixed(4)}, y: ${state.align.y.toFixed(4)}, `
          + `yaw: ${state.align.yaw.toFixed(4)}} · residual ${state.align.residual_m} m · `
          + `${state.align.source}`
        : '/frame_alignment not received')
      : `n/a · ${profile.robot} has no ekf_local`],
    ['robot profile', profile.profile_path],
    ['stack launch', stack?.log_path
      ? `${profile.stack_launch} · log ${stack.log_path}`
      : `${profile.stack_launch} · not launched by this dashboard`],
  ];

  return (
    <div style={{
      position: 'relative', zIndex: 30, display: 'flex', alignItems: 'center', gap: 14,
      padding: '0 12px', height: 46, borderBottom: `1px solid ${C.line}`,
      background: C.inset, whiteSpace: 'nowrap', minWidth: 0,
    }}
    >
      <span style={{ font: `600 12px ${F.sans}`, letterSpacing: '.1em' }}>RAL</span>
      <div style={{ display: 'flex', gap: 2 }}>
        <span style={{
          font: `500 11.5px ${F.sans}`, padding: '4px 8px', background: C.header,
          color: C.text,
        }}
        >
          Console
        </span>
        <button type="button" onClick={() => ctx.go('runs')}
          style={{ ...linkTab, background: 'transparent', border: 0, cursor: 'pointer' }}
        >
          Runs
        </button>
        <button type="button" onClick={() => ctx.go('compare')}
          style={{ ...linkTab, background: 'transparent', border: 0, cursor: 'pointer' }}
        >
          Compare
        </button>
        <button type="button" onClick={() => ctx.go('reference')}
          style={{ ...linkTab, background: 'transparent', border: 0, cursor: 'pointer' }}
        >
          Reference
        </button>
      </div>
      <span style={{ width: 1, height: 22, background: C.lineSoft }} />

      <button
        type="button"
        onClick={() => ctx.setPop(ctx.pop === 'robot' ? null : 'robot')}
        style={{
          display: 'flex', alignItems: 'center', gap: 8,
          background: ctx.pop === 'robot' ? C.line : 'transparent',
          border: `1px solid ${C.border}`, color: C.text, padding: '4px 9px',
          cursor: 'pointer',
        }}
      >
        <span style={{ font: `600 12px ${F.mono}` }}>{robotLabel}</span>
        <span style={{ font: `10px ${F.mono}`, color: C.text4 }}>▾</span>
      </button>

      <div style={{ display: 'flex', alignItems: 'baseline', gap: 6 }}>
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>sim</span>
        <span style={{
          font: `600 14px ${F.mono}`, minWidth: 56, textAlign: 'right',
        }}
        >
          {state.simNow.toFixed(1)}
        </span>
        <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>s</span>
        <span style={{ font: `500 11px ${F.mono}`, color: colour }}>
          {glyph}
          {' '}
          {word}
        </span>
      </div>

      <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>RTF</span>
        <span style={{ font: `500 12.5px ${F.mono}` }}>
          {state.rtf ? state.rtf.samples.split(',')[0].trim().slice(0, 4) : '—'}
        </span>
        <Tip
          text={state.rtf
            ? `Real-time factor, gz windowed: ${state.rtf.samples} (${state.rtf.source}). `
              + 'Every measured duration is sim time, so a low factor is not degradation.'
            : 'Real-time factor is read from gz only while a stack is up.'}
        />
      </div>

      <div style={{ display: 'flex', alignItems: 'baseline', gap: 6 }}>
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>
          {rec ? 'frame source' : 'dashboard link'}
        </span>
        <span style={{
          font: `500 11.5px ${F.mono}`, color: state.linkUp ? C.text2 : C.warn,
        }}
        >
          {rec
            ? '◼ static file'
            : (state.linkUp
              ? '● connected'
              : `○ disconnected ${state.linkDownWallMs
                ? f2((Date.now() - state.linkDownWallMs) / 1000) : '0.00'} s wall`)}
        </span>
        <Tip
          text={rec
            ? 'There is no connection. The frames come from a committed JSON export of '
              + `${rec.provenance.bag ?? 'the run\'s bag'}; no socket, no gateway, no `
              + 'ROS graph is involved.'
            : "JSON over SSE between this browser and ral_eval's dashboard gateway. It is not a robotics protocol and carries no ROS messages."}
        />
      </div>

      <button
        type="button"
        onClick={() => ctx.setPop(ctx.pop === 'prov' ? null : 'prov')}
        style={{
          display: 'flex', alignItems: 'baseline', gap: 6, minWidth: 40, flex: '1 1 0',
          overflow: 'hidden', background: 'transparent', border: 0, color: C.text,
          padding: 0, cursor: 'pointer', textAlign: 'left',
        }}
      >
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>{rec ? 'replay' : 'run'}</span>
        <span style={{
          font: `500 11.5px ${F.mono}`, overflow: 'hidden', textOverflow: 'ellipsis',
          minWidth: 0, textDecoration: `underline dotted ${C.borderMax}`,
        }}
        >
          {rec ? rec.run_id : (mission ? mission.experiment_id : '—')}
        </span>
      </button>

      <span
        title={rec
          ? `REPLAY of ${rec.run_id} (${rec.robot} in ${rec.world}) · outcome `
            + `${rec.outcome} · git_sha ${rec.provenance.git_sha ?? 'unrecorded'} · `
            + `image_ref ${rec.provenance.image_ref ?? 'unrecorded'} · exported `
            + `${rec.provenance.exported_utc ?? 'unrecorded'} · nothing here is live`
          : undefined}
        style={{
          font: `500 10.5px ${F.mono}`, letterSpacing: '.04em',
          border: `1px solid ${rec ? C.warn : C.borderAlt}`, padding: '3px 7px',
          color: rec ? C.warn : undefined,
        }}
      >
        {source.kind === 'replay' ? 'REPLAY' : (source.kind === 'mock' ? 'MOCK' : 'LIVE')}
      </span>
      <button type="button" onClick={() => ctx.setPop(ctx.pop === 'legend' ? null : 'legend')}
        style={{
          background: 'transparent', border: `1px solid ${C.lineBright}`, color: C.text,
          font: `500 11.5px ${F.sans}`, padding: '4px 9px', cursor: 'pointer',
        }}
      >
        Legend
      </button>
      <button type="button" onClick={() => ctx.setPop(ctx.pop === 'keys' ? null : 'keys')}
        style={{
          background: 'transparent', border: `1px solid ${C.lineBright}`, color: C.text,
          font: `500 11.5px ${F.sans}`, padding: '4px 9px', cursor: 'pointer',
        }}
      >
        Keys
      </button>

      {ctx.pop === 'robot' ? <RobotPopover ctx={ctx} /> : null}
      {ctx.pop === 'prov' ? <ProvPopover rows={provRows} /> : null}
      {ctx.pop === 'legend' ? <LegendPopover /> : null}
      {ctx.pop === 'keys' ? <KeysPopover /> : null}
    </div>
  );
}

function RobotPopover({ ctx }: { ctx: ConsoleCtx }) {
  const { reference, state } = ctx;
  const stack = state.stack;
  const running = stack && stack.state !== 'idle';
  const robots = Object.keys(reference.profiles).sort();
  const busy = ctx.lifecycle?.status === 'pending';

  return (
    <div style={{
      position: 'absolute', left: 300, top: 44, width: 640, background: C.dim,
      border: `1px solid ${C.lineBright}`, boxShadow: '0 10px 30px rgba(0,0,0,.55)',
      padding: 14, display: 'grid', gap: 12, whiteSpace: 'normal',
    }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }}>
        <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
          ROBOT
        </span>
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>
          switching = stack relaunch
        </span>
      </div>
      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 10 }}>
        {robots.map((name) => {
          const p = reference.profiles[name];
          const attached = stack?.robot === name;
          const faultCount = p.faults.length;
          const unsupported = Object.entries(p.unsupported ?? {});
          return (
            <div
              key={name}
              style={{
                background: C.sunken, border: `1px solid ${attached ? '#9a9a9a' : C.lineSoft}`,
                padding: 10, display: 'grid', gap: 7, alignContent: 'start',
              }}
            >
              <div style={{
                display: 'flex', justifyContent: 'space-between', alignItems: 'baseline',
              }}
              >
                <span style={{ font: `600 13px ${F.mono}` }}>{`${p.model} · ${name}`}</span>
                <span style={{
                  font: `500 10.5px ${F.sans}`, color: attached ? C.text : C.text4,
                }}
                >
                  {attached ? `attached · ${stack?.state}` : 'not attached'}
                </span>
              </div>
              <div style={{
                display: 'grid', gridTemplateColumns: '70px minmax(0,1fr)', gap: '3px 8px',
                fontSize: 11,
              }}
              >
                <span style={{ color: C.text4 }}>estimator</span>
                <span style={{ fontFamily: F.mono }}>
                  {`${p.estimator_label} · ${p.estimate.topic}`}
                </span>
                <span style={{ color: C.text4 }}>planner</span>
                <span style={{ fontFamily: F.mono }}>{p.mission_driver}</span>
                <span style={{ color: C.text4 }}>command</span>
                <span style={{ fontFamily: F.mono }}>{p.command.label}</span>
                <span style={{ color: C.text4 }}>plant</span>
                <span style={{ fontFamily: F.mono }}>
                  {`${p.plant.max_linear_m_s} m/s · ${p.plant.max_angular_rad_s} rad/s · `
                    + `min turn ${p.plant.min_turn_radius_m} m`}
                </span>
                <span style={{ color: C.text4 }}>footprint</span>
                <span style={{ fontFamily: F.mono }}>
                  {p.footprint.shape === 'rectangle'
                    ? `${footprintLabel(p)} box · circumscribed r ${p.footprint.radius_m} m`
                    : `radius ${p.footprint.radius_m} m`}
                </span>
                <span style={{ color: C.text4 }}>faults</span>
                <span style={{ fontFamily: F.mono }}>
                  {unsupported.length
                    ? `${faultCount} · ${unsupported.map(([k]) => k).join(', ')} unsupported`
                    : `${faultCount} · all supported`}
                </span>
                <span style={{ color: C.text4 }}>bring-up</span>
                <span style={{ fontFamily: F.mono }}>
                  {`${p.bringup_wall_limit_s} s wall budget`}
                </span>
              </div>
              <div style={{
                display: 'grid', gap: 6, borderTop: `1px solid ${C.line}`, paddingTop: 8,
              }}
              >
                {running ? (
                  <button
                    type="button"
                    disabled={busy || stack?.mission_active}
                    onClick={() => {
                      // `attached` means this IS the running robot, so the
                      // button is a plain stop. Otherwise it must stop and
                      // then start - issuing both at once left the start
                      // refused by the running stack's own /clock.
                      if (attached) {
                        ctx.setStartRobot(name);
                        ctx.stopStack();
                      } else {
                        ctx.stopThenStart(name);
                      }
                    }}
                    style={{
                      justifySelf: 'start', background: 'transparent',
                      border: `1px solid ${C.border}`, color: C.text,
                      font: `500 11px ${F.sans}`, padding: '3px 9px',
                      cursor: busy ? 'default' : 'pointer', opacity: busy ? 0.5 : 1,
                    }}
                  >
                    {attached ? 'Stop stack' : `Stop, then start ${name}`}
                  </button>
                ) : (
                  <button
                    type="button"
                    disabled={busy}
                    onClick={() => {
                      ctx.setStartRobot(name);
                      ctx.setPop(null);
                      ctx.startStack(name);
                    }}
                    style={{
                      justifySelf: 'start', background: C.text, border: 0, color: C.ink,
                      font: `600 11px ${F.sans}`, padding: '3px 9px',
                      cursor: busy ? 'default' : 'pointer', opacity: busy ? 0.5 : 1,
                    }}
                  >
                    {`Start stack as ${name}`}
                  </button>
                )}
              </div>
            </div>
          );
        })}
      </div>
      <div style={{
        display: 'flex', gap: 10, alignItems: 'baseline', borderTop: `1px solid ${C.line}`,
        paddingTop: 10,
      }}
      >
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>world</span>
        <span style={{ font: `600 12px ${F.mono}` }}>{reference.world}</span>
        <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>only authored world</span>
      </div>
      {ctx.lifecycle ? (
        <div style={{
          font: `10.5px/1.45 ${F.mono}`, color: C.text2, background: C.sunken,
          border: `1px solid ${C.line}`, padding: '6px 8px', whiteSpace: 'pre-wrap',
        }}
        >
          {ctx.lifecycle.status === 'pending'
            ? `pending · ${ctx.lifecycle.service} · awaiting the gateway's answer`
            : (ctx.lifecycle.error
              ?? `response · ${ctx.lifecycle.response?.success ? 'accepted' : 'refused'} · `
                + `${ctx.lifecycle.response?.message ?? ''}`)}
        </div>
      ) : null}
    </div>
  );
}

function ProvPopover({ rows }: { rows: [string, string][] }) {
  return (
    <div style={{
      position: 'absolute', right: 150, top: 44, width: 560, background: C.dim,
      border: `1px solid ${C.lineBright}`, boxShadow: '0 10px 30px rgba(0,0,0,.55)',
      padding: 14, display: 'grid', gap: 8, whiteSpace: 'normal',
    }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'baseline' }}>
        <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
          PROVENANCE · live stack
        </span>
        <CopyButton label="Copy all" text={rows.map(([k, v]) => `${k}: ${v}`).join('\n')} />
      </div>
      <div style={{
        font: `10.5px ${F.sans}`, color: C.text3, borderBottom: `1px solid ${C.line}`,
        paddingBottom: 6,
      }}
      >
        A live mission writes no run directory, so there is no env.json for it. The
        committed provenance of a MEASURED run is on the Runs screen.
      </div>
      <div style={{
        display: 'grid', gridTemplateColumns: '140px minmax(0,1fr)', gap: '4px 10px',
        font: `11px ${F.mono}`,
      }}
      >
        {rows.map(([k, v]) => (
          <span key={k} style={{ display: 'contents' }}>
            <span style={{ color: C.text4 }}>{k}</span>
            <span style={{ wordBreak: 'break-all' }}>{v}</span>
          </span>
        ))}
      </div>
    </div>
  );
}

function LegendPopover() {
  return (
    <div style={{
      position: 'absolute', right: 66, top: 44, width: 360, background: C.dim,
      border: `1px solid ${C.lineBright}`, padding: '14px 16px', display: 'grid', gap: 10,
      boxShadow: '0 10px 30px rgba(0,0,0,.55)', whiteSpace: 'normal',
    }}
    >
      <div style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
        PROVENANCE
      </div>
      <div style={{
        display: 'grid', gridTemplateColumns: '18px 1fr', gap: '6px 8px',
        alignItems: 'baseline', font: `11.5px ${F.mono}`,
      }}
      >
        <span style={{ color: C.truth }}>■</span>
        <span>simulator truth · evaluation only</span>
        <span style={{ color: C.estimate }}>◆</span>
        <span>robot estimate</span>
        <span style={{ color: C.text }}>●</span>
        <span>measured</span>
        <span style={{ color: C.command }}>▶</span>
        <span>operator command</span>
        <span style={{ color: C.text4 }}>◇</span>
        <span>derived</span>
      </div>
      <div style={{
        font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
        borderTop: `1px solid ${C.line}`, paddingTop: 8,
      }}
      >
        FAULT EVENTS
      </div>
      <div style={{ display: 'flex', gap: 14, font: `11px ${F.mono}` }}>
        <span><span style={{ color: C.faultRequested }}>○</span> requested</span>
        <span><span style={{ color: C.fault }}>▲</span> activated</span>
        <span><span style={{ color: C.faultRejected }}>✕</span> rejected</span>
        <span><span style={{ color: C.faultCleared }}>■</span> cleared</span>
      </div>
      <div style={{ font: `11px ${F.sans}`, color: C.text4 }}>
        All robotics times are sim seconds.
      </div>
    </div>
  );
}

function KeysPopover() {
  const rows: [string, string][] = [
    ['1–4', 'fault sheet'],
    ['Q W E R', 'send armed slot'],
    ['P', 'place on map'],
    ['G', 'pick goal on map'],
    ['F', 'side drawer'],
    ['Esc', 'cancel, close'],
  ];
  return (
    <div style={{
      position: 'absolute', right: 12, top: 44, width: 280, background: C.dim,
      border: `1px solid ${C.lineBright}`, padding: '14px 16px', display: 'grid', gap: 8,
      boxShadow: '0 10px 30px rgba(0,0,0,.55)', whiteSpace: 'normal',
    }}
    >
      <div style={{
        display: 'grid', gridTemplateColumns: '70px 1fr', gap: '5px 10px', fontSize: 11.5,
      }}
      >
        {rows.map(([key, text]) => (
          <span key={key} style={{ display: 'contents' }}>
            <span style={{ font: `600 11px ${F.mono}` }}>{key}</span>
            <span>{text}</span>
          </span>
        ))}
      </div>
    </div>
  );
}
