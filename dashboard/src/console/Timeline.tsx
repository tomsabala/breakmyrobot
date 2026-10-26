import { C, EVENT_MARK, F, TERMINAL_STATES } from '../theme/tokens';
import { Chevron, CopyButton, f2 } from '../ui/primitives';
import type { ConsoleCtx } from './context';

/**
 * The mission and fault timeline, in sim seconds.
 *
 * Every marker is ONE message that arrived. Nothing is interpolated between
 * markers, no state is inferred for an interval, and a stream gap is hatched
 * rather than smoothed: the lane shows what was received, not what probably
 * happened.
 */
export function Timeline({ ctx }: { ctx: ConsoleCtx }) {
  const { state, reference } = ctx;
  const replay = ctx.source.replay;

  const horizon = Math.max(60, state.simNow + 5,
    (state.tTerminal ?? 0) + 5);
  const pct = (t: number) => Math.max(0, Math.min(100, (t / horizon) * 100));
  const relative = ctx.timeMode === 'rel' && state.tAccept !== null;
  const step = horizon > 240 ? 60 : horizon > 120 ? 20 : horizon > 60 ? 10 : 5;
  const ticks: { left: number; label: string }[] = [];
  for (let t = 0; t <= horizon; t += step) {
    const shown = relative ? t - state.tAccept! : t;
    ticks.push({
      left: pct(t),
      label: (relative && shown > 0 ? '+' : '') + shown.toFixed(relative ? 1 : 0),
    });
  }

  const missionMarks = state.missions.map((m, index) => ({
    left: pct(m.stamp_sim_s),
    label: m.state,
    terminal: TERMINAL_STATES.has(m.state),
    index,
  }));
  for (let i = 1; i < missionMarks.length; i += 1) {
    if (missionMarks[i].left - missionMarks[i - 1].left < 9) {
      missionMarks[i].left = missionMarks[i - 1].left + 9;
    }
  }

  const byId = new Map<string, { event: typeof state.events[number]; index: number }[]>();
  state.events.forEach((event, index) => {
    const list = byId.get(event.id) ?? [];
    list.push({ event, index });
    byId.set(event.id, list);
  });
  const lanes = [...byId.entries()].map(([id, entries]) => {
    const first = entries[0].event.stamp_sim_s;
    const last = entries[entries.length - 1].event.stamp_sim_s;
    const lifetime = reference.catalogue.faults[entries[0].event.kind]?.lifetime;
    const closed = entries.some((e) => e.event.state === 'cleared'
      || e.event.state === 'rejected') || lifetime === 'oneshot';
    return {
      id,
      left: pct(first),
      width: Math.max(0, pct(closed ? last : state.simNow) - pct(first)),
      marks: entries.map(({ event, index }) => ({
        left: pct(event.stamp_sim_s),
        shape: EVENT_MARK[event.state]?.[0] ?? '·',
        colour: EVENT_MARK[event.state]?.[1] ?? C.text4,
        index,
      })),
    };
  });

  const selected = (() => {
    const marker = ctx.selectedMarker;
    if (!marker) return null;
    if (marker.type === 'm') {
      const m = state.missions[marker.index];
      if (!m) return null;
      return {
        head: `/mission_state · ${m.state}`,
        meta: `sim ${f2(m.stamp_sim_s)} s · outcome ${m.outcome === '' ? "''" : m.outcome}`
          + ` · ${m.experiment_id}`,
        detail: m.detail,
        honesty: '',
      };
    }
    const e = state.events[marker.index];
    if (!e) return null;
    return {
      head: `${e.id} · ${e.state}`,
      meta: `sim ${f2(e.stamp_sim_s)} s · ${e.kind} · injection_point ${e.injection_point}`
        + ` · target ${e.target}`,
      detail: e.detail,
      honesty: reference.catalogue.faults[e.kind]?.honesty_note ?? '',
    };
  })();

  const status = state.tTerminal !== null
    ? `mission ended at sim ${f2(state.tTerminal)}`
    : state.tAccept !== null
      ? `mission running since sim ${f2(state.tAccept)}`
      : 'live · no mission window yet';

  return (
    <div style={{
      borderTop: `1px solid ${C.line}`, background: C.panel, display: 'flex',
      flexDirection: 'column',
    }}
    >
      <div style={{
        display: 'flex', alignItems: 'center', gap: 10, padding: '6px 12px',
        borderBottom: `1px solid ${C.header}`,
      }}
      >
        <button
          type="button"
          onClick={() => ctx.toggle('timeline')}
          style={{
            all: 'unset', display: 'flex', gap: 6, alignItems: 'center', cursor: 'pointer',
          }}
        >
          <Chevron open={ctx.open.timeline} />
          <span style={{
            font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
          }}
          >
            TIMELINE
          </span>
        </button>
        <div style={{ display: 'flex', border: `1px solid ${C.lineMid}` }}>
          {(['abs', 'rel'] as const).map((mode) => (
            <button
              key={mode}
              type="button"
              onClick={() => ctx.setTimeMode(mode)}
              style={{
                background: ctx.timeMode === mode ? C.text : 'transparent',
                color: ctx.timeMode === mode ? C.ink : C.text2, border: 0,
                font: `500 10.5px ${F.mono}`, padding: '3px 8px', cursor: 'pointer',
              }}
            >
              {mode === 'abs' ? 'sim s' : 'since accept'}
            </button>
          ))}
        </div>
        <span style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>{status}</span>
        <div style={{ flex: 1 }} />
        {replay ? <ReplayBar ctx={ctx} /> : null}
      </div>

      {ctx.open.timeline ? (
        <div style={{ display: 'flex', minHeight: 0 }}>
          <div style={{
            flex: 1, minWidth: 0, position: 'relative', padding: '4px 0 8px',
            maxHeight: 180, overflowY: 'auto',
          }}
          >
            <div style={{
              position: 'absolute', top: 0, bottom: 0, left: 160, right: 14,
              pointerEvents: 'none',
            }}
            >
              {state.tAccept !== null ? (
                <div style={{
                  position: 'absolute', top: 0, bottom: 0, left: `${pct(state.tAccept)}%`,
                  width: `${pct(state.tTerminal ?? state.simNow) - pct(state.tAccept)}%`,
                  background: 'rgba(255,255,255,.055)',
                }}
                />
              ) : null}
              {state.gaps.map((gap) => (
                <div
                  key={`${gap.from}-${gap.to}`}
                  style={{
                    position: 'absolute', top: 0, bottom: 0, left: `${pct(gap.from)}%`,
                    width: `${pct(gap.to) - pct(gap.from)}%`,
                    background: 'repeating-linear-gradient(45deg,rgba(255,255,255,.08) 0 4px,'
                      + 'transparent 4px 8px)',
                  }}
                />
              ))}
              <div style={{
                position: 'absolute', top: 0, bottom: 0, left: `${pct(state.simNow)}%`,
                borderLeft: `1px solid ${C.text}`,
              }}
              />
            </div>

            <div style={{ display: 'flex', height: 18, alignItems: 'flex-end' }}>
              <span style={{
                width: 160, paddingLeft: 12, font: `10px ${F.mono}`, color: C.text4,
              }}
              >
                {relative ? 's since goal_accepted' : 'sim s'}
              </span>
              <div style={{
                position: 'relative', flex: 1, marginRight: 14, height: '100%',
                borderBottom: `1px solid ${C.lineSoft}`,
              }}
              >
                {ticks.map((tick) => (
                  <span
                    key={tick.label + tick.left}
                    style={{
                      position: 'absolute', left: `${tick.left}%`, bottom: 2,
                      transform: 'translateX(-50%)', font: `10px ${F.mono}`,
                      color: C.text4,
                    }}
                  >
                    {tick.label}
                  </span>
                ))}
              </div>
            </div>

            <div style={{ display: 'flex', height: 24, alignItems: 'center' }}>
              <span style={{
                width: 160, paddingLeft: 12, font: `500 10.5px ${F.mono}`,
                color: C.textBright,
              }}
              >
                mission
              </span>
              <div style={{ position: 'relative', flex: 1, marginRight: 14, height: '100%' }}>
                {missionMarks.map((mark) => (
                  <button
                    key={`${mark.label}-${mark.index}`}
                    type="button"
                    onClick={() => ctx.setSelectedMarker({ type: 'm', index: mark.index })}
                    style={{
                      position: 'absolute', left: `${mark.left}%`, top: 3,
                      transform: 'translateX(-4px)',
                      background: mark.terminal ? C.lineSoft : C.row,
                      border: `1px solid ${C.borderHi}`, color: C.text,
                      font: `500 10px ${F.mono}`, padding: '1px 5px', cursor: 'pointer',
                      whiteSpace: 'nowrap',
                    }}
                  >
                    {mark.label}
                  </button>
                ))}
              </div>
            </div>

            {!ctx.isCar ? (
              <div style={{ display: 'flex', height: 16, alignItems: 'center' }}>
                <span style={{
                  width: 160, paddingLeft: 12, font: `10px ${F.mono}`, color: C.text4,
                }}
                >
                  /plan publications
                </span>
                <div style={{
                  position: 'relative', flex: 1, marginRight: 14, height: '100%',
                }}
                >
                  {state.planTimes.map((t) => (
                    <span
                      key={t}
                      style={{
                        position: 'absolute', left: `${pct(t)}%`, top: 3, height: 10,
                        borderLeft: `1px solid ${C.plan}`, opacity: 0.7,
                      }}
                    />
                  ))}
                </div>
              </div>
            ) : null}

            {lanes.length === 0 ? (
              <div style={{ display: 'flex', height: 20, alignItems: 'center' }}>
                <span style={{
                  width: 160, paddingLeft: 12, font: `10px ${F.mono}`, color: C.text4,
                }}
                >
                  faults
                </span>
                <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>none</span>
              </div>
            ) : null}

            {lanes.map((lane) => (
              <div key={lane.id} style={{ display: 'flex', height: 22, alignItems: 'center' }}>
                <span style={{
                  width: 160, paddingLeft: 12, font: `10px ${F.mono}`, color: C.textBright,
                  overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                }}
                >
                  {lane.id}
                </span>
                <div style={{
                  position: 'relative', flex: 1, marginRight: 14, height: '100%',
                }}
                >
                  <div style={{
                    position: 'absolute', top: '50%', left: `${lane.left}%`,
                    width: `${lane.width}%`, borderTop: `1px dotted ${C.borderAlt}`,
                  }}
                  />
                  {lane.marks.map((mark) => (
                    <button
                      key={mark.index}
                      type="button"
                      onClick={() => ctx.setSelectedMarker({ type: 'e', index: mark.index })}
                      style={{
                        position: 'absolute', left: `${mark.left}%`, top: 2,
                        transform: 'translateX(-50%)', background: 'transparent',
                        border: 0, color: mark.colour, font: `14px/18px ${F.mono}`,
                        padding: '0 2px', cursor: 'pointer',
                      }}
                    >
                      {mark.shape}
                    </button>
                  ))}
                </div>
              </div>
            ))}
          </div>

          <div style={{
            width: 380, flex: 'none', borderLeft: `1px solid ${C.header}`,
            padding: '8px 12px', display: 'grid', gap: 4, alignContent: 'start',
            maxHeight: 180, overflow: 'auto',
          }}
          >
            {!selected ? (
              <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>
                Click a marker for its detail.
              </span>
            ) : (
              <>
                <div style={{ display: 'flex', gap: 6, alignItems: 'baseline' }}>
                  <span style={{
                    font: `600 11.5px ${F.mono}`, flex: 1, wordBreak: 'break-all',
                  }}
                  >
                    {selected.head}
                  </span>
                  <CopyButton
                    text={`${selected.head}\n${selected.meta}\n${selected.detail}`}
                  />
                </div>
                <span style={{
                  font: `10px ${F.mono}`, color: C.text4, wordBreak: 'break-word',
                }}
                >
                  {selected.meta}
                </span>
                <span style={{
                  font: `10.5px/1.45 ${F.mono}`, color: C.textBright,
                  whiteSpace: 'pre-wrap', wordBreak: 'break-word',
                }}
                >
                  {selected.detail}
                </span>
                {selected.honesty ? (
                  <span style={{
                    font: `10.5px ${F.sans}`, color: C.text3, fontStyle: 'italic',
                  }}
                  >
                    {selected.honesty}
                  </span>
                ) : null}
              </>
            )}
          </div>
        </div>
      ) : null}
    </div>
  );
}

/**
 * The replay bar: MOCK ONLY.
 *
 * A live stack has no replay. Showing Play/Pause/step against one would imply
 * the operator can rewind a running simulation, which is a control the system
 * does not have.
 */
function ReplayBar({ ctx }: { ctx: ConsoleCtx }) {
  const replay = ctx.source.replay!;
  return (
    <div style={{
      display: 'flex', alignItems: 'center', gap: 6, border: `1px dashed ${C.border}`,
      padding: '3px 6px',
    }}
    >
      <span style={{
        font: `600 10px ${F.mono}`, letterSpacing: '.06em', color: C.text2,
      }}
      >
        REPLAY
      </span>
      <select
        value={replay.scenario()}
        onChange={(event) => replay.pick(event.target.value)}
        style={{
          background: C.ink, color: C.text, border: `1px solid ${C.lineMid}`,
          fontSize: 10.5, padding: '2px 4px',
        }}
      >
        {replay.scenarios().map((option) => (
          <option key={option.value} value={option.value} disabled={option.disabled}>
            {option.label}
          </option>
        ))}
      </select>
      <button type="button" onClick={replay.restart} style={smallButton}>Restart</button>
      <button type="button" onClick={replay.seekAccept} style={smallButton}>→ accept</button>
      <button
        type="button"
        onClick={replay.togglePlay}
        style={{
          background: C.text, border: 0, color: C.ink, font: `600 10.5px ${F.sans}`,
          padding: '3px 9px', cursor: 'pointer', minWidth: 54,
          opacity: replay.closed() ? 0.5 : 1,
        }}
      >
        {replay.closed() ? 'Ended' : (replay.playing() ? 'Pause' : 'Play')}
      </button>
      <button type="button" onClick={() => replay.step(0.1)} style={smallButton}>+0.1 s</button>
      <div style={{ display: 'flex', border: `1px solid ${C.lineMid}` }}>
        {[1, 4, 16].map((speed) => (
          <button
            key={speed}
            type="button"
            onClick={() => replay.setSpeed(speed)}
            style={{
              background: replay.speed() === speed ? C.text : 'transparent',
              color: replay.speed() === speed ? C.ink : C.text2, border: 0,
              font: `500 10px ${F.mono}`, padding: '2px 6px', cursor: 'pointer',
            }}
          >
            {`${speed}×`}
          </button>
        ))}
      </div>
    </div>
  );
}

const smallButton = {
  background: 'transparent', border: `1px solid ${C.lineBright}`, color: C.text,
  font: `500 10.5px ${F.sans}`, padding: '2px 7px', cursor: 'pointer',
} as const;
