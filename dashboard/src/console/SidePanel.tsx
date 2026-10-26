import { useEffect, useRef } from 'react';
import { C, F, STATES_A, STATES_B } from '../theme/tokens';
import { Chevron, CopyButton, Tip, Unavailable, f2, f3 } from '../ui/primitives';
import { mapToWorld, worldToMap } from '../client/store';
import { drawDivergence } from './draw';
import type { ConsoleCtx } from './context';

/**
 * The caption under an outcome: WHICH system reported it.
 *
 * Keyed on the outcome string itself, which is a closed set
 * (ral_eval.mission_driver.OUTCOMES). An outcome with no caption gets none -
 * the outcome is shown verbatim either way, and inventing a caption for an
 * unrecognised one would describe a mechanism nobody verified.
 */
const OUTCOME_CAPTION: Record<string, string> = {
  'nav2:STATUS_SUCCEEDED': 'reported by /navigate_to_pose/_action/status (GoalStatus 4)',
  'nav2:STATUS_CANCELED': 'reported by /navigate_to_pose/_action/status (GoalStatus 5)',
  'nav2:STATUS_ABORTED': 'reported by /navigate_to_pose/_action/status (GoalStatus 6)',
  'mavros:reached_wp_radius':
    'reported by /mavros/nav_controller_output/output wp_dist and /mavros/state, under '
    + 'the runner arrival rule',
  'mavros:stalled_no_progress':
    'runner policy: a deterministic no-progress rule in the mission driver, not a '
    + 'decision the autopilot made',
  'mavros:canceled_timeout': 'runner: timeout_sec reached and HOLD commanded',
  'runner:not_sent:goal_cell_occupied':
    'runner pre-flight refusal: the goal was never sent to the stack',
  'runner:not_sent:footprint_outside_map':
    'runner pre-flight refusal: the goal was never sent to the stack',
  'runner:not_sent:initial_pose_occupied':
    'runner pre-flight refusal: the spawn pose was refused, so no stack was started',
  'runner:unknown:clock_stalled': 'runner: /clock stopped advancing; no terminal arrived',
  'runner:unknown:no_terminal_status': 'runner: the stack produced no terminal status',
};

export function SidePanel({ ctx }: { ctx: ConsoleCtx }) {
  const { state, profile, isCar, reference } = ctx;
  const divRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    let frame = 0;
    const loop = () => {
      frame = window.requestAnimationFrame(loop);
      const canvas = divRef.current;
      // No reference line: the committed start-pose residual belongs to a
      // RUN, and this is a live mission with no metrics.json behind it.
      if (canvas) drawDivergence(canvas, state, null);
    };
    frame = window.requestAnimationFrame(loop);
    return () => window.cancelAnimationFrame(frame);
  });

  const mission = state.mission;
  const visited = new Set(state.missions.map((m) => m.state));
  const current = mission?.state ?? '';
  const outcome = mission?.outcome ?? '';
  const lastPair = state.pairs[state.pairs.length - 1];
  const activeList = state.active?.active ?? [];

  let errShort = '';
  let errReason = '';
  if (isCar && !state.align) {
    errShort = 'no /frame_alignment yet';
    errReason = '/frame_alignment not yet received. Pairing the EKF3 estimate with truth '
      + 'needs the measured world_to_ekf_local; an implicit identity transform is '
      + 'forbidden.';
  } else if (!state.estimate) {
    errShort = 'no estimate yet';
    errReason = `no estimate received yet on ${profile.estimate.topic}.`;
  } else if (!lastPair) {
    errShort = 'no pair yet';
    errReason = 'no estimate sample has paired with truth within 50 ms yet.';
  }

  const estimateAge = state.estimate
    ? Math.max(0, state.simNow - state.estimate.stamp_sim_s) : null;
  const cov = state.estimate?.covariance_trace ?? null;
  const convergedBelow = profile.estimate.uncertainty.converged_below;

  const goalWorld = mission
    ? mapToWorld(mission.goal.x, mission.goal.y, mission.goal.yaw_rad,
      reference.world_to_map)
    : null;
  const draftGoalMap = (() => {
    const x = Number(ctx.goal.x);
    const y = Number(ctx.goal.y);
    const yaw = Number(ctx.goal.yaw);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    return worldToMap(x, y, Number.isFinite(yaw) ? yaw : 0, reference.world_to_map);
  })();

  const stack = state.stack;
  const ready = stack?.state === 'ready';
  const missionActive = Boolean(stack?.mission_active);
  const missionBusy = ctx.lifecycle?.status === 'pending';
  const startDisabledReason = !ready
    ? `the stack is ${stack?.state ?? 'idle'}; a goal can only be sent to a stack that `
      + 'is ready'
    : missionActive
      ? 'a mission is already running'
      : draftGoalMap === null
        ? 'the goal must be two finite numbers in the gz world frame'
        : '';

  return (
    <div style={{
      width: 340, flex: 'none', overflowY: 'auto', background: C.side,
      borderRight: `1px solid ${C.line}`,
    }}
    >
      <button
        type="button"
        onClick={() => ctx.toggle('mission')}
        style={{
          all: 'unset', boxSizing: 'border-box', display: 'flex', alignItems: 'center',
          gap: 8, width: '100%', padding: '10px 14px 6px', cursor: 'pointer',
        }}
      >
        <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
          MISSION
        </span>
        <span style={{ font: `10px ${F.mono}`, color: C.text3 }}>●</span>
        <span style={{ flex: 1 }} />
        <Chevron open={ctx.open.mission} />
      </button>

      <div style={{
        padding: '0 14px 12px', borderBottom: `1px solid ${C.line}`, display: 'grid',
        gap: 8,
      }}
      >
        {!mission ? (
          <Unavailable
            short="no mission state published"
            reason="/mission_state carries one publisher: whichever process is driving a mission. Nothing here is inferred from the robot's motion."
          />
        ) : (
          <>
            <div style={{ display: 'flex', gap: 8, alignItems: 'baseline' }}>
              <span style={{ font: `600 17px ${F.mono}` }}>{current}</span>
              <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>
                {`${f2(mission.stamp_sim_s)} s`}
              </span>
            </div>
            {[STATES_A, STATES_B].map((row, index) => (
              <div
                key={index === 0 ? 'a' : 'b'}
                style={{
                  display: 'flex', gap: 3, flexWrap: 'wrap',
                  marginTop: index === 1 ? -4 : 0,
                }}
              >
                {row.map((name) => {
                  const isCurrent = name === current;
                  return (
                    <span
                      key={name}
                      style={{
                        font: `500 9.5px ${F.mono}`, padding: '1px 5px',
                        background: isCurrent ? C.text : 'transparent',
                        color: isCurrent ? C.ink : (visited.has(name) ? '#d6d6d6' : C.text6),
                        border: `1px solid ${isCurrent ? C.text
                          : (visited.has(name) ? C.borderHi : '#363636')}`,
                      }}
                    >
                      {name}
                    </span>
                  );
                })}
              </div>
            ))}
            {ctx.open.mission ? (
              <>
                <div style={{
                  display: 'grid', gridTemplateColumns: '64px minmax(0,1fr)',
                  gap: '5px 8px', alignItems: 'baseline',
                }}
                >
                  <span style={{ color: C.text4, fontSize: 11 }}>outcome</span>
                  <span style={{ display: 'grid', gap: 1, minWidth: 0 }}>
                    <span style={{ font: `600 12px ${F.mono}`, wordBreak: 'break-all' }}>
                      {outcome === '' ? "''" : outcome}
                    </span>
                    <span style={{ font: `10.5px ${F.sans}`, color: C.text4 }}>
                      {outcome === ''
                        ? 'empty until terminal'
                        : (OUTCOME_CAPTION[outcome] ?? '')}
                    </span>
                  </span>
                  <span style={{ color: C.text4, fontSize: 11 }}>goal</span>
                  <span style={{ font: `11px ${F.mono}` }}>
                    {`map (${f2(mission.goal.x)}, ${f2(mission.goal.y)}, `
                      + `${f2(mission.goal.yaw_rad)})`}
                    {goalWorld
                      ? ` · world (${f2(goalWorld.x)}, ${f2(goalWorld.y)})`
                      : ''}
                    <span style={{ color: C.command, fontSize: 10 }}> ▶</span>
                  </span>
                  <span style={{ color: C.text4, fontSize: 11 }}>
                    {isCar ? 'wp_dist' : 'remaining'}
                  </span>
                  <span style={{
                    font: `500 11.5px ${F.mono}`, display: 'flex', gap: 6,
                    alignItems: 'center',
                  }}
                  >
                    {isCar
                      ? (state.wp
                        ? `${state.wp.wp_dist} m · in [${state.wp.wp_dist}, `
                          + `${state.wp.wp_dist + 1}) ●`
                        : 'not reported')
                      : (state.feedback
                        ? `${f3(state.feedback.distance_remaining)} m ◆`
                        : 'no feedback')}
                    <Tip
                      text={isCar
                        ? '/mavros/nav_controller_output/output. wp_dist is uint16 METRES and truncates, so n means anywhere in [n, n+1) m.'
                        : "Nav2 feedback distance_remaining, computed by the robot from its own estimate. Never shown as a percentage."}
                    />
                  </span>
                  <span style={{ color: C.text4, fontSize: 11 }}>recoveries</span>
                  <span style={{
                    font: `11px ${F.mono}`, display: 'flex', gap: 6, alignItems: 'center',
                  }}
                  >
                    {isCar
                      ? 'unavailable · no Nav2'
                      : (state.feedback
                        ? `${state.feedback.number_of_recoveries} · Nav2 count`
                        : 'no feedback')}
                    <Tip
                      text={isCar
                        ? 'No Nav2 on this platform; ArduRover reports no recovery count.'
                        : "Nav2's own number_of_recoveries, verbatim. A recovery is never synthesised by the dashboard."}
                    />
                  </span>
                </div>
                {isCar && visited.has('goal_accepted') ? (
                  <div style={{
                    display: 'flex', gap: 6, alignItems: 'center',
                    font: `10.5px ${F.mono}`, color: C.text2, background: '#272727',
                    padding: '4px 7px',
                  }}
                  >
                    <span style={{ flex: 1 }}>
                      accepted = setpoint published (unacknowledged)
                    </span>
                    <Tip text="SET_POSITION_TARGET_LOCAL_NED is not acknowledged. Acceptance means published while GUIDED and armed." />
                  </div>
                ) : null}
                <div style={{ display: 'grid', gap: 3 }}>
                  <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
                    <span style={{ color: C.text4, fontSize: 11, flex: 1 }}>detail</span>
                    <button
                      type="button"
                      onClick={() => ctx.toggle('mdetail')}
                      style={{
                        background: 'transparent', border: `1px solid ${C.lineBright}`,
                        color: C.text2, font: `10.5px ${F.sans}`, padding: '1px 7px',
                        cursor: 'pointer',
                      }}
                    >
                      {ctx.open.mdetail ? 'Less' : 'Full'}
                    </button>
                    <CopyButton text={mission.detail} />
                  </div>
                  <div style={{
                    font: `10.5px/1.45 ${F.mono}`, color: C.textBright, background: C.sunken,
                    border: `1px solid ${C.line}`, padding: '6px 8px',
                    maxHeight: ctx.open.mdetail ? 'none' : '4.4em', overflow: 'auto',
                    whiteSpace: 'pre-wrap', wordBreak: 'break-word',
                  }}
                  >
                    {mission.detail}
                  </div>
                </div>
              </>
            ) : null}
          </>
        )}
        {state.tTerminal !== null ? (
          <div style={{
            font: `11px ${F.sans}`, color: C.text, background: C.rowAlt, padding: '6px 8px',
          }}
          >
            {`Terminal at sim ${f2(state.tTerminal)}. A dashboard mission writes no run `
              + 'directory; the Runs screen carries run_scenario output only.'}
          </div>
        ) : null}

        <button
          type="button"
          onClick={() => ctx.toggle('setup')}
          style={{
            all: 'unset', boxSizing: 'border-box', display: 'flex', alignItems: 'center',
            gap: 6, cursor: 'pointer',
          }}
        >
          <Chevron open={ctx.open.setup} />
          <span style={{ font: `500 11px ${F.sans}`, color: C.text2 }}>
            Setup &amp; pre-flight
          </span>
        </button>
        {ctx.open.setup ? (
          <div style={{
            display: 'grid', gap: 7, background: C.sunkenAlt, border: '1px solid #2e2e2e',
            padding: '8px 9px',
          }}
          >
            <div style={{
              display: 'grid', gridTemplateColumns: '44px 1fr 1fr', gap: '3px 8px',
              font: `10.5px ${F.mono}`, alignItems: 'baseline',
            }}
            >
              <span />
              <span style={{ color: C.text4 }}>world</span>
              <span style={{ color: C.text4 }}>map</span>
              <span style={{ color: C.text4, fontFamily: F.sans }}>spawn</span>
              <span>
                {stack?.spawn
                  ? `(${f2(stack.spawn.x)}, ${f2(stack.spawn.y)}, ${f2(stack.spawn.yaw)})`
                  : `(${profile.spawn.x}, ${profile.spawn.y}, ${profile.spawn.yaw}) default`}
              </span>
              <span>
                {(() => {
                  const s = stack?.spawn ?? {
                    x: profile.spawn.x, y: profile.spawn.y, yaw: profile.spawn.yaw,
                  };
                  const m = worldToMap(s.x, s.y, s.yaw, reference.world_to_map);
                  return `(${f2(m.x)}, ${f2(m.y)}, ${f2(m.yaw)})`;
                })()}
              </span>
              <span style={{ color: C.text4, fontFamily: F.sans }}>goal</span>
              <span>{`(${ctx.goal.x || '—'}, ${ctx.goal.y || '—'}, ${ctx.goal.yaw})`}</span>
              <span>
                {draftGoalMap
                  ? `(${f2(draftGoalMap.x)}, ${f2(draftGoalMap.y)}, ${f2(draftGoalMap.yaw)})`
                  : '—'}
              </span>
            </div>
            <div style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>
              {`world_to_map (${reference.world_to_map.x}, ${reference.world_to_map.y}, `
                + `${reference.world_to_map.yaw}) · footprint ${profile.footprint.radius_m} m `
                + `· ${profile.footprint.source}`}
            </div>

            <div style={{
              display: 'grid', gridTemplateColumns: '1fr 1fr 1fr 1fr', gap: 6,
              alignItems: 'end',
            }}
            >
              {([['x', 'goal x · world m'], ['y', 'goal y · world m'],
                ['yaw', 'goal yaw · rad'], ['timeout', 'timeout_sec']] as const)
                .map(([key, label]) => (
                  <label key={key} style={{ display: 'grid', gap: 2, minWidth: 0 }}>
                    <span style={{
                      font: `10px ${F.mono}`, color: C.text4, overflow: 'hidden',
                      textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                    }}
                    >
                      {label}
                    </span>
                    <input
                      value={ctx.goal[key]}
                      onChange={(event) => {
                        const { value } = event.target;
                        ctx.setGoal((g) => ({ ...g, [key]: value }));
                      }}
                      style={{
                        background: C.ink, border: `1px solid ${C.lineMid}`, color: C.text,
                        fontSize: 11.5, padding: '4px 5px', width: '100%',
                      }}
                    />
                  </label>
                ))}
            </div>

            <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', alignItems: 'center' }}>
              <button
                type="button"
                onClick={() => ctx.setPick(ctx.pick === 'goal' ? null : 'goal')}
                style={{
                  background: ctx.pick === 'goal' ? C.command : 'transparent',
                  border: `1px dashed ${C.command}`,
                  color: ctx.pick === 'goal' ? C.ink : C.draftText,
                  font: `500 11px ${F.sans}`, padding: '3px 9px', cursor: 'pointer',
                }}
              >
                {ctx.pick === 'goal' ? 'Picking… click the map' : 'Pick on map (G)'}
              </button>
              <button
                type="button"
                disabled={Boolean(startDisabledReason) || missionBusy}
                data-tip={startDisabledReason || undefined}
                onClick={ctx.startMission}
                style={{
                  background: startDisabledReason ? 'transparent' : C.text,
                  border: `1px solid ${startDisabledReason ? C.lineMid : C.text}`,
                  color: startDisabledReason ? C.text5 : C.ink,
                  font: `600 11px ${F.sans}`, padding: '3px 9px',
                  cursor: startDisabledReason ? 'help' : 'pointer',
                }}
              >
                {missionBusy ? 'Pending…' : 'Start mission'}
              </button>
              <button
                type="button"
                disabled={!missionActive}
                onClick={ctx.cancelMission}
                style={{
                  background: 'transparent',
                  border: `1px solid ${missionActive ? C.border : C.lineMid}`,
                  color: missionActive ? C.text : C.text5,
                  font: `500 11px ${F.sans}`, padding: '3px 9px',
                  cursor: missionActive ? 'pointer' : 'default',
                }}
              >
                Cancel
              </button>
              {stack?.state === 'idle' || !stack ? (
                <button
                  type="button"
                  disabled={missionBusy}
                  onClick={() => ctx.startStack(ctx.startRobot)}
                  style={{
                    background: C.text, border: 0, color: C.ink,
                    font: `600 11px ${F.sans}`, padding: '3px 9px', cursor: 'pointer',
                  }}
                >
                  {`Start stack (${ctx.startRobot})`}
                </button>
              ) : (
                <button
                  type="button"
                  disabled={missionBusy || missionActive}
                  onClick={ctx.stopStack}
                  style={{
                    background: 'transparent', border: `1px solid ${C.border}`,
                    color: C.text, font: `500 11px ${F.sans}`, padding: '3px 9px',
                    cursor: missionActive ? 'default' : 'pointer',
                  }}
                >
                  Stop stack
                </button>
              )}
            </div>

            {startDisabledReason ? (
              <div style={{ font: `10.5px ${F.sans}`, color: C.text4 }}>
                {startDisabledReason}
              </div>
            ) : null}

            {ctx.lifecycle ? (
              <div style={{
                font: `10.5px/1.45 ${F.mono}`, color: C.text2, background: C.sunken,
                border: `1px solid ${C.line}`, padding: '6px 8px', whiteSpace: 'pre-wrap',
              }}
              >
                {ctx.lifecycle.status === 'pending'
                  ? `pending · ${ctx.lifecycle.service} · awaiting the gateway's answer`
                  : (ctx.lifecycle.error
                    ?? `${ctx.lifecycle.response?.success ? 'accepted' : 'refused'} · `
                      + `${ctx.lifecycle.response?.message ?? ''}`)}
              </div>
            ) : null}

            {stack?.state === 'failed' ? (
              <div style={{
                font: `10.5px/1.45 ${F.mono}`, color: C.warn,
                border: `1px solid ${C.warnBorder}`, padding: '6px 8px',
                whiteSpace: 'pre-wrap',
              }}
              >
                {stack.detail}
              </div>
            ) : null}

            {state.missions.some((m) => m.state === 'unreachable') ? (
              <div style={{
                font: `10.5px/1.45 ${F.mono}`, color: C.bad,
                border: `1px solid ${C.faultRejected}`, padding: '6px 8px',
                whiteSpace: 'pre-wrap',
              }}
              >
                {state.missions.filter((m) => m.state === 'unreachable').slice(-1)[0].detail}
              </div>
            ) : null}
          </div>
        ) : null}
      </div>

      <button
        type="button"
        onClick={() => ctx.toggle('div')}
        style={{
          all: 'unset', boxSizing: 'border-box', display: 'flex', alignItems: 'center',
          gap: 8, width: '100%', padding: '10px 14px 6px', cursor: 'pointer',
        }}
      >
        <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
          ESTIMATE VS TRUTH
        </span>
        <span style={{ font: `10px ${F.mono}`, color: C.text3 }}>◇ derived (live)</span>
        <span style={{ flex: 1 }} />
        <Chevron open={ctx.open.div} />
      </button>
      <div style={{
        padding: '0 14px 12px', borderBottom: `1px solid ${C.line}`, display: 'grid',
        gap: 7,
      }}
      >
        {errReason ? (
          <Unavailable short={`unavailable · ${errShort}`} reason={errReason} />
        ) : (
          <div style={{ display: 'grid', gap: 2 }}>
            <div style={{ display: 'flex', alignItems: 'baseline', gap: 7 }}>
              <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>|est − truth|</span>
              <span style={{ font: `600 18px ${F.mono}` }}>{f3(lastPair!.e)}</span>
              <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>m</span>
            </div>
            {/* The residual is as old as the newest PAIR, and an estimate that
                is motion-gated can leave that pair far behind `now`. Stamping
                it is the difference between "the robot is 0.095 m out" and
                "the robot was 0.095 m out, half a minute ago". */}
            <span style={{
              font: `10px ${F.mono}`,
              color: state.simNow - lastPair!.t > 1.0 ? C.warn : C.text4,
            }}
            >
              {state.simNow - lastPair!.t > 1.0
                ? `measured at sim ${f2(lastPair!.t)} s · `
                  + `${f2(state.simNow - lastPair!.t)} s ago, not now`
                : `measured at sim ${f2(lastPair!.t)} s`}
            </span>
          </div>
        )}
        {ctx.open.div ? (
          <>
            <div style={{
              position: 'relative', height: 92, background: C.sunken,
              border: `1px solid ${C.line}`,
            }}
            >
              <canvas
                ref={divRef}
                style={{
                  position: 'absolute', inset: 0, width: '100%', height: '100%',
                  display: 'block',
                }}
              />
            </div>
            <div style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
              <span style={{ font: `10px ${F.mono}`, color: C.text4, flex: 1 }}>
                {`${state.pairs.length} pairs · ${state.unpaired} unpaired dropped`}
                {state.droppedFrames
                  ? ` · ${state.droppedFrames} transport frames dropped`
                  : ''}
              </span>
              <Tip text="Paired by header stamp within 50 ms. Unpaired samples are dropped, never extrapolated. The axis is never clipped. Shaded: the mission window." />
            </div>
          </>
        ) : null}
        <div style={{
          display: 'grid', gridTemplateColumns: '78px minmax(0,1fr)', gap: '4px 8px',
          alignItems: 'center',
        }}
        >
          <span style={{ color: C.text4, fontSize: 11 }}>estimate age</span>
          <span style={{ display: 'flex', gap: 6, alignItems: 'center' }}>
            <span style={{ font: `500 11.5px ${F.mono}` }}>
              {estimateAge === null ? 'none yet' : `${f2(estimateAge)} s`}
            </span>
            <Tip
              text={isCar
                ? 'Sim seconds since the last /mavros/local_position/odom. The stream can stall and then flush. A long age is not a fault.'
                : 'Sim seconds since the last /amcl_pose. AMCL is motion-gated (update_min_d 0.1 m, update_min_a 0.2 rad): a stationary robot publishes nothing. A long age is not a fault.'}
            />
          </span>
          <span style={{ color: C.text4, fontSize: 11 }}>uncertainty</span>
          <span>
            {profile.estimate.uncertainty.kind === 'covariance_trace' ? (
              <span style={{ font: `500 11.5px ${F.mono}` }}>
                {cov === null ? '—' : f3(cov)}
                <span style={{ fontSize: 10, color: C.text4 }}>
                  {cov === null
                    ? ` no ${profile.estimate.topic} yet`
                    : (convergedBelow !== undefined
                      ? (cov < convergedBelow
                        ? ` < ${convergedBelow} converged` : ` ≥ ${convergedBelow}`)
                      : '')}
                </span>
                <span style={{ fontSize: 10, color: C.estimate }}> ◆</span>
              </span>
            ) : (
              <Unavailable
                short="unavailable"
                reason={`${profile.robot}.yaml declares uncertainty kind `
                  + `${profile.estimate.uncertainty.kind} on `
                  + `${profile.estimate.uncertainty.topic ?? 'no topic'}: ArduPilot sends `
                  + 'no LOCAL_POSITION_NED_COV and refused SET_MESSAGE_INTERVAL for '
                  + 'ESTIMATOR_STATUS, so this vehicle publishes no numeric pose '
                  + 'uncertainty at all.'}
              />
            )}
          </span>
        </div>
      </div>

      <div style={{ display: 'flex', alignItems: 'center', gap: 8, padding: '10px 14px 6px' }}>
        <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4 }}>
          ACTIVE FAULTS
        </span>
        <span style={{ font: `600 12px ${F.mono}` }}>
          {state.active ? activeList.length : '—'}
        </span>
      </div>
      <div style={{ padding: '0 14px 14px', display: 'grid', gap: 6 }}>
        {!state.active ? (
          <div style={{
            border: `1px dashed ${C.border}`, padding: '6px 9px',
            font: `500 10.5px ${F.mono}`, color: C.text,
          }}
          >
            no /faults/active received: the fault manager is not running, or no stack is up
          </div>
        ) : null}
        {state.active && activeList.length === 0 ? (
          <div style={{ font: `11.5px ${F.sans}`, color: C.text4 }}>No faults in force.</div>
        ) : null}
        {activeList.map((fault) => {
          const entry = reference.catalogue.faults[fault.kind];
          const lifetime = entry?.lifetime ?? 'singleton';
          const key = `row:${fault.id}`;
          const card = ctx.cardFor(key);
          const open = Boolean(ctx.open.rows[fault.id]);
          return (
            <div
              key={fault.id}
              style={{ background: C.row, border: `1px solid ${C.lineSoft}`, display: 'grid' }}
            >
              <div style={{
                display: 'flex', alignItems: 'center', gap: 7, padding: '6px 8px',
              }}
              >
                <button
                  type="button"
                  onClick={() => ctx.toggleRow(fault.id)}
                  style={{
                    all: 'unset', display: 'flex', alignItems: 'baseline', gap: 7, flex: 1,
                    minWidth: 0, cursor: 'pointer',
                  }}
                >
                  <span style={{ color: C.fault }}>▲</span>
                  <span style={{ display: 'grid', minWidth: 0 }}>
                    <span style={{ font: `600 11.5px ${F.mono}` }}>{fault.kind}</span>
                    <span style={{
                      font: `10px ${F.mono}`, color: C.text4, overflow: 'hidden',
                      textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                    }}
                    >
                      {`${fault.target} · ${f2(fault.stamp_sim_s)} s · `
                        + `${entry?.alters_truth ? 'changes world' : 'world unchanged'}`}
                    </span>
                  </span>
                </button>
                {lifetime !== 'oneshot' ? (
                  <button
                    type="button"
                    disabled={Boolean(ctx.blockReason) || ctx.pending(key)}
                    onClick={() => {
                      if (lifetime === 'keyed_set') {
                        ctx.sendFault(key, 'clear_obstacle', { name: fault.target },
                          `clear '${fault.target}'`);
                      } else {
                        ctx.sendFault(key, fault.kind, { enabled: false }, 'enabled: false');
                      }
                    }}
                    style={{
                      background: 'transparent', border: `1px solid ${C.borderAlt}`,
                      color: C.text, font: `500 10.5px ${F.sans}`, padding: '2px 8px',
                      cursor: 'pointer', whiteSpace: 'nowrap',
                      opacity: ctx.blockReason || ctx.pending(key) ? 0.5 : 1,
                    }}
                  >
                    {ctx.pending(key)
                      ? 'pending…'
                      : (lifetime === 'keyed_set' ? 'Clear' : 'Disable')}
                  </button>
                ) : null}
                <Chevron open={open} />
              </div>
              {card ? (
                <div style={{
                  font: `10px ${F.mono}`, padding: '0 8px 6px',
                  color: card.status === 'pending'
                    ? C.text
                    : (card.response?.success ? C.text : C.bad),
                }}
                >
                  {card.status === 'pending'
                    ? `pending · sent on ${card.service} at sim ${f2(card.sentSim)} s`
                    : (card.error ?? card.response?.message ?? '')}
                </div>
              ) : null}
              {open ? (
                <div style={{
                  display: 'grid', gap: 6, padding: '6px 8px 8px',
                  borderTop: `1px solid ${C.line}`,
                }}
                >
                  <span style={{
                    font: `10px ${F.mono}`, color: C.text4, wordBreak: 'break-all',
                  }}
                  >
                    {`${fault.id} · ${fault.injection_point}`}
                  </span>
                  <span style={{
                    font: `10.5px/1.45 ${F.mono}`, color: '#d6d6d6', maxHeight: 160,
                    overflow: 'auto', whiteSpace: 'pre-wrap', wordBreak: 'break-word',
                  }}
                  >
                    {fault.detail}
                  </span>
                  {entry?.honesty_note ? (
                    <span style={{
                      font: `10.5px/1.45 ${F.sans}`, color: C.text3, fontStyle: 'italic',
                    }}
                    >
                      {entry.honesty_note}
                    </span>
                  ) : null}
                </div>
              ) : null}
            </div>
          );
        })}
        <div style={{ display: 'flex', gap: 8, alignItems: 'center', paddingTop: 4 }}>
          <button
            type="button"
            disabled={Boolean(ctx.blockReason) || ctx.pending('clearall')}
            onClick={() => ctx.twoStep('all',
              () => ctx.sendFault('clearall', 'clear_all', {}, 'clear all'))}
            style={{
              background: ctx.confirm === 'all' ? C.text : 'transparent',
              border: `1px solid ${C.borderAlt}`,
              color: ctx.confirm === 'all' ? C.ink : C.text,
              font: `500 11px ${F.sans}`, padding: '3px 9px', cursor: 'pointer',
              opacity: ctx.blockReason ? 0.5 : 1,
            }}
          >
            {ctx.pending('clearall')
              ? 'Clear all · pending'
              : (ctx.confirm === 'all' ? 'Confirm: clear all' : 'Clear all')}
          </button>
          <span style={{
            font: `10px ${F.mono}`, color: C.text3, overflow: 'hidden',
            textOverflow: 'ellipsis', whiteSpace: 'nowrap', minWidth: 0, flex: 1,
          }}
          >
            {ctx.cardFor('clearall')?.response?.message
              ?? ctx.cardFor('clearall')?.error ?? ''}
          </span>
        </div>
      </div>
    </div>
  );
}
