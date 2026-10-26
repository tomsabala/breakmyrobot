import { useCallback, useEffect, useRef, useState } from 'react';
import { C, F } from '../theme/tokens';
import { Tip, f2 } from '../ui/primitives';
import { mapToWorld, worldToMap } from '../client/store';
import { drawMap, toMap } from './draw';
import type { DraftBox, DraftPose, Layers, View } from './draw';
import { num } from './faults';
import type { ConsoleCtx } from './context';

const LAYER_ORDER: (keyof Layers)[] = ['map', 'world', 'gcost', 'lcost', 'truth', 'est',
  'plan', 'scan', 'raw', 'foot'];

export function WorldView({ ctx }: { ctx: ConsoleCtx }) {
  const { state, reference, profile, isCar } = ctx;
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const viewRef = useRef<View | null>(null);
  const dragRef = useRef<{ kind: 'obs' | 'kid' | 'goal'; a: { x: number; y: number } } | null>(null);
  const [, setDrawTick] = useState(0);

  const draftBox: DraftBox | null = (() => {
    if (ctx.sheet !== 'obs' || !(ctx.drawerOpen || ctx.pick === 'obs')) return null;
    const f = ctx.forms.obs;
    const x = num(f.x);
    const y = num(f.y);
    const sx = num(f.sx);
    const sy = num(f.sy);
    if (x === null || y === null || sx === null || sy === null) return null;
    return { name: f.name, x, y, sx, sy };
  })();

  const draftKidnap: DraftPose | null = (() => {
    if (ctx.sheet !== 'kid' || !(ctx.drawerOpen || ctx.pick === 'kid')) return null;
    const f = ctx.forms.kid;
    const x = num(f.x);
    const y = num(f.y);
    const yaw = num(f.yaw);
    if (x === null || y === null) return null;
    return { x, y, yaw: yaw ?? 0 };
  })();

  const draftGoal: DraftPose | null = (() => {
    const x = Number(ctx.goal.x);
    const y = Number(ctx.goal.y);
    const yaw = Number(ctx.goal.yaw);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    return { x, y, yaw: Number.isFinite(yaw) ? yaw : 0 };
  })();

  useEffect(() => {
    let frame = 0;
    const loop = () => {
      frame = window.requestAnimationFrame(loop);
      const canvas = canvasRef.current;
      if (!canvas) return;
      viewRef.current = drawMap(canvas, {
        state,
        reference,
        profile,
        layers: ctx.layers,
        selectedObstacle: ctx.selectedObstacle,
        draftBox,
        draftKidnap,
        draftGoal,
        armed: ctx.armed.map((a) => ({ slot: a.slot, kind: a.kind, params: a.params })),
        linkUp: state.linkUp,
        goalToleranceM: profile.goal_tolerance_m,
      });
    };
    frame = window.requestAnimationFrame(loop);
    return () => window.cancelAnimationFrame(frame);
  });

  const pointer = useCallback((event: React.MouseEvent) => {
    const canvas = canvasRef.current;
    const view = viewRef.current;
    if (!canvas || !view) return null;
    const m = toMap(view, canvas, event.clientX, event.clientY);
    return mapToWorld(m.x, m.y, 0, reference.world_to_map);
  }, [reference]);

  const onDown = (event: React.MouseEvent) => {
    const world = pointer(event);
    if (!world) return;
    if (ctx.pick === 'obs') {
      dragRef.current = { kind: 'obs', a: world };
      ctx.setForms((f) => ({ ...f, obs: { ...f.obs, x: f2(world.x), y: f2(world.y) } }));
      return;
    }
    if (ctx.pick === 'kid') {
      dragRef.current = { kind: 'kid', a: world };
      ctx.setForms((f) => ({ ...f, kid: { ...f.kid, x: f2(world.x), y: f2(world.y) } }));
      return;
    }
    if (ctx.pick === 'goal') {
      dragRef.current = { kind: 'goal', a: world };
      ctx.setGoal((g) => ({ ...g, x: f2(world.x), y: f2(world.y) }));
      return;
    }
    const hit = Object.keys(state.obstacles).find((name) => {
      const o = state.obstacles[name];
      return Math.abs(world.x - o.x) <= o.sx / 2 + 0.12
        && Math.abs(world.y - o.y) <= o.sy / 2 + 0.12;
    });
    ctx.setSelectedObstacle(hit ?? null);
    ctx.setPop(null);
  };

  const onMove = (event: React.MouseEvent) => {
    const drag = dragRef.current;
    if (!drag) return;
    const world = pointer(event);
    if (!world) return;
    const dx = world.x - drag.a.x;
    const dy = world.y - drag.a.y;
    const distance = Math.hypot(dx, dy);
    if (drag.kind === 'obs' && distance > 0.08) {
      ctx.setForms((f) => ({
        ...f,
        obs: {
          ...f.obs,
          x: f2((drag.a.x + world.x) / 2),
          y: f2((drag.a.y + world.y) / 2),
          sx: f2(Math.max(0.05, Math.abs(dx))),
          sy: f2(Math.max(0.05, Math.abs(dy))),
        },
      }));
    }
    if (drag.kind === 'kid' && distance > 0.1) {
      ctx.setForms((f) => ({ ...f, kid: { ...f.kid, yaw: f2(Math.atan2(dy, dx)) } }));
    }
    if (drag.kind === 'goal' && distance > 0.1) {
      ctx.setGoal((g) => ({ ...g, yaw: f2(Math.atan2(dy, dx)) }));
    }
    setDrawTick((v) => v + 1);
  };

  const onUp = () => {
    if (!dragRef.current) return;
    dragRef.current = null;
    ctx.setPick(null);
  };

  const scan = state.scan;
  const returns = scan ? scan.ranges.filter((r) => r !== null).length : null;
  const scanActive = (state.active?.active ?? [])
    .some((a) => a.kind === 'scan_degradation');
  const selected = ctx.selectedObstacle ? state.obstacles[ctx.selectedObstacle] : null;
  const view = viewRef.current;
  let selX = 0;
  let selY = 0;
  if (selected && view) {
    const m = worldToMap(selected.x, selected.y + selected.sy / 2, 0, reference.world_to_map);
    selX = view.ox + (m.x - view.x0) * view.s;
    selY = view.oy + (view.y1 - m.y) * view.s - 8;
  }
  const stack = state.stack;
  const bringing = stack?.state === 'starting';

  const layerLabel: Record<keyof Layers, string> = {
    map: isCar
      ? 'Background grid (committed map, NOT the robot\u2019s own)'
      : 'Map (static, /map from map_server)',
    world: `World geometry (${reference.world_geometry.source.split('/').slice(-1)[0]})`,
    gcost: 'Global costmap (/global_costmap/costmap)',
    lcost: 'Local costmap',
    truth: 'GROUND TRUTH (evaluation only)',
    est: `ESTIMATE — ${profile.estimate.label}`,
    plan: 'Plan (/plan, Nav2 global planner)',
    scan: '/scan — what the consumers see',
    raw: '/scan_raw — the undegraded sensor stream',
    foot: `Footprint r ${profile.footprint.radius_m} m (truth and estimate)`,
  };
  const layerDisabled: Partial<Record<keyof Layers, string>> = {
    lcost: 'published in the odom frame; map←odom composition is not implemented',
    ...(isCar
      ? {
        plan: 'ArduRover plans inside the autopilot; this platform publishes no /plan',
        gcost: 'no Nav2 on this platform, so there is no costmap',
      }
      : {}),
  };

  return (
    <div style={{
      flex: 1, minWidth: 0, display: 'flex', flexDirection: 'column',
      background: C.canvas, borderRight: `1px solid ${C.line}`,
    }}
    >
      <div style={{
        position: 'relative', zIndex: 12, display: 'flex', alignItems: 'center', gap: 10,
        padding: '7px 12px', borderBottom: '1px solid #3c3c3c', background: C.rowAlt,
      }}
      >
        <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
          WORLD VIEW
        </span>
        <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>frame map</span>
        <button
          type="button"
          onClick={() => ctx.setPop(ctx.pop === 'layers' ? null : 'layers')}
          style={{
            background: 'transparent', border: `1px solid ${C.border}`, color: C.text,
            font: `500 11px ${F.sans}`, padding: '3px 9px', cursor: 'pointer',
          }}
        >
          Layers ▾
        </button>
        {ctx.pop === 'layers' ? (
          <div style={{
            position: 'absolute', left: 170, top: 34, width: 380, background: C.dim,
            border: `1px solid ${C.lineBright}`, boxShadow: '0 10px 30px rgba(0,0,0,.55)',
            padding: 6, display: 'grid', gap: 1,
          }}
          >
            {LAYER_ORDER.map((key) => {
              const reason = layerDisabled[key];
              const on = ctx.layers[key] && !reason;
              return (
                <button
                  key={key}
                  type="button"
                  disabled={Boolean(reason)}
                  data-tip={reason}
                  onClick={() => ctx.toggleLayer(key)}
                  style={{
                    display: 'flex', alignItems: 'center', gap: 8, textAlign: 'left',
                    background: 'transparent', border: 0,
                    color: reason ? C.text6 : C.text, font: `11px ${F.mono}`,
                    padding: '5px 6px', cursor: reason ? 'help' : 'pointer',
                  }}
                >
                  <span style={{
                    width: 12, height: 12, border: `1px solid ${C.borderMax}`,
                    background: on ? '#d6d6d6' : 'transparent', flex: 'none',
                  }}
                  />
                  <span style={{ flex: 1 }}>
                    {layerLabel[key]}
                    {reason ? ` · unavailable` : ''}
                  </span>
                </button>
              );
            })}
          </div>
        ) : null}
        <div style={{ flex: 1 }} />
        <button
          type="button"
          onClick={() => ctx.setDrawerOpen(!ctx.drawerOpen)}
          style={{
            background: ctx.drawerOpen ? C.text : 'transparent',
            border: `1px solid ${C.border}`, color: ctx.drawerOpen ? C.ink : C.text,
            font: `500 11px ${F.sans}`, padding: '3px 10px', cursor: 'pointer',
          }}
        >
          Inject faults
        </button>
      </div>

      <div style={{ position: 'relative', flex: 1, minHeight: 0 }}>
        <div
          onMouseDown={onDown}
          onMouseMove={onMove}
          onMouseUp={onUp}
          style={{
            position: 'absolute', inset: 0,
            cursor: ctx.pick ? 'crosshair' : 'default',
          }}
        >
          <canvas
            ref={canvasRef}
            style={{
              position: 'absolute', inset: 0, width: '100%', height: '100%',
              display: 'block',
            }}
          />
        </div>

        <div style={{
          position: 'absolute', top: 10, right: 12, display: 'grid', justifyItems: 'end',
          gap: 4, background: 'rgba(30,30,30,.88)', padding: '6px 9px',
          border: `1px solid ${C.lineMid}`,
        }}
        >
          <span style={{ font: `600 12.5px ${F.mono}`, color: C.scan }}>
            {returns === null
              ? 'no /scan yet'
              : `${returns} / ${scan?.ranges.length ?? 0} returns`}
          </span>
          {scanActive ? (
            <button
              type="button"
              onClick={() => ctx.toggleLayer('raw')}
              style={{
                background: ctx.layers.raw ? C.text : 'transparent',
                color: ctx.layers.raw ? C.ink : C.text,
                border: `1px solid ${C.borderMax}`, font: `500 10.5px ${F.mono}`,
                padding: '2px 7px', cursor: 'pointer',
              }}
            >
              {ctx.layers.raw ? 'hide /scan_raw' : 'compare /scan_raw'}
            </button>
          ) : null}
        </div>

        {isCar ? (
          <div style={{
            position: 'absolute', top: 10, left: 12, display: 'flex', gap: 6,
            alignItems: 'center', background: 'rgba(30,30,30,.9)',
            border: `1px dashed ${C.borderMax}`, padding: '4px 8px',
          }}
          >
            <span style={{ font: `500 10.5px ${F.mono}`, color: C.text }}>
              background · not the robot&apos;s map
            </span>
            <Tip text="ArduRover does not plan against an occupancy grid, so the car has no /map, no costmaps and no /plan. The grid is drawn only to locate the body in the apartment." />
          </div>
        ) : null}

        {ctx.pick ? (
          <div style={{
            position: 'absolute', top: 10, left: '50%', transform: 'translateX(-50%)',
            background: C.text, color: C.ink, padding: '6px 12px',
            font: `600 11.5px ${F.sans}`, display: 'flex', gap: 10, alignItems: 'center',
          }}
          >
            <span>
              {ctx.pick === 'obs'
                ? 'Drag a rectangle for the obstacle footprint (world frame), or click to place at the current size'
                : ctx.pick === 'kid'
                  ? 'Click the kidnap destination; drag to set yaw'
                  : 'Click the goal position (world frame); drag to set yaw'}
            </span>
            <button
              type="button"
              onClick={() => ctx.setPick(null)}
              style={{
                background: C.ink, color: C.text, border: 0, font: `500 11px ${F.sans}`,
                padding: '2px 8px', cursor: 'pointer',
              }}
            >
              Esc
            </button>
          </div>
        ) : null}

        {selected && view ? (
          <div style={{
            position: 'absolute', left: selX, top: selY,
            transform: 'translate(-50%,-100%)', background: C.dim,
            border: `1px solid ${C.borderMax}`, padding: '6px 8px', display: 'grid',
            gap: 5, boxShadow: '0 6px 18px rgba(0,0,0,.5)', whiteSpace: 'nowrap',
          }}
          >
            <span style={{ font: `600 11.5px ${F.mono}` }}>{ctx.selectedObstacle}</span>
            <span style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>
              {`world (${f2(selected.x)}, ${f2(selected.y)}) · ${f2(selected.sx)} × `
                + `${f2(selected.sy)} m · ${selected.id}`}
            </span>
            <div style={{ display: 'flex', gap: 6 }}>
              <button
                type="button"
                disabled={Boolean(ctx.blockReason) || ctx.pending('clear')}
                onClick={() => ctx.sendFault('clear', 'clear_obstacle',
                  { name: ctx.selectedObstacle }, `clear '${ctx.selectedObstacle}'`)}
                style={{
                  background: C.text, color: C.ink, border: 0, font: `600 11px ${F.sans}`,
                  padding: '3px 9px', cursor: 'pointer',
                  opacity: ctx.blockReason || ctx.pending('clear') ? 0.5 : 1,
                }}
              >
                {ctx.pending('clear') ? 'pending…' : 'Clear obstacle'}
              </button>
              <button
                type="button"
                onClick={() => ctx.setSelectedObstacle(null)}
                style={{
                  background: 'transparent', color: C.text2,
                  border: `1px solid ${C.border}`, font: `500 11px ${F.sans}`,
                  padding: '3px 8px', cursor: 'pointer',
                }}
              >
                Close
              </button>
            </div>
          </div>
        ) : null}

        {ctx.requests.length ? (
          <div style={{
            position: 'absolute', left: 12, bottom: 10, width: 360, maxWidth: '45%',
            background: 'rgba(30,30,30,.92)', border: `1px solid ${C.lineMid}`,
            padding: '6px 9px', display: 'grid', gap: 4,
          }}
          >
            <span style={{
              font: `600 10px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
            }}
            >
              REQUESTS
            </span>
            {ctx.requests.slice(-3).reverse().map((request, index) => {
              const [label, colour] = request.status === 'pending'
                ? ['PENDING', C.text]
                : request.status === 'failed'
                  ? ['NO RESPONSE', C.warn]
                  : request.response?.success
                    ? ['OK', C.good]
                    : ['REFUSED', C.bad];
              return (
                <div
                  key={`${request.key}-${request.sentSim}-${index}`}
                  style={{
                    display: 'grid', gridTemplateColumns: 'auto minmax(0,1fr)', gap: 6,
                    alignItems: 'baseline',
                  }}
                  data-tip={request.response
                    ? `${request.service} at sim ${f2(request.sentSim)} s\n`
                      + `response message (verbatim):\n${request.response.message}`
                    : request.error
                      ?? `${request.service} sent at sim ${f2(request.sentSim)} s. `
                        + 'Nothing in the world has changed yet.'}
                >
                  <span style={{
                    font: `600 10px ${F.mono}`, color: colour, border: `1px solid ${colour}`,
                    padding: '0 4px',
                  }}
                  >
                    {label}
                  </span>
                  <span style={{
                    font: `10.5px ${F.mono}`, color: C.textBright, overflow: 'hidden',
                    textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                  }}
                  >
                    {`${request.service} · ${request.summary}`}
                    {request.response?.fault_id ? ` · ${request.response.fault_id}` : ''}
                  </span>
                </div>
              );
            })}
          </div>
        ) : null}

        {ctx.armed.length ? (
          <div style={{
            position: 'absolute', right: 12, bottom: 10, width: 320, maxWidth: '45%',
            background: 'rgba(30,30,30,.94)', border: `1px dashed ${C.text5}`,
            padding: '7px 9px', display: 'grid', gap: 5,
          }}
          >
            <span style={{
              font: `600 10px ${F.sans}`, letterSpacing: '.08em', color: C.text,
            }}
            >
              ARMED · NOT SENT
            </span>
            {ctx.armed.map((entry, index) => (
              <div
                key={entry.slot}
                style={{
                  display: 'grid', gridTemplateColumns: 'auto minmax(0,1fr) auto auto',
                  gap: 6, alignItems: 'center',
                }}
              >
                <span style={{
                  font: `600 11px ${F.mono}`, background: C.text, color: C.ink, width: 18,
                  textAlign: 'center',
                }}
                >
                  {entry.slot}
                </span>
                <span style={{ display: 'grid', minWidth: 0 }}>
                  <span style={{ font: `500 11px ${F.mono}` }}>{entry.kind}</span>
                  <span style={{
                    font: `10px ${F.mono}`, color: C.text3, overflow: 'hidden',
                    textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                  }}
                  >
                    {entry.summary}
                  </span>
                </span>
                <button
                  type="button"
                  disabled={Boolean(ctx.blockReason)}
                  onClick={() => ctx.fire(index)}
                  style={{
                    background: C.text, color: C.ink, border: 0,
                    font: `600 10.5px ${F.sans}`, padding: '3px 8px', cursor: 'pointer',
                    opacity: ctx.blockReason ? 0.45 : 1,
                  }}
                >
                  Send
                </button>
                <button
                  type="button"
                  onClick={() => ctx.disarm(index)}
                  style={{
                    background: 'transparent', color: C.text3,
                    border: `1px solid ${C.border}`, font: `500 10.5px ${F.sans}`,
                    padding: '2px 6px', cursor: 'pointer',
                  }}
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        ) : null}

        {bringing && stack ? <BringUp ctx={ctx} /> : null}
      </div>

      <div style={{
        display: 'flex', flexWrap: 'wrap', gap: '5px 16px', padding: '7px 12px',
        borderTop: '1px solid #3c3c3c', background: C.rowAlt, font: `10.5px ${F.mono}`,
        color: '#d6d6d6',
      }}
      >
        <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span style={{ width: 20, borderTop: `2px solid ${C.truth}` }} />
          GROUND TRUTH (evaluation only)
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span style={{ width: 20, borderTop: `2px dashed ${C.estimate}` }} />
          {`ESTIMATE — ${profile.estimate.label}`}
        </span>
        {!isCar ? (
          <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
            <span style={{ width: 20, borderTop: `2px dashed ${C.plan}` }} />
            Plan (nav2)
          </span>
        ) : null}
        <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span style={{ width: 6, height: 6, background: C.scan }} />
          /scan
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span style={{
            width: 10, height: 10, border: `1.5px solid ${C.command}`, borderRadius: '50%',
          }}
          />
          {`goal ± ${profile.goal_tolerance_m} m`}
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span style={{
            width: 10, height: 8, border: `1px solid ${C.truth}`,
            background: 'rgba(255,0,0,.15)',
          }}
          />
          obstacle
        </span>
        <span style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
          <span style={{ width: 10, height: 8, border: `1px dashed ${C.command}` }} />
          draft
        </span>
      </div>
    </div>
  );
}

function BringUp({ ctx }: { ctx: ConsoleCtx }) {
  const stack = ctx.state.stack!;
  return (
    <div style={{
      position: 'absolute', inset: 0, background: 'rgba(24,24,24,.82)', display: 'grid',
      placeItems: 'center',
    }}
    >
      <div style={{
        width: 420, background: C.dim, border: `1px solid ${C.lineBright}`,
        padding: '14px 16px', display: 'grid', gap: 8,
      }}
      >
        <div style={{
          display: 'flex', justifyContent: 'space-between', alignItems: 'baseline',
        }}
        >
          <span style={{
            font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
          }}
          >
            {`BRING-UP · ${stack.robot ?? ''}`}
          </span>
          <span style={{ font: `500 11px ${F.mono}` }}>
            {`${(stack.wall_elapsed_s ?? 0).toFixed(1)} s wall · budget `
              + `${stack.wall_limit_s ?? 0} s`}
          </span>
        </div>
        <div style={{ display: 'grid', gap: 3 }}>
          {stack.bringup.map((row) => (
            <div
              key={row.topic}
              style={{
                display: 'flex', justifyContent: 'space-between', font: `11px ${F.mono}`,
                color: row.seen ? C.text : C.text6,
              }}
            >
              <span>{`${row.seen ? '●' : '○'} ${row.topic}`}</span>
              <span>
                {row.seen && row.at_wall_s !== null
                  ? `+${row.at_wall_s.toFixed(1)} s`
                  : 'waiting'}
              </span>
            </div>
          ))}
        </div>
        <div style={{ font: `10.5px/1.45 ${F.mono}`, color: C.text3 }}>{stack.detail}</div>
      </div>
    </div>
  );
}
