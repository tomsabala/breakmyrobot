import { useEffect, useMemo, useState } from 'react';
import type { Screen } from '../App';
import type { Ral } from '../client/useRal';
import type { MetricValue, RunDetail, RunEntry } from '../client/types';
import { C, F } from '../theme/tokens';
import { Chevron, CopyButton, Tip, useTips } from '../ui/primitives';

const COMPARE_MAX = 4;

/** Provenance keys, in the order env.json records them. */
const PROV_KEYS = ['image_ref', 'git_sha', 'git_dirty', 'git_diff_sha256', 'map_sha256',
  'scenario_sha256', 'nav2_params_sha256', 'autopilot_parm_sha256', 'firmware_sha',
  'ros_domain_id', 'launch_command', 'world_to_map', 'world_to_ekf_local'];

/** The fields two runs must share before their metrics may be compared. */
const COMPARABLE: [string, (run: RunEntry) => unknown][] = [
  ['image_ref', (run) => run.prov.image_ref],
  ['git_sha', (run) => run.prov.git_sha],
  ['scenario_sha256', (run) => run.prov.scenario_sha256],
];

const COMPARE_METRICS = ['mission_duration_s', 'path_length_m', 'goal_pose_error_m',
  'est_error_mean_m', 'est_error_max_m', 'mission_result'];

/**
 * A metric value as text.
 *
 * Several metrics are STRUCTURED - mission_result carries {state, outcome,
 * source}, cmd_vs_achieved carries per-axis errors and a sample count - and
 * `String(object)` renders them as `[object Object]`, which hides the whole
 * measurement. They are shown as compact JSON instead: the shape is the
 * metric's own, and nothing is dropped to make it fit.
 */
function metricText(value: MetricValue['value']): string {
  if (value === null) return 'null';
  if (typeof value === 'object') return JSON.stringify(value);
  return String(value);
}

export function Inspector({ ral, screen, go }: {
  ral: Ral; screen: Screen; go: (screen: Screen) => void;
}) {
  const { source, reference } = ral;
  const [runs, setRuns] = useState<RunEntry[] | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [selected, setSelected] = useState<string | null>(null);
  const [detail, setDetail] = useState<RunDetail | null>(null);
  const [details, setDetails] = useState<Record<string, RunDetail>>({});
  const [checked, setChecked] = useState<string[]>([]);
  const [filterRobot, setFilterRobot] = useState('all');
  const [filterScenario, setFilterScenario] = useState('all');
  const [filterOutcome, setFilterOutcome] = useState('all');
  const [newestFirst, setNewestFirst] = useState(true);
  const { onTip, hideTip, layer } = useTips();

  useEffect(() => {
    source.runs()
      .then((list) => {
        setRuns(list);
        setSelected((current) => current ?? list[0]?.id ?? null);
      })
      .catch((reason: unknown) => setError(String(reason)));
  }, [source]);

  useEffect(() => {
    if (!selected) return;
    setDetail(null);
    source.run(selected)
      .then((value) => {
        setDetail(value);
        setDetails((current) => ({ ...current, [value.id]: value }));
      })
      .catch((reason: unknown) => setError(String(reason)));
  }, [source, selected]);

  // Compare needs the metrics, which live in the DETAIL payload. Fetch each
  // checked run once; the index alone cannot answer a metric question.
  useEffect(() => {
    for (const id of checked) {
      if (details[id]) continue;
      source.run(id)
        .then((value) => setDetails((current) => ({ ...current, [value.id]: value })))
        .catch(() => undefined);
    }
  }, [checked, details, source]);

  const filtered = useMemo(() => {
    if (!runs) return [];
    const list = runs.filter((run) => (filterRobot === 'all' || run.robot === filterRobot)
      && (filterScenario === 'all' || run.scenario === filterScenario)
      && (filterOutcome === 'all' || run.outcome === filterOutcome));
    return newestFirst ? list : [...list].reverse();
  }, [runs, filterRobot, filterScenario, filterOutcome, newestFirst]);

  const scenarios = useMemo(
    () => [...new Set((runs ?? []).map((run) => run.scenario))].sort(), [runs]);
  const outcomes = useMemo(
    () => [...new Set((runs ?? []).map((run) => run.outcome).filter(Boolean))].sort(),
    [runs]);

  return (
    <div
      onMouseOver={onTip}
      onFocus={onTip}
      onBlur={hideTip}
      onMouseLeave={hideTip}
      style={{
        display: 'grid', gridTemplateRows: 'auto minmax(0,1fr)', height: '100vh',
        minHeight: 700, minWidth: 1280, fontSize: '12.5px', lineHeight: 1.4,
      }}
    >
      <div style={{
        display: 'flex', alignItems: 'center', gap: 14, padding: '0 12px', height: 46,
        borderBottom: `1px solid ${C.line}`, background: C.inset, whiteSpace: 'nowrap',
      }}
      >
        <span style={{ font: `600 12px ${F.sans}`, letterSpacing: '.1em' }}>RAL</span>
        <div style={{ display: 'flex', gap: 2 }}>
          {([['console', 'Console'], ['runs', 'Runs'], ['compare', 'Compare'],
            ['reference', 'Reference']] as [Screen, string][]).map(([key, label]) => (
              <button
                key={key}
                type="button"
                onClick={() => go(key)}
                style={{
                  font: `500 11.5px ${F.sans}`, padding: '4px 8px',
                  background: screen === key ? C.header : 'transparent',
                  color: screen === key ? C.text : C.text3, border: 0, cursor: 'pointer',
                }}
              >
                {label}
              </button>
          ))}
        </div>
        <div style={{ flex: 1 }} />
        <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>
          {`runs/ · ${runs?.length ?? '…'} directories`}
        </span>
        <span style={{
          font: `500 10.5px ${F.mono}`, letterSpacing: '.04em',
          border: `1px solid ${C.borderAlt}`, padding: '3px 7px',
        }}
        >
          {source.kind === 'mock' ? 'MOCK' : 'LIVE'}
        </span>
      </div>

      {error ? (
        <div style={{ padding: 20, font: `12px ${F.mono}`, color: C.bad }}>{error}</div>
      ) : screen === 'runs' ? (
        <div style={{ display: 'flex', minHeight: 0 }}>
          <div style={{
            width: 500, flex: 'none', display: 'flex', flexDirection: 'column',
            borderRight: `1px solid ${C.line}`, background: C.drawer, minHeight: 0,
          }}
          >
            <div style={{
              display: 'flex', gap: 6, alignItems: 'center', padding: '10px 12px',
              borderBottom: `1px solid ${C.header}`, flexWrap: 'wrap',
            }}
            >
              <div style={{ display: 'flex', border: `1px solid ${C.lineMid}` }}>
                {['all', ...Object.keys(reference!.profiles).sort()].map((robot) => (
                  <button
                    key={robot}
                    type="button"
                    onClick={() => setFilterRobot(robot)}
                    style={{
                      background: filterRobot === robot ? C.text : 'transparent',
                      color: filterRobot === robot ? C.ink : C.text2, border: 0,
                      font: `500 10.5px ${F.mono}`, padding: '3px 8px', cursor: 'pointer',
                    }}
                  >
                    {robot}
                  </button>
                ))}
              </div>
              <select
                value={filterScenario}
                onChange={(event) => setFilterScenario(event.target.value)}
                style={selectStyle}
              >
                <option value="all">all scenarios</option>
                {scenarios.map((s) => <option key={s} value={s}>{s}</option>)}
              </select>
              <select
                value={filterOutcome}
                onChange={(event) => setFilterOutcome(event.target.value)}
                style={{ ...selectStyle, maxWidth: 170 }}
              >
                <option value="all">all outcomes</option>
                {outcomes.map((o) => <option key={o} value={o}>{o}</option>)}
              </select>
              <button
                type="button"
                onClick={() => setNewestFirst((value) => !value)}
                style={{
                  background: 'transparent', border: `1px solid ${C.lineMid}`,
                  color: C.text2, font: `500 10.5px ${F.mono}`, padding: '3px 8px',
                  cursor: 'pointer',
                }}
              >
                {newestFirst ? 'newest first' : 'oldest first'}
              </button>
            </div>

            <div style={{ flex: 1, overflowY: 'auto' }}>
              {filtered.map((run) => {
                const comparable = !run.partial && run.exit === 0 && run.scorable;
                const isChecked = checked.includes(run.id);
                return (
                  <div
                    key={run.id}
                    style={{
                      display: 'flex', gap: 8, alignItems: 'flex-start', padding: '8px 12px',
                      borderBottom: `1px solid ${C.row}`,
                      background: selected === run.id ? C.rowAlt : 'transparent',
                    }}
                  >
                    <input
                      type="checkbox"
                      checked={isChecked}
                      disabled={!comparable || (!isChecked && checked.length >= COMPARE_MAX)}
                      data-tip={comparable ? undefined
                        : 'not comparable: partial, non-zero exit, or metrics_version < 5'}
                      onChange={() => setChecked((list) => (isChecked
                        ? list.filter((id) => id !== run.id)
                        : [...list, run.id]))}
                      style={{ marginTop: 2, accentColor: C.text3 }}
                    />
                    <button
                      type="button"
                      onClick={() => setSelected(run.id)}
                      style={{
                        all: 'unset', flex: 1, minWidth: 0, display: 'grid', gap: 2,
                        cursor: 'pointer',
                      }}
                    >
                      <span style={{
                        font: `500 11px ${F.mono}`, overflow: 'hidden',
                        textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                      }}
                      >
                        {run.id}
                      </span>
                      <span style={{
                        display: 'flex', gap: 6, alignItems: 'baseline', flexWrap: 'wrap',
                      }}
                      >
                        <span style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>
                          {run.robot ?? '—'}
                        </span>
                        <span style={{
                          font: `500 10.5px ${F.mono}`,
                          color: run.outcome ? C.text : C.text6,
                        }}
                        >
                          {run.partial ? 'no run.json' : (run.outcome || '—')}
                        </span>
                        {badges(run).map(([text, colour]) => (
                          <span
                            key={text}
                            style={{
                              font: `500 9.5px ${F.mono}`, color: colour,
                              border: `1px solid ${colour === C.warn ? C.warnBorder : C.border}`,
                              padding: '0 4px',
                            }}
                          >
                            {text}
                          </span>
                        ))}
                      </span>
                    </button>
                    <span style={{ font: `10px ${F.mono}`, color: C.text6 }}>
                      {(run.wall[0] ?? '').slice(5, 16).replace('T', ' ')}
                    </span>
                  </div>
                );
              })}
              {filtered.length === 0 ? (
                <div style={{ padding: 14, font: `11.5px ${F.sans}`, color: C.text6 }}>
                  No runs match.
                </div>
              ) : null}
            </div>

            <div style={{
              display: 'flex', gap: 8, alignItems: 'center', padding: '8px 12px',
              borderTop: `1px solid ${C.header}`,
            }}
            >
              <span style={{ font: `10.5px ${F.mono}`, color: C.text4, flex: 1 }}>
                {`${checked.length} selected for compare · max ${COMPARE_MAX}`}
              </span>
              <button
                type="button"
                disabled={checked.length < 2}
                onClick={() => go('compare')}
                style={{
                  background: C.text, color: C.ink, border: 0, font: `600 11px ${F.sans}`,
                  padding: '4px 11px', cursor: 'pointer',
                  opacity: checked.length < 2 ? 0.45 : 1,
                }}
              >
                Compare
              </button>
            </div>
          </div>

          <div style={{
            flex: 1, minWidth: 0, overflowY: 'auto', padding: '16px 20px', display: 'grid',
            gap: 14, alignContent: 'start',
          }}
          >
            {detail ? <RunDetailView detail={detail} source={source} /> : (
              <div style={{
                font: `12px ${F.sans}`, color: C.text6, paddingTop: 40, textAlign: 'center',
              }}
              >
                {selected ? 'loading…' : 'Select a run.'}
              </div>
            )}
          </div>
        </div>
      ) : screen === 'compare' ? (
        <Compare
          checked={checked}
          details={details}
          onRemove={(id) => setChecked((list) => list.filter((item) => item !== id))}
          ral={ral}
        />
      ) : (
        <ReferenceView ral={ral} />
      )}
      {layer}
    </div>
  );
}

const selectStyle = {
  background: C.ink, color: C.text, border: `1px solid ${C.lineMid}`, fontSize: 10.5,
  padding: '3px 4px',
} as const;

function badges(run: RunEntry): [string, string][] {
  const list: [string, string][] = [];
  if (run.partial) list.push(['partial', C.text3]);
  if (run.version !== null && run.version < 5) {
    list.push([`metrics_version ${run.version}`, C.warn]);
  }
  if (run.prov.git_dirty === true) list.push(['git_dirty', C.warn]);
  if (run.exit !== null && run.exit !== 0) list.push([`exit ${run.exit}`, C.text3]);
  return list;
}

function Section({ title, sub, children, defaultOpen = false }: {
  title: string; sub: string; children: React.ReactNode; defaultOpen?: boolean;
}) {
  const [open, setOpen] = useState(defaultOpen);
  return (
    <div style={{
      borderTop: '1px solid #2e2e2e', paddingTop: 8, display: 'grid', gap: 8,
    }}
    >
      <button
        type="button"
        onClick={() => setOpen((value) => !value)}
        style={{
          all: 'unset', display: 'flex', gap: 8, alignItems: 'baseline', cursor: 'pointer',
        }}
      >
        <Chevron open={open} />
        <span style={{
          font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2,
        }}
        >
          {title}
        </span>
        <span style={{ font: `10.5px ${F.mono}`, color: C.text6 }}>{sub}</span>
      </button>
      {open ? <div style={{ paddingLeft: 18 }}>{children}</div> : null}
    </div>
  );
}

function KeyValues({ rows }: { rows: [string, string, string?][] }) {
  return (
    <div style={{
      display: 'grid', gridTemplateColumns: '200px minmax(0,1fr)', gap: '4px 12px',
      font: `11px ${F.mono}`,
    }}
    >
      {rows.map(([key, value, colour]) => (
        <span key={key} style={{ display: 'contents' }}>
          <span style={{ color: C.text4 }}>{key}</span>
          <span style={{ color: colour ?? C.text, wordBreak: 'break-all' }}>{value}</span>
        </span>
      ))}
    </div>
  );
}

function RunDetailView({ detail, source }: { detail: RunDetail; source: Ral['source'] }) {
  const banners: [string, string, string][] = [];
  if (detail.partial) banners.push([`exit ${detail.exit ?? 2} · ${detail.note}`, C.text2, C.border]);
  if (detail.outcome === 'runner:bringup_failed') {
    banners.push(['runner:bringup_failed · no mission state exists for this run',
      C.text2, C.border]);
  }
  if (detail.version !== null && detail.version < 5) {
    banners.push([`metrics_version ${detail.version} found · scoring requires version 5 `
      + '(earlier outcomes lack the runner: prefix and run.json has no mission_driver); '
      + 'the data below is shown as recorded', C.warn, C.warnBorder]);
  }
  if (detail.prov.git_dirty === true) {
    banners.push(['git_dirty: true · this run does not correspond to a commit',
      C.warn, C.warnBorder]);
  }

  const metricKeys = detail.metrics ? Object.keys(detail.metrics) : [];
  const nulls = metricKeys.filter((k) => detail.metrics![k].value === null).length;

  return (
    <>
      <div style={{ display: 'grid', gap: 6 }}>
        <div style={{ display: 'flex', gap: 10, alignItems: 'baseline' }}>
          <span style={{ font: `600 15px ${F.mono}`, wordBreak: 'break-all', flex: 1 }}>
            {detail.id}
          </span>
          <CopyButton text={detail.id} label="Copy id" />
        </div>
        <div style={{
          display: 'flex', gap: 16, flexWrap: 'wrap', font: `11px ${F.mono}`,
          color: C.text3,
        }}
        >
          <span>{detail.robot ?? '—'}</span>
          <span>{detail.world ?? '—'}</span>
          <span>{`seed ${detail.seed ?? '—'}`}</span>
          <span>{`exit ${detail.exit ?? '—'}`}</span>
          <span>
            {detail.sim[0] !== null
              ? `sim ${detail.sim[0]}–${detail.sim[1]} s`
              : 'sim —'}
          </span>
          <span>{`wall ${detail.wall[0] ?? '—'} → ${detail.wall[1] ?? '—'}`}</span>
        </div>
      </div>

      {banners.map(([text, fg, bd]) => (
        <div
          key={text}
          style={{
            border: `1px solid ${bd}`, color: fg, padding: '7px 10px',
            font: `500 11.5px ${F.mono}`,
          }}
        >
          {text}
        </div>
      ))}

      {detail.outcome ? (
        <div style={{
          display: 'grid', gap: 5, background: C.side, border: `1px solid ${C.line}`,
          padding: '10px 12px',
        }}
        >
          <div style={{
            display: 'flex', gap: 10, alignItems: 'baseline', flexWrap: 'wrap',
          }}
          >
            <span style={{ font: `11px ${F.sans}`, color: C.text4 }}>outcome</span>
            <span style={{ font: `600 13px ${F.mono}` }}>{detail.outcome}</span>
            <span style={{ font: `11px ${F.mono}`, color: C.text4 }}>{detail.state}</span>
          </div>
          <div style={{
            font: `10.5px/1.45 ${F.mono}`, color: C.text2, whiteSpace: 'pre-wrap',
            wordBreak: 'break-word',
          }}
          >
            {detail.detail}
          </div>
        </div>
      ) : null}

      <Section title="PROVENANCE" sub="env.json" defaultOpen>
        <KeyValues
          rows={PROV_KEYS.map((key) => {
            const value = detail.prov[key];
            const text = value === null || value === undefined
              ? (key === 'world_to_ekf_local' && detail.robot !== 'car'
                ? `n/a · ${detail.robot} has no ekf_local`
                : 'null')
              : JSON.stringify(value).replace(/^"|"$/g, '');
            const colour = value === null || value === undefined
              ? C.text6
              : (key === 'git_dirty' && value === true ? C.warn : C.text);
            return [key, text, colour];
          })}
        />
      </Section>

      {detail.metrics ? (
        <Section
          title="METRICS"
          sub={`metrics_version ${detail.version} · ${nulls} null`}
          defaultOpen
        >
          <div style={{ display: 'grid' }}>
            {metricKeys.map((key) => {
              const metric: MetricValue = detail.metrics![key];
              const isNull = metric.value === null;
              return (
                <div
                  key={key}
                  style={{
                    display: 'grid',
                    gridTemplateColumns: '240px 170px minmax(0,1fr) 18px',
                    gap: 10, alignItems: 'center', padding: '4px 0',
                    borderBottom: `1px solid ${C.row}`,
                  }}
                >
                  <span style={{ font: `500 11px ${F.mono}` }}>{key}</span>
                  <span style={{
                    font: `600 11px ${F.mono}`, color: isNull ? C.text4 : C.text,
                  }}
                  >
                    {isNull ? 'null' : metricText(metric.value)}
                  </span>
                  <span style={{
                    font: `10.5px ${F.mono}`, color: C.text4, overflow: 'hidden',
                    textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                  }}
                  >
                    {isNull ? metric.reason ?? '' : metric.unit}
                  </span>
                  <Tip
                    text={`unit: ${metric.unit || "''"}\n`
                      + `source: ${metric.source ?? 'not recorded'}\n`
                      + `definition: ${metric.definition ?? 'not recorded'}`
                      + (isNull ? `\nreason: ${metric.reason ?? ''}` : '')}
                  />
                </div>
              );
            })}
          </div>
        </Section>
      ) : null}

      <Section title="INTERVENTIONS" sub={String(detail.interventions.length)}>
        {detail.interventions.length === 0 ? (
          <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>none</span>
        ) : (
          <div style={{ display: 'grid', gap: 8 }}>
            {detail.interventions.map((raw, index) => {
              const row = raw as Record<string, unknown>;
              return (
                <div
                  key={String(row.id ?? index)}
                  style={{
                    border: `1px solid ${C.row}`, padding: '6px 8px', display: 'grid',
                    gap: 4, font: `10.5px ${F.mono}`,
                  }}
                >
                  <span style={{ font: `600 11px ${F.mono}` }}>
                    {`${String(row.id)} · ${String(row.type)} · at_sec ${String(row.at_sec)}`
                      + ` · dispatched ${String(row.dispatched_sim_s)} · `
                      + `${String(row.result)}`}
                  </span>
                  <span style={{ color: C.text4, wordBreak: 'break-all' }}>
                    {`${String(row.service)} · ${String(row.injection_point)} · `
                      + `${String(row.target)} · ${JSON.stringify(row.params)}`}
                  </span>
                  <span style={{ whiteSpace: 'pre-wrap', wordBreak: 'break-word' }}>
                    {String(row.message ?? '')}
                  </span>
                </div>
              );
            })}
          </div>
        )}
      </Section>

      <Section
        title="PRE-FLIGHT"
        sub={detail.preflight
          ? (detail.preflight.ok ? 'ok' : String(detail.preflight.outcome))
          : 'not recorded'}
      >
        {detail.preflight ? (
          <KeyValues
            rows={Object.entries(detail.preflight).map(([key, value]) => (
              [key, typeof value === 'string' ? value : JSON.stringify(value)]
            ))}
          />
        ) : <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>not recorded</span>}
      </Section>

      <Section
        title="BAG MANIFEST"
        sub={detail.bag_duration_s !== null
          ? `mcap · ${detail.bag_duration_s.toFixed(1)} s`
          : 'no metadata.yaml'}
      >
        {detail.bag ? (
          <div style={{ display: 'grid' }}>
            {detail.bag.map(([topic, type, count]) => (
              <div
                key={topic}
                style={{
                  display: 'grid', gridTemplateColumns: 'minmax(0,1.3fr) minmax(0,1.6fr) 170px',
                  gap: 10, padding: '4px 0', borderBottom: `1px solid ${C.row}`,
                  font: `10.5px ${F.mono}`, color: count === 0 ? C.text4 : C.text,
                }}
              >
                <span style={{ wordBreak: 'break-all' }}>{topic}</span>
                <span style={{ wordBreak: 'break-all' }}>{type}</span>
                <span>{count === 0 ? 'recorded, 0 messages' : String(count)}</span>
              </div>
            ))}
          </div>
        ) : (
          <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>
            no bag/metadata.yaml: the recorder was killed before it finalised, or the run
            never got that far
          </span>
        )}
      </Section>

      <Section title="FIGURE" sub="plot.png">
        {detail.has_plot ? (
          <img
            src={`/api/runs/${encodeURIComponent(detail.id)}/plot.png`}
            alt={`plot.png for ${detail.id}`}
            style={{ maxWidth: 860, width: '100%', border: `1px solid ${C.line}` }}
          />
        ) : (
          <span style={{ font: `11px ${F.sans}`, color: C.text6 }}>
            no plot.png: the figure is written after the metrics, so a run that failed
            before that has none
          </span>
        )}
      </Section>

      <Section title="LOGS" sub="launch.log · bag_record.log">
        <div style={{ display: 'grid', gap: 8 }}>
          {([['launch.log', detail.log_tail],
            ['bag_record.log', detail.bag_record_log_tail]] as const).map(([name, text]) => (
              <div key={name} style={{ display: 'grid', gap: 4 }}>
                <span style={{ font: `10.5px ${F.mono}`, color: C.text4 }}>
                  {`${name} · last 4 KiB`}
                </span>
                <div style={{
                  font: `10.5px/1.5 ${F.mono}`, background: C.ink,
                  border: `1px solid ${C.line}`, padding: '8px 10px',
                  whiteSpace: 'pre-wrap', maxHeight: 260, overflow: 'auto',
                }}
                >
                  {text ?? 'absent'}
                </div>
              </div>
          ))}
        </div>
      </Section>

      <Section title="SCENARIO" sub="scenario.yaml, as copied into the run">
        <div style={{
          font: `10.5px/1.5 ${F.mono}`, background: C.ink, border: `1px solid ${C.line}`,
          padding: '8px 10px', whiteSpace: 'pre-wrap', maxHeight: 320, overflow: 'auto',
        }}
        >
          {detail.scenario_yaml ?? 'absent'}
        </div>
      </Section>
      {source.kind === 'mock' ? null : null}
    </>
  );
}

function Compare({ checked, details, onRemove, ral }: {
  checked: string[];
  details: Record<string, RunDetail>;
  onRemove: (id: string) => void;
  ral: Ral;
}) {
  const runs = checked.map((id) => details[id]).filter(Boolean);
  const refusals: string[] = [];
  if (runs.length >= 2) {
    for (const [field, get] of COMPARABLE) {
      const values = [...new Set(runs.map((run) => JSON.stringify(get(run))))];
      if (values.length > 1) {
        refusals.push(`Refused: ${field} differs (${values.map((v) => v.replace(/^"|"$/g, '')).join(' vs ')})`);
      }
    }
  }
  const ok = runs.length >= 2 && refusals.length === 0;
  const reference = ral.reference!;
  const profiles = Object.keys(reference.profiles).sort();

  return (
    <div style={{
      overflowY: 'auto', padding: '16px 20px', display: 'grid', gap: 16,
      alignContent: 'start',
    }}
    >
      <div style={{ display: 'flex', gap: 8, alignItems: 'center', flexWrap: 'wrap' }}>
        <span style={{
          font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text4,
        }}
        >
          COMPARE
        </span>
        {checked.map((id) => (
          <span
            key={id}
            style={{
              display: 'flex', gap: 6, alignItems: 'center',
              border: `1px solid ${C.lineMid}`, padding: '2px 4px 2px 8px',
              font: `10.5px ${F.mono}`,
            }}
          >
            {id}
            <button
              type="button"
              onClick={() => onRemove(id)}
              style={{
                background: 'transparent', border: 0, color: C.text4, cursor: 'pointer',
                fontSize: 12,
              }}
            >
              ×
            </button>
          </span>
        ))}
        {checked.length < 2 ? (
          <span style={{ font: `11.5px ${F.sans}`, color: C.text6 }}>
            Pick 2–4 runs in Runs.
          </span>
        ) : null}
        {checked.length >= 2 && runs.length < checked.length ? (
          <span style={{ font: `11.5px ${F.sans}`, color: C.text6 }}>loading…</span>
        ) : null}
      </div>

      {refusals.map((text) => (
        <div
          key={text}
          style={{
            border: `1px solid ${C.warnBorder}`, color: C.warn, padding: '8px 10px',
            font: `500 11.5px ${F.mono}`,
          }}
        >
          {text}
        </div>
      ))}
      {refusals.length ? (
        <div style={{ font: `11px ${F.sans}`, color: C.text4 }}>
          Two runs produced by different code, a different image or a different scenario
          are not two samples of one thing. The metrics are not shown rather than shown
          with a caveat.
        </div>
      ) : null}

      {ok ? (
        <>
          <div style={{ display: 'grid', border: `1px solid ${C.line}` }}>
            <div style={{
              display: 'grid',
              gridTemplateColumns: `220px repeat(${runs.length}, minmax(0,1fr))`,
              gap: 10, padding: '5px 10px', borderBottom: `1px solid ${C.row}`,
              background: C.dim, font: `500 10.5px ${F.mono}`, color: C.text4,
            }}
            >
              <span>metric</span>
              {runs.map((run) => (
                <span key={run.id} style={{ wordBreak: 'break-all' }}>{run.id}</span>
              ))}
            </div>
            {COMPARE_METRICS.map((key) => (
              <div
                key={key}
                style={{
                  display: 'grid',
                  gridTemplateColumns: `220px repeat(${runs.length}, minmax(0,1fr))`,
                  gap: 10, padding: '5px 10px', borderBottom: `1px solid ${C.row}`,
                  font: `11px ${F.mono}`,
                }}
              >
                <span style={{ color: C.text4 }}>{key}</span>
                {runs.map((run) => {
                  const metric = run.metrics?.[key];
                  return (
                    <span
                      key={run.id}
                      style={{ color: metric?.value === null ? C.text4 : C.text }}
                      data-tip={metric?.value === null ? metric.reason ?? undefined : undefined}
                    >
                      {metric ? metricText(metric.value) : '—'}
                    </span>
                  );
                })}
              </div>
            ))}
            <div style={{
              display: 'grid',
              gridTemplateColumns: `220px repeat(${runs.length}, minmax(0,1fr))`,
              gap: 10, padding: '5px 10px', font: `11px ${F.mono}`, color: C.text4,
            }}
            >
              <span>N</span>
              {runs.map((run) => <span key={run.id}>1</span>)}
            </div>
          </div>
          <div style={{
            display: 'flex', gap: 6, alignItems: 'center', font: `11px ${F.sans}`,
            color: C.text4,
          }}
          >
            <span>
              {`${runs.length} runs shown side by side. This is not a spread: a spread is `
                + 'measured over repeated runs of one scenario.'}
            </span>
            <Tip text="seed governs only ral_faults' RNG. nav2_amcl exposes no seed, ArduPilot EKF3 is not seedable, and callback scheduling is not deterministic. A wider spread is a property of the platform." />
          </div>
        </>
      ) : null}

      <div style={{
        display: 'grid', gap: 8, borderTop: '1px solid #2e2e2e', paddingTop: 12,
      }}
      >
        <span style={{
          font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2,
        }}
        >
          {`NARROW DOORWAY · ${profiles.join(' VS ').toUpperCase()}`}
        </span>
        <DoorwayTable ral={ral} details={details} />
      </div>
    </div>
  );
}

function DoorwayTable({ ral, details }: {
  ral: Ral; details: Record<string, RunDetail>;
}) {
  const reference = ral.reference!;
  const names = Object.keys(reference.profiles).sort();
  const doorways = reference.world_geometry.doorways;
  const rows: [string, string, string][] = [
    ['', names[0] ?? '', names[1] ?? ''],
    ...names.length === 2 ? [
      ['min turn radius',
        `${reference.profiles[names[0]].plant.min_turn_radius_m} m`,
        `${reference.profiles[names[1]].plant.min_turn_radius_m} m`] as [string, string, string],
      ['mission driver',
        reference.profiles[names[0]].mission_driver,
        reference.profiles[names[1]].mission_driver] as [string, string, string],
      ['goal tolerance',
        `${reference.profiles[names[0]].goal_tolerance_m} m`,
        `${reference.profiles[names[1]].goal_tolerance_m} m`] as [string, string, string],
      ['footprint radius',
        `${reference.profiles[names[0]].footprint.radius_m} m`,
        `${reference.profiles[names[1]].footprint.radius_m} m`] as [string, string, string],
      ['body width (2r)',
        `${(reference.profiles[names[0]].footprint.radius_m * 2).toFixed(3)} m`,
        `${(reference.profiles[names[1]].footprint.radius_m * 2).toFixed(3)} m`] as [string, string, string],
    ] : [],
    ...doorways.map((door) => [
      `${door.label} ${door.width_m.toFixed(2)} m`,
      names[0] ? clearance(reference.profiles[names[0]].footprint.radius_m, door.width_m) : '',
      names[1] ? clearance(reference.profiles[names[1]].footprint.radius_m, door.width_m) : '',
    ] as [string, string, string]),
  ];

  // The outcomes, only where a narrow_doorway run for that robot exists.
  for (const robot of names) {
    const run = Object.values(details)
      .find((d) => d.robot === robot && d.scenario.startsWith('narrow_doorway'));
    rows.push([`narrow_doorway_${robot} outcome`,
      run ? run.outcome : 'not loaded: select that run in Runs', '']);
  }

  return (
    <div style={{ display: 'grid', border: `1px solid ${C.line}`, maxWidth: 900 }}>
      {rows.map((row, index) => (
        <div
          key={row[0] + index}
          style={{
            display: 'grid', gridTemplateColumns: '240px 1fr 1fr', gap: 10,
            padding: '5px 10px', borderBottom: `1px solid ${C.row}`,
            font: `11px ${F.mono}`, background: index === 0 ? C.dim : 'transparent',
          }}
        >
          <span style={{ color: C.text4 }}>{row[0]}</span>
          <span>{row[1]}</span>
          <span>{row[2]}</span>
        </div>
      ))}
    </div>
  );
}

function clearance(radius: number, width: number): string {
  const gap = width - radius * 2;
  return `${gap.toFixed(3)} m clear (${(radius * 2).toFixed(3)} m body)`;
}

function ReferenceView({ ral }: { ral: Ral }) {
  const reference = ral.reference!;
  const kinds = Object.keys(reference.catalogue.faults);
  const names = Object.keys(reference.profiles).sort();

  return (
    <div style={{
      overflowY: 'auto', padding: '16px 20px', display: 'grid', gap: 18,
      alignContent: 'start',
    }}
    >
      <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
        FAULT CATALOGUE · fault_catalogue.yaml
      </span>
      <div style={{
        display: 'grid', gridTemplateColumns: 'repeat(auto-fill,minmax(400px,1fr))', gap: 10,
      }}
      >
        {kinds.map((kind) => {
          const entry = reference.catalogue.faults[kind];
          const supported = names.filter((n) => reference.profiles[n].faults.includes(kind));
          const unsupported = names
            .filter((n) => !reference.profiles[n].faults.includes(kind))
            .map((n) => `${n}: ${reference.profiles[n].unsupported?.[kind]
              ?? 'not listed in its profile'}`);
          const ranges = Object.entries(entry.param_ranges);
          return (
            <div
              key={kind}
              style={{
                background: C.side, border: `1px solid ${C.line}`, padding: 12,
                display: 'grid', gap: 6, alignContent: 'start',
              }}
            >
              <div style={{
                display: 'flex', justifyContent: 'space-between', alignItems: 'baseline',
              }}
              >
                <span style={{ font: `600 12.5px ${F.mono}` }}>{kind}</span>
                <span style={{ font: `10.5px ${F.sans}`, color: C.text4 }}>
                  {`${entry.category} · ${entry.lifetime} · `
                    + `${entry.alters_truth ? 'changes world' : 'world unchanged'}`}
                </span>
              </div>
              <div style={{
                display: 'grid', gridTemplateColumns: '120px minmax(0,1fr)', gap: '4px 10px',
                font: `10.5px ${F.mono}`,
              }}
              >
                <span style={{ color: C.text4 }}>service</span>
                <span style={{ wordBreak: 'break-word' }}>{entry.service}</span>
                <span style={{ color: C.text4 }}>type</span>
                <span style={{ wordBreak: 'break-word' }}>{entry.srv_type}</span>
                <span style={{ color: C.text4 }}>injection point</span>
                <span style={{ wordBreak: 'break-word' }}>
                  {entry.injection_point.replace('{world}', reference.world)}
                </span>
                <span style={{ color: C.text4 }}>target rule</span>
                <span>{entry.target_rule}</span>
                <span style={{ color: C.text4 }}>removal</span>
                <span>{entry.cleared_by ?? 'none — one-shot, cannot be cleared'}</span>
                <span style={{ color: C.text4 }}>supported</span>
                <span style={{ wordBreak: 'break-word' }}>
                  {supported.join(', ') || 'none'}
                  {unsupported.length ? ` · ${unsupported.join(' · ')}` : ''}
                </span>
                <span style={{ color: C.text4 }}>bounds</span>
                <span style={{ wordBreak: 'break-word' }}>
                  {ranges.length
                    ? ranges.map(([field, [lo, hi]]) => `${field} [${lo ?? 'None'}, `
                      + `${hi ?? 'None'}]`).join(' · ')
                    : 'none declared; finiteness is still checked'}
                </span>
              </div>
              {entry.honesty_note ? (
                <div style={{
                  font: `11px/1.45 ${F.sans}`, color: C.text3, fontStyle: 'italic',
                  borderTop: '1px solid #2e2e2e', paddingTop: 6,
                }}
                >
                  {entry.honesty_note}
                </div>
              ) : null}
            </div>
          );
        })}
      </div>

      <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
        ROBOT PROFILES · config/robots/*.yaml
      </span>
      <div style={{ display: 'grid', border: `1px solid ${C.line}`, maxWidth: 1100 }}>
        {([
          ['field', ...names],
          ['model', ...names.map((n) => reference.profiles[n].model)],
          ['stack launch', ...names.map((n) => reference.profiles[n].stack_launch)],
          ['mission driver', ...names.map((n) => reference.profiles[n].mission_driver)],
          ['estimate', ...names.map((n) => `${reference.profiles[n].estimate.topic} · `
            + `${reference.profiles[n].estimate.frame}`)],
          ['estimate label', ...names.map((n) => reference.profiles[n].estimate.label)],
          ['uncertainty', ...names.map((n) => {
            const u = reference.profiles[n].estimate.uncertainty;
            return u.kind === 'covariance_trace'
              ? `covariance trace, converged < ${u.converged_below}`
              : `${u.kind} on ${u.topic ?? 'no topic'}`;
          })],
          ['command', ...names.map((n) => reference.profiles[n].command.label)],
          ['plant', ...names.map((n) => {
            const p = reference.profiles[n].plant;
            return `${p.max_linear_m_s} m/s · ${p.max_angular_rad_s} rad/s · `
              + `min turn ${p.min_turn_radius_m} m`;
          })],
          ['footprint radius', ...names.map((n) => `${reference.profiles[n].footprint.radius_m} m`)],
          ['goal tolerance', ...names.map((n) => `${reference.profiles[n].goal_tolerance_m} m`)],
          ['kidnap_z_m', ...names.map((n) => String(reference.profiles[n].kidnap_z_m))],
          ['bring-up budget', ...names.map((n) => `${reference.profiles[n].bringup_wall_limit_s} s wall`)],
          ['faults', ...names.map((n) => reference.profiles[n].faults.join(', '))],
        ] as string[][]).map((row, index) => (
          <div
            key={row[0] + index}
            style={{
              display: 'grid', gridTemplateColumns: `220px repeat(${names.length}, 1fr)`,
              gap: 10, padding: '5px 10px', borderBottom: `1px solid ${C.row}`,
              font: `11px ${F.mono}`, background: index === 0 ? C.dim : 'transparent',
            }}
          >
            {row.map((cell, cellIndex) => (
              <span
                key={`${cell}-${cellIndex}`}
                style={{ color: cellIndex === 0 ? C.text4 : C.text, wordBreak: 'break-word' }}
              >
                {cell}
              </span>
            ))}
          </div>
        ))}
      </div>

      <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
        FRAMES
      </span>
      <div style={{ display: 'grid', border: `1px solid ${C.line}`, maxWidth: 1100 }}>
        {reference.frames.map((frame) => (
          <div
            key={frame.frame}
            style={{
              display: 'grid', gridTemplateColumns: '140px 1fr 1.4fr', gap: 10,
              padding: '5px 10px', borderBottom: `1px solid ${C.row}`,
              font: `11px ${F.mono}`,
            }}
          >
            <span style={{ fontWeight: 600 }}>{frame.frame}</span>
            <span>{frame.established_by}</span>
            <span style={{ color: C.text3 }}>{frame.used_for}</span>
          </div>
        ))}
      </div>

      <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
        MAP
      </span>
      <KeyValues
        rows={[
          ['grid', `${reference.map.width} × ${reference.map.height} cells at `
            + `${reference.map.resolution} m · origin [${reference.map.origin.join(', ')}]`],
          ['thresholds', `occupied ${reference.map.occupied_thresh} · free `
            + `${reference.map.free_thresh} · ${reference.map.mode}`],
          ['map_sha256', reference.map.map_sha256],
          ['world_to_map', `{x: ${reference.world_to_map.x}, y: ${reference.world_to_map.y},`
            + ` yaw: ${reference.world_to_map.yaw}} · ${reference.alignment_path}`],
          ['map yaml', reference.map.yaml_path],
        ]}
      />

      <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
        WORLD GEOMETRY
      </span>
      <KeyValues
        rows={[
          ['source', reference.world_geometry.source],
          ['static boxes', String(reference.world_geometry.boxes.length)],
          ...reference.world_geometry.doorways.map((door) => [
            door.label,
            `${door.width_m.toFixed(3)} m at world x ${door.x}, y ${door.y} · between `
              + `${door.between.join(' and ')}`,
          ] as [string, string]),
        ]}
      />

      <span style={{ font: `600 11px ${F.sans}`, letterSpacing: '.08em', color: C.text2 }}>
        SCENARIOS · share/ral_eval/scenarios
      </span>
      <div style={{ display: 'grid', border: `1px solid ${C.line}`, maxWidth: 1100 }}>
        {reference.scenarios.map((scenario) => (
          <div
            key={scenario.file}
            style={{
              display: 'grid', gridTemplateColumns: '240px 80px 1fr 1fr',
              gap: 10, padding: '5px 10px', borderBottom: `1px solid ${C.row}`,
              font: `11px ${F.mono}`,
            }}
          >
            <span>{scenario.experiment_id ?? scenario.file}</span>
            <span style={{ color: C.text4 }}>{scenario.robot ?? '—'}</span>
            <span>
              {scenario.goal
                ? `goal (${scenario.goal.x}, ${scenario.goal.y}, ${scenario.goal.yaw})`
                : 'no goal'}
            </span>
            <span style={{ color: C.text4 }}>
              {scenario.interventions.length
                ? scenario.interventions.map((i) => `${i.type}@${i.at_sec}s`).join(', ')
                : 'no interventions'}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}
