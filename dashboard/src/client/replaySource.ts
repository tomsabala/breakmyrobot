// Copyright 2026 Resilient Autonomy Lab contributors
// SPDX-License-Identifier: Apache-2.0

import referenceFixture from '../fixtures/reference.json';
import { RESET_TOPIC } from './source';
import type { RalSource, ReplayControls, ReplayInfo } from './source';
import type { MissionMsg, Reference, RunDetail, RunEntry } from './types';

/**
 * A RECORDING of a real run, played back in the console.
 *
 * There is no container, no gz, no ROS graph and no autonomy stack behind
 * this source: it reads a fixture exported from a run's rosbag by
 * ral_eval's replay exporter and re-emits that run's gateway frames, on the
 * same topic names and with the same payloads the live gateway published, so
 * `store.ingest` cannot tell the difference and the console therefore cannot
 * grow a code path that only works against a recording.
 *
 * What is REAL here: every frame was produced by the real stack during the
 * recorded run, and `metrics` is that run's metrics.json verbatim.
 * What is NOT: nothing can be commanded. `post()` refuses every control path
 * instead of reporting a success that did not happen, and the console labels
 * itself REPLAY rather than LIVE.
 *
 * Fixtures are fetched at runtime from the served root (`replay/index.json`,
 * resolved against document.baseURI so the bundle works under a subdirectory)
 * and are never imported into the bundle.
 */

export interface ReplayManifestEntry {
  run_id: string;
  robot: string;
  outcome: string;
  headline: string;
  file: string;
  bytes: number;
  t_start: number;
  t_end: number;
}

interface ReplayFrame { t: number; topic: string; payload: unknown }

interface ReplayFixture {
  replay_version: number;
  run_id: string;
  robot: string;
  world: string;
  outcome: string;
  headline: string;
  provenance: ReplayInfo['provenance'];
  t_start: number;
  t_end: number;
  reference: Reference | null;
  metrics: RunDetail['metrics'];
  frames: ReplayFrame[];
}

const fallbackReference = referenceFixture as unknown as Reference;

/** Speeds the bar offers. Sim time per wall second; 1 = the recorded rate. */
const SPEEDS = [0.5, 1, 2, 4];

/** Pacing guard: at most this many frames leave the fixture per animation
 *  frame, so a 4x scrub over a dense stretch cannot freeze the tab. The
 *  cursor is not skipped - the surplus is emitted on the next tick. */
const MAX_FRAMES_PER_TICK = 4000;

type Publish = (topic: string, payload: unknown) => void;

function assetUrl(path: string): string {
  return new URL(path, document.baseURI).toString();
}

async function fetchJson<T>(path: string): Promise<T> {
  const url = assetUrl(path);
  let response: Response;
  try {
    response = await fetch(url);
  } catch (reason) {
    throw new Error(`${url} could not be fetched (${String(reason)}). The replay console `
      + 'serves its fixtures as static files; for a live stack run scripts/dashboard.sh.');
  }
  if (!response.ok) {
    throw new Error(`${url}: HTTP ${response.status}. No replay fixture is published at `
      + 'that path.');
  }
  const text = await response.text();
  try {
    return JSON.parse(text) as T;
  } catch {
    throw new Error(`${url}: not JSON (${text.slice(0, 200)})`);
  }
}

export function createReplaySource(): RalSource {
  const listeners = new Set<Publish>();
  const emit: Publish = (topic, payload) => {
    for (const fn of listeners) fn(topic, payload);
  };

  let manifest: ReplayManifestEntry[] = [];
  let fixture: ReplayFixture | null = null;
  let usedFallbackReference = false;

  // Playback state. `cursor` is the index of the next unemitted frame and
  // `simT` the sim time played up to; together they ARE the playhead, and
  // (cursor, simT) is always consistent with "every frame with t <= simT has
  // been emitted exactly once".
  let cursor = 0;
  let simT = 0;
  let playing = true;
  let speed = 1;
  let frameHandle = 0;
  let lastWallMs = 0;
  let running = false;

  const load = async (runId?: string): Promise<ReplayFixture> => {
    if (manifest.length === 0) {
      const index = await fetchJson<{ replays: ReplayManifestEntry[] }>('replay/index.json');
      manifest = index.replays ?? [];
      if (manifest.length === 0) {
        throw new Error(`${assetUrl('replay/index.json')} lists no replays.`);
      }
    }
    const wanted = runId
      ?? new URLSearchParams(window.location.search).get('run')
      ?? manifest[0].run_id;
    const entry = manifest.find((item) => item.run_id === wanted);
    if (!entry) {
      throw new Error(`?run=${wanted} is not in replay/index.json; it lists `
        + `${manifest.map((item) => item.run_id).join(', ')}.`);
    }
    const loaded = await fetchJson<ReplayFixture>(`replay/${entry.file}`);
    usedFallbackReference = !loaded.reference;
    fixture = loaded;
    cursor = 0;
    simT = loaded.t_start;
    return loaded;
  };

  /**
   * Replay the fixture prefix [t_start, target] into a FRESH store.
   *
   * This is how a backwards scrub stays correct. `store.ingest` is a left
   * fold over the ordered frame sequence - it appends trails, missions,
   * fault events and pairs and never removes any - so "the state at sim time
   * T" is defined only as "fold of every frame with t <= T". Rewinding by
   * subtracting is therefore impossible without a second, divergent copy of
   * the store's semantics. Instead the source emits RESET_TOPIC, which makes
   * useRal throw the UiState away and build an empty one, and then re-folds
   * the prefix. The result is byte-for-byte the state a continuous play to T
   * would have produced, because it IS that fold, run again.
   *
   * A forwards seek needs no reset: continuing the fold from the current
   * cursor is already that same prefix.
   */
  const seekTo = (target: number) => {
    const run = fixture;
    if (!run) return;
    const clamped = Math.max(run.t_start, Math.min(run.t_end, target));
    if (clamped < simT) {
      emit(RESET_TOPIC, null);
      cursor = 0;
      simT = run.t_start;
      hello();
    }
    while (cursor < run.frames.length && run.frames[cursor].t <= clamped) {
      const frame = run.frames[cursor];
      cursor += 1;
      emit(frame.topic, frame.payload);
    }
    simT = clamped;
  };

  function hello() {
    const run = fixture;
    emit('hello', {
      world: run ? run.world : fallbackReference.world,
      repo_root: run?.provenance.git_sha ?? 'replay',
      ros_domain_id: 'replay (no ROS graph)',
      source: 'replay',
    });
    // `link` is the console's own "am I receiving frames" signal, synthesised
    // by every source. For a recording it is up for as long as the fixture
    // has frames left; the top bar says FILE, not connected, so nobody reads
    // it as an open socket to a gateway.
    emit('link', { up: true });
  }

  const ended = () => !!fixture && cursor >= fixture.frames.length;

  const tick = () => {
    frameHandle = window.requestAnimationFrame(tick);
    const now = performance.now();
    const dtWall = Math.min(0.25, (now - lastWallMs) / 1000);
    lastWallMs = now;
    const run = fixture;
    if (!run || !playing || ended()) return;
    simT = Math.min(run.t_end, simT + dtWall * speed);
    let budget = MAX_FRAMES_PER_TICK;
    while (cursor < run.frames.length && run.frames[cursor].t <= simT && budget > 0) {
      const frame = run.frames[cursor];
      cursor += 1;
      budget -= 1;
      emit(frame.topic, frame.payload);
    }
  };

  const start = () => {
    if (running) return;
    running = true;
    lastWallMs = performance.now();
    hello();
    frameHandle = window.requestAnimationFrame(tick);
  };

  const acceptTime = (): number | null => {
    const run = fixture;
    if (!run) return null;
    // The same rule store.ingestMission uses for tAccept: the first mission
    // frame the recorded gateway published with state 'goal_accepted'.
    const found = run.frames.find((frame) => frame.topic === 'mission'
      && (frame.payload as MissionMsg | null)?.state === 'goal_accepted');
    return found ? found.t : null;
  };

  const info = (): ReplayInfo | null => (fixture ? {
    run_id: fixture.run_id,
    robot: fixture.robot,
    world: fixture.world,
    outcome: fixture.outcome,
    headline: fixture.headline,
    provenance: fixture.provenance,
    t_start: fixture.t_start,
    t_end: fixture.t_end,
    frames: fixture.frames.length,
    reference_fallback: usedFallbackReference,
  } : null);

  const replay: ReplayControls = {
    scenarios: () => manifest.map((entry) => ({
      value: entry.run_id,
      label: `${entry.run_id} · ${entry.outcome}`,
      disabled: false,
    })),
    scenario: () => fixture?.run_id ?? '',
    pick: (value) => {
      if (!fixture || value === fixture.run_id) return;
      emit(RESET_TOPIC, null);
      fixture = null;
      // Keep the URL pointing at what is on screen, so a shared link names
      // the run it shows instead of silently reopening the first one.
      const url = new URL(window.location.href);
      url.searchParams.set('run', value);
      window.history.replaceState(null, '', url.toString());
      void load(value).then(() => {
        hello();
        playing = true;
      }).catch(() => {
        // The index named this run, so a failure here is a publishing bug.
        // Say so on a topic the console already shows rather than silently
        // leaving the playhead on nothing.
        emit('link', { up: false });
      });
    },
    restart: () => { seekTo(Number.NEGATIVE_INFINITY); },
    acceptTime,
    seekAccept: () => {
      const t = acceptTime();
      if (t !== null) seekTo(t);
    },
    playing: () => playing,
    togglePlay: () => { playing = !playing; },
    step: (seconds) => { seekTo(simT + seconds); },
    seek: (t) => { seekTo(t); },
    now: () => simT,
    span: () => (fixture
      ? { start: fixture.t_start, end: fixture.t_end }
      : { start: 0, end: 0 }),
    speeds: () => SPEEDS,
    speed: () => speed,
    setSpeed: (value) => { speed = value; },
    closed: () => ended(),
    robot: () => fixture?.robot ?? 'roomba',
    info,
  };

  return {
    kind: 'replay',
    subscribe(fn) {
      listeners.add(fn);
      if (listeners.size === 1) start();
      return () => {
        listeners.delete(fn);
        if (listeners.size === 0) {
          window.cancelAnimationFrame(frameHandle);
          running = false;
        }
      };
    },
    async post<T>(path: string): Promise<T> {
      // Every control path, including /api/faults/*: a recording cannot be
      // injected into. Refusing is the honest answer; `accepted: false` is
      // what the console renders as a failed request.
      return {
        accepted: false,
        message: `${path} is unavailable in replay mode. This console is a RECORDING of `
          + `a real run (${fixture?.run_id ?? 'replay fixture'}): there is no container, `
          + 'no gz and no autonomy stack behind it, so nothing can be started, stopped, '
          + 'driven or faulted, and no action was taken. Run scripts/dashboard.sh for '
          + 'the live console.',
      } as unknown as T;
    },
    reference: async () => {
      const run = fixture ?? await load();
      return run.reference ?? fallbackReference;
    },
    runs: async () => {
      const run = fixture ?? await load();
      return [runEntry(run)];
    },
    run: async (id: string) => {
      const run = fixture ?? await load();
      if (id !== run.run_id) {
        throw new Error(`${id} is not in this replay fixture; it carries ${run.run_id} `
          + 'only. The runs index lives on the gateway (scripts/dashboard.sh).');
      }
      return runDetail(run);
    },
    mapImage: async () => null,
    replay,
  };
}

/**
 * The fixture as a runs-index row.
 *
 * Only the fields the exporter actually carries are filled in. Everything the
 * fixture does not record - seed, exit code, wall clock, scenario file - is
 * `null`, not a guess: the runs index is a gateway artifact and this is not it.
 */
function runEntry(run: ReplayFixture): RunEntry {
  return {
    id: run.run_id,
    scenario: '',
    robot: run.robot,
    world: run.world,
    seed: null,
    version: null,
    exit: null,
    state: 'replay',
    outcome: run.outcome,
    detail: run.headline,
    sim: [run.t_start, run.t_end],
    wall: [null, null],
    prov: run.provenance as unknown as Record<string, unknown>,
    partial: false,
    note: 'Read from a committed replay fixture, not from the gateway\'s runs index. '
      + 'Fields the fixture does not carry are null rather than inferred.',
    scorable: false,
  };
}

function runDetail(run: ReplayFixture): RunDetail {
  return {
    ...runEntry(run),
    metrics: run.metrics,
    meta: run.provenance as unknown as Record<string, unknown>,
    interventions: [],
    preflight: null,
    preflight_initial: null,
    bumper_liveness: null,
    bag: null,
    bag_duration_s: null,
    scenario_yaml: null,
    log_tail: null,
    bag_record_log_tail: null,
    has_plot: false,
  };
}
