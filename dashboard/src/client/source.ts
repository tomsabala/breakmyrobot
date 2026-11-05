import type { Reference, RunDetail, RunEntry } from './types';

/**
 * Where the console's data comes from.
 *
 * Three implementations: `liveSource` (the gateway over SSE + POST),
 * `mockSource` (the design fixture, for UI work with no stack) and
 * `replaySource` (a recording of a REAL run, exported from its rosbag and
 * served as a static file). They publish the SAME view model, which is the
 * only reason a mock or a recording is admissible at all: the console cannot
 * tell them apart, so it cannot grow a code path that only works against
 * made-up or canned data. `kind` exists so the UI can SAY which one it is
 * showing - and it must, because a recording that looked live would be a lie
 * about what is running - and so the replay controls, which only a fixture
 * can honour, appear only where they mean something.
 */
export interface RalSource {
  readonly kind: 'live' | 'mock' | 'replay';
  subscribe(fn: (topic: string, payload: unknown) => void): () => void;
  post<T = unknown>(path: string, body?: unknown): Promise<T>;
  reference(): Promise<Reference>;
  runs(): Promise<RunEntry[]>;
  run(id: string): Promise<RunDetail>;
  mapImage(): Promise<unknown>;
  /** Mock and replay only: a transport whose playhead can be moved. Absent
   *  on a live stack, which has no rewind. */
  replay?: ReplayControls;
}

/**
 * The synthetic topic a seekable source emits to say "discard the UiState
 * and start an empty one".
 *
 * `store.ingest` is a left fold that only ever appends - trails, missions,
 * fault events, pairs - so the state at sim time T is DEFINED as the fold of
 * every frame up to T and cannot be un-applied. A backwards scrub therefore
 * resets and re-folds instead of subtracting. The name starts with `$` so it
 * can never collide with a gateway SSE topic, and `useRal` consumes it before
 * `ingest` ever sees it.
 */
export const RESET_TOPIC = '$reset';

export interface ReplayControls {
  /** The pickable recordings (replay) or scripted scenarios (mock). */
  scenarios(): { value: string; label: string; disabled: boolean }[];
  scenario(): string;
  pick(scenario: string): void;
  restart(): void;
  /** Sim time of the run's `goal_accepted`, or null when it never happened
   *  - in which case the console must not offer to seek to it. */
  acceptTime(): number | null;
  seekAccept(): void;
  playing(): boolean;
  togglePlay(): void;
  step(seconds: number): void;
  /** Move the playhead to an absolute sim time, forwards or backwards. */
  seek(t: number): void;
  now(): number;
  /** The scrubbable sim-time window. `end <= start` means the source has no
   *  known end (the mock generates indefinitely), so no scrub bar. */
  span(): { start: number; end: number };
  speeds(): number[];
  speed(): number;
  setSpeed(speed: number): void;
  closed(): boolean;
  /** The robot the playhead belongs to, so the console picks that profile
   *  rather than the operator's pre-launch selection. */
  robot(): string;
  /** The recording's identity and provenance, or null for the mock, which is
   *  not a recording of anything. */
  info(): ReplayInfo | null;
}

/** Identity of a recorded run, read from the fixture the exporter wrote. */
export interface ReplayInfo {
  run_id: string;
  robot: string;
  world: string;
  outcome: string;
  headline: string;
  provenance: {
    git_sha?: string;
    git_dirty?: boolean;
    image_ref?: string;
    map_sha256?: string;
    exported_utc?: string;
    bag?: string;
    /** Where the occupancy grid came from when the bag carried no /map. */
    grid_source?: string;
    /** Per topic: 'read' or 'absent'. An absent topic is a hole in the
     *  recording, not a quiet stream, and the console must not read it as
     *  one. */
    topic_status?: Record<string, string>;
    decimation?: Record<string, string>;
  };
  t_start: number;
  t_end: number;
  frames: number;
  /** True when the fixture carried no `reference` and the console fell back
   *  to the committed design fixture - which may not match this run. */
  reference_fallback: boolean;
}

export function chooseSourceKind(): 'live' | 'mock' | 'replay' {
  const query = new URLSearchParams(window.location.search).get('source');
  if (query === 'mock' || query === 'live' || query === 'replay') return query;
  const configured = import.meta.env.VITE_RAL_SOURCE;
  if (configured === 'mock' || configured === 'replay') return configured;
  return 'live';
}
