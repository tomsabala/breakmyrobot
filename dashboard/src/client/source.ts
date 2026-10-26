import type { Reference, RunDetail, RunEntry } from './types';

/**
 * Where the console's data comes from.
 *
 * Two implementations: `liveSource` (the gateway over SSE + POST) and
 * `mockSource` (the design fixture, for UI work with no stack). They publish
 * the SAME view model, which is the only reason a mock is admissible at all:
 * the console cannot tell them apart, so it cannot grow a code path that only
 * works against made-up data. `kind` exists so the UI can SAY which one it is
 * showing, and so the replay controls - which only a fixture can honour -
 * appear only in mock mode.
 */
export interface RalSource {
  readonly kind: 'live' | 'mock';
  subscribe(fn: (topic: string, payload: unknown) => void): () => void;
  post<T = unknown>(path: string, body?: unknown): Promise<T>;
  reference(): Promise<Reference>;
  runs(): Promise<RunEntry[]>;
  run(id: string): Promise<RunDetail>;
  mapImage(): Promise<unknown>;
  /** Mock only: the scripted replay transport. Absent on a live stack. */
  replay?: ReplayControls;
}

export interface ReplayControls {
  scenarios(): { value: string; label: string; disabled: boolean }[];
  scenario(): string;
  pick(scenario: string): void;
  restart(): void;
  seekAccept(): void;
  playing(): boolean;
  togglePlay(): void;
  step(seconds: number): void;
  speed(): number;
  setSpeed(speed: number): void;
  closed(): boolean;
  setRobot(robot: string): void;
}

export function chooseSourceKind(): 'live' | 'mock' {
  const query = new URLSearchParams(window.location.search).get('source');
  if (query === 'mock' || query === 'live') return query;
  return import.meta.env.VITE_RAL_SOURCE === 'mock' ? 'mock' : 'live';
}
