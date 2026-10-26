import { useEffect, useMemo, useRef, useState } from 'react';
import { createLiveSource } from './liveSource';
import { createMockSource } from './mockSource';
import { chooseSourceKind } from './source';
import type { RalSource } from './source';
import { createState, ingest } from './store';
import type { UiState } from './store';
import type { Reference } from './types';

export interface Ral {
  source: RalSource;
  reference: Reference | null;
  error: string | null;
  state: UiState | null;
  /** Re-render pulse. The state object is mutated in place at up to 20 Hz;
   *  copying it per message would cost more than the whole console draws. */
  tick: number;
}

const UI_REFRESH_MS = 120;

export function useRal(): Ral {
  const source = useMemo<RalSource>(
    () => (chooseSourceKind() === 'mock' ? createMockSource() : createLiveSource()),
    [],
  );
  const [reference, setReference] = useState<Reference | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [tick, setTick] = useState(0);
  const stateRef = useRef<UiState | null>(null);

  useEffect(() => {
    let cancelled = false;
    source.reference()
      .then((value) => { if (!cancelled) setReference(value); })
      .catch((reason: unknown) => {
        if (!cancelled) setError(`/api/reference: ${String(reason)}`);
      });
    return () => { cancelled = true; };
  }, [source]);

  useEffect(() => {
    if (!reference) return undefined;
    stateRef.current = createState(reference.world_to_map);
    const unsubscribe = source.subscribe((topic, payload) => {
      const state = stateRef.current;
      if (state) ingest(state, topic, payload);
    });
    const timer = window.setInterval(() => setTick((value) => value + 1), UI_REFRESH_MS);
    return () => {
      unsubscribe();
      window.clearInterval(timer);
    };
  }, [source, reference]);

  return { source, reference, error, state: stateRef.current, tick };
}
