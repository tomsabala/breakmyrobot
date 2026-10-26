import type { RalSource } from './source';
import type { Reference, RunDetail, RunEntry } from './types';

/**
 * The gateway, over SSE for telemetry and POST for operator actions.
 *
 * `link` is synthesised HERE and nowhere else: it is the browser's own view
 * of the EventSource, not a robotics signal, and the console labels it
 * "dashboard link" for exactly that reason. When it drops, the console marks
 * a gap; it never interpolates across one and never ages a value silently.
 */
export function createLiveSource(): RalSource {
  const listeners = new Set<(topic: string, payload: unknown) => void>();
  let stream: EventSource | null = null;
  let linkUp = false;

  const emit = (topic: string, payload: unknown) => {
    for (const fn of listeners) fn(topic, payload);
  };

  const connect = () => {
    stream = new EventSource('/api/stream');
    stream.onopen = () => {
      linkUp = true;
      emit('link', { up: true, at_wall_ms: Date.now() });
    };
    stream.onmessage = (event) => {
      try {
        const frame: unknown = JSON.parse(event.data);
        if (frame && typeof frame === 'object' && 'topic' in frame
            && typeof frame.topic === 'string' && 'payload' in frame) {
          emit(frame.topic, frame.payload);
        } else {
          emit('link', { up: linkUp, parse_error: `frame without topic: ${event.data}` });
        }
      } catch (error) {
        emit('link', { up: linkUp, parse_error: String(error) });
      }
    };
    stream.onerror = () => {
      if (linkUp) {
        linkUp = false;
        emit('link', { up: false, at_wall_ms: Date.now() });
      }
      // EventSource reconnects on its own; the gateway replays the latched
      // topics on the new connection, so a reconnect repopulates the UI.
    };
  };

  const json = async <T>(path: string, init?: RequestInit): Promise<T> => {
    const response = await fetch(path, init);
    const text = await response.text();
    let parsed: unknown;
    try {
      parsed = JSON.parse(text);
    } catch {
      throw new Error(`${path}: ${response.status} ${text.slice(0, 400)}`);
    }
    if (!response.ok) {
      // The gateway answers every refusal with {message}; narrow rather than
      // assert, so a differently shaped 500 still reports its body.
      let message = text;
      if (parsed && typeof parsed === 'object' && 'message' in parsed
          && typeof parsed.message === 'string') {
        message = parsed.message;
      }
      throw new Error(`${path}: ${response.status} ${message}`);
    }
    // The gateway is the only writer of these payloads and each route's shape
    // is pinned by ral_eval/test/test_dashboard_api.py; T names that contract.
    return parsed as T;
  };

  return {
    kind: 'live',
    subscribe(fn) {
      listeners.add(fn);
      if (!stream) connect();
      return () => {
        listeners.delete(fn);
        if (listeners.size === 0 && stream) {
          stream.close();
          stream = null;
          linkUp = false;
        }
      };
    },
    post<T>(path: string, body?: unknown) {
      return json<T>(path, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body ?? {}),
      });
    },
    reference: () => json<Reference>('/api/reference'),
    runs: () => json<RunEntry[]>('/api/runs'),
    run: (id: string) => json<RunDetail>(`/api/runs/${encodeURIComponent(id)}`),
    mapImage: () => json('/api/map'),
  };
}
