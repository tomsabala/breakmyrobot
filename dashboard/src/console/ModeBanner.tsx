// Copyright 2026 Resilient Autonomy Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * The strip that says this console is NOT driving a robot.
 *
 * On the public site a visitor lands directly in the console, watches a
 * mission play out, and has no reason to assume the robot is a recording. The
 * `REPLAY` badge in the top bar is 48 px of 10 px type next to nine other
 * fields; it is not enough, and a reader who mistakes a recording for a live
 * stack has been misled by this project rather than informed by it.
 *
 * Rules this component follows:
 *
 *   * It renders for `replay` and for `mock` and NEVER for `live`. A live
 *     console must look exactly as it did before this file existed.
 *   * It can be collapsed to one line, never dismissed: there is no state in
 *     which the console shows no marking at all. The collapsed form still
 *     carries the word and the run id.
 *   * Every fact in it comes from the fixture (`source.replay.info()`) or is
 *     true by construction of the mode. Nothing here is computed or guessed.
 *   * Its colour is NOT one of the five provenance colours (red truth, blue
 *     estimate, green plan, orange command, magenta fault): those encode data
 *     everywhere else in this UI, and a chrome element wearing one of them
 *     would be a lie about what it is. It uses the warning amber the rest of
 *     the console already reserves for "read this before you believe the
 *     panel underneath".
 */

import { useState } from 'react';
import { C, F } from '../theme/tokens';
import type { RalSource } from '../client/source';

const STORAGE_KEY = 'ral-mode-banner-collapsed';

function storedCollapsed(): boolean {
  try {
    return localStorage.getItem(STORAGE_KEY) === '1';
  } catch {
    return false;
  }
}

export function ModeBanner({ source }: { source: RalSource }) {
  const [collapsed, setCollapsed] = useState(storedCollapsed);

  if (source.kind === 'live') return null;

  const info = source.kind === 'replay' ? source.replay?.info() ?? null : null;
  const word = source.kind === 'replay' ? 'REPLAY' : 'MOCK';
  const runId = info?.run_id ?? null;

  const sentence = source.kind === 'replay'
    ? 'You are watching a recording of one real run, played from a static file. '
      + 'There is no robot, no simulator, no autonomy stack and no gateway behind '
      + 'this page, so nothing here can be started, driven or faulted.'
    : 'You are watching a scripted design fixture, not a robot. The motion and the '
      + 'lidar on this page are generated in your browser; only the catalogue, the '
      + 'run metrics and the fault wordings are recordings of a real gateway.';

  const toggle = () => {
    const next = !collapsed;
    setCollapsed(next);
    try {
      localStorage.setItem(STORAGE_KEY, next ? '1' : '0');
    } catch {
      /* storage blocked: the banner simply reopens next load */
    }
  };

  return (
    <div
      role="status"
      aria-label={`${word} mode: this console is not driving a robot`}
      style={{
        display: 'flex',
        alignItems: collapsed ? 'center' : 'flex-start',
        gap: 12,
        padding: collapsed ? '5px 12px' : '10px 14px',
        borderBottom: `1px solid ${C.warnBorder}`,
        background: 'linear-gradient(90deg, #3a2f12, #2a2410 60%, #241f10)',
        color: C.warn,
        font: `12.5px/1.5 ${F.sans}`,
        flexWrap: 'wrap',
      }}
    >
      <span
        style={{
          flex: '0 0 auto',
          padding: '2px 8px',
          borderRadius: 3,
          background: C.warn,
          color: C.ink,
          font: `700 11px/1.4 ${F.mono}`,
          letterSpacing: '0.08em',
        }}
      >
        {word}
      </span>

      {collapsed ? (
        <span style={{ flex: '1 1 20rem', color: C.warn }}>
          {source.kind === 'replay'
            ? 'a recording, not a live robot'
            : 'a scripted fixture, not a robot'}
          {runId ? <span style={{ color: C.text5, font: `11px ${F.mono}` }}>{` · ${runId}`}</span> : null}
        </span>
      ) : (
        <div style={{ flex: '1 1 24rem', minWidth: 0 }}>
          <div style={{ color: C.warn }}>{sentence}</div>
          <div style={{ marginTop: 4, color: C.text5, font: `11px/1.5 ${F.mono}` }}>
            {runId ? <span>{`playing ${runId}`}</span> : <span>no fixture loaded</span>}
            {info?.outcome ? <span>{` · terminal outcome ${info.outcome}`}</span> : null}
            <span>{' · the live console is '}</span>
            <span style={{ color: C.text3 }}>scripts/dashboard.sh</span>
            <span>{' in the repository'}</span>
          </div>
        </div>
      )}

      <button
        type="button"
        onClick={toggle}
        aria-expanded={!collapsed}
        title={collapsed ? 'Show what this mode means' : 'Collapse to one line'}
        style={{
          flex: '0 0 auto',
          marginLeft: 'auto',
          padding: '2px 8px',
          border: `1px solid ${C.warnBorder}`,
          borderRadius: 3,
          background: 'transparent',
          color: C.warn,
          font: `11px ${F.mono}`,
          cursor: 'pointer',
        }}
      >
        {collapsed ? 'what is this?' : 'collapse'}
      </button>
    </div>
  );
}
