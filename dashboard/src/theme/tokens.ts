// Every colour and font the design uses, named once.
//
// The provenance colours are NOT decoration: red is simulator truth and is
// never used for anything the robot knows, blue is the robot's own estimate,
// orange is an operator command, magenta is a fault, grey-white is a measured
// value and grey is derived. A reader who learns the five once can read any
// panel, and reusing one of them for a different meaning would quietly lie.

export const C = {
  // provenance
  truth: '#FF0000',
  estimate: '#00AAFF',
  estimateOdom: '#0064C8',
  plan: '#00FF00',
  command: '#FF8C00',
  scan: '#FFFF00',
  fault: '#CC33CC',
  faultRequested: '#888888',
  faultRejected: '#d04040',
  faultCleared: '#00aa00',

  // surfaces, dark to light
  bg: '#181818',
  panel: '#1c1c1c',
  drawer: '#1d1d1d',
  inset: '#1e1e1e',
  side: '#202020',
  dim: '#222',
  tab: '#242424',
  row: '#262626',
  rowAlt: '#2a2a2a',
  header: '#2c2c2c',
  canvas: '#303030',
  sunken: '#1a1a1a',
  sunkenAlt: '#1b1b1b',
  tip: '#2b2b2b',
  warnBg: '#2e2e2e',
  free: '#232323',

  // borders
  line: '#333',
  lineSoft: '#3a3a3a',
  lineMid: '#444',
  lineBright: '#4a4a4a',
  border: '#555',
  borderAlt: '#5a5a5a',
  borderHi: '#6a6a6a',
  borderMax: '#777',

  // text
  text: '#e6e6e6',
  text2: '#cfcfcf',
  text3: '#bdbdbd',
  text4: '#a8a8a8',
  text5: '#8a8a8a',
  text6: '#7d7d7d',
  textBright: '#dcdcdc',

  // accents
  link: '#8fd0ff',
  warn: '#f0d08a',
  warnBorder: '#6a5a30',
  bad: '#f0b8b8',
  good: '#9ad09a',
  faultText: '#e8a6e8',
  obstacleText: '#ffb3b3',
  draftText: '#FFB466',
  ink: '#141414',
} as const;

export const F = {
  sans: "'IBM Plex Sans', system-ui, sans-serif",
  mono: "'IBM Plex Mono', monospace",
} as const;

export const BASE_FONT_SIZE = '12.5px';
export const BASE_LINE_HEIGHT = 1.4;

/** The four fault-event states, with the glyph and colour the design gives them. */
export const EVENT_MARK: Record<string, [string, string]> = {
  requested: ['○', C.faultRequested],
  activated: ['▲', C.fault],
  rejected: ['✕', C.faultRejected],
  cleared: ['■', C.faultCleared],
};

/** MissionState's nine states, in the two rows the chip strip shows. */
export const STATES_A = ['idle', 'goal_accepted', 'navigating'];
export const STATES_B = ['succeeded', 'aborted', 'canceled', 'unreachable', 'rejected',
  'unknown'];
export const TERMINAL_STATES = new Set(STATES_B);
