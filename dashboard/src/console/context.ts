import type { MutableRefObject } from 'react';
import type { Screen } from '../App';
import type { RalSource } from '../client/source';
import type { UiState } from '../client/store';
import type {
  FaultResponse, Reference, RobotProfile, ScenarioSummary,
} from '../client/types';
import type { Camera, Layers, View } from './draw';
import type { Forms, SheetKey } from './faults';

/** One outstanding or finished operator request, as the console shows it. */
export interface RequestRecord {
  key: string;
  kind: string;
  service: string;
  summary: string;
  sentSim: number;
  status: 'pending' | 'resolved' | 'failed';
  response?: FaultResponse;
  atSim?: number;
  error?: string;
}

export interface ArmedRequest {
  slot: string;
  sheet: SheetKey;
  kind: string;
  params: Record<string, unknown>;
  summary: string;
}

export interface OpenSections {
  mission: boolean;
  mdetail: boolean;
  setup: boolean;
  div: boolean;
  params: boolean;
  hon: boolean;
  timeline: boolean;
  rows: Record<string, boolean>;
}

export interface GoalForm { x: string; y: string; yaw: string; timeout: string }

/** The SPAWN pose a stack will be started at. Consumed at launch: the gateway
 *  forwards it to `ros2 launch` as spawn_x/spawn_y/spawn_yaw, so it cannot
 *  change once the stack is up - a running body is moved by `reposition`. */
export interface StartForm { x: string; y: string; yaw: string }

/** Where a map click goes. `start` is the pre-launch spawn pose, `repos` the
 *  live reposition destination. `repos` is deliberately NOT `kid`: the banner
 *  text and the dispatched kind differ, and a kidnap is not a reposition. */
export type PickKind = 'obs' | 'kid' | 'goal' | 'start' | 'repos';

export interface ConsoleCtx {
  source: RalSource;
  reference: Reference;
  state: UiState;
  go: (screen: Screen) => void;

  /** The profile of the robot a stack is up for, or of the one selected to start. */
  profile: RobotProfile;
  robot: string;
  isCar: boolean;
  scenarios: ScenarioSummary[];

  pop: string | null;
  setPop: (pop: string | null) => void;
  layers: Layers;
  toggleLayer: (key: keyof Layers) => void;
  /** Operator view control. A REF: the world view redraws every frame anyway,
   *  so a camera change must not re-render the console. */
  camera: MutableRefObject<Camera>;
  resetCamera: () => void;
  /** Written by WorldView's render loop each frame; the single copy each. */
  viewRef: MutableRefObject<View | null>;
  canvasRef: MutableRefObject<HTMLCanvasElement | null>;
  zoomKeyboard: (factor: number) => void;
  drawerOpen: boolean;
  setDrawerOpen: (open: boolean) => void;
  tab: 'inject' | 'sensing' | 'metrics';
  setTab: (tab: 'inject' | 'sensing' | 'metrics') => void;
  sheet: SheetKey;
  setSheet: (sheet: SheetKey) => void;

  open: OpenSections;
  toggle: (key: keyof OpenSections) => void;
  toggleRow: (id: string) => void;

  forms: Forms;
  setForms: (update: (forms: Forms) => Forms) => void;
  errors: Record<string, string>;

  armed: ArmedRequest[];
  arm: () => void;
  fire: (index: number) => void;
  disarm: (index: number) => void;

  requests: RequestRecord[];
  cardFor: (key: string) => RequestRecord | undefined;
  pending: (key: string) => boolean;
  sendFault: (key: string, kind: string, params: Record<string, unknown>,
    summary: string) => void;

  confirm: string | null;
  twoStep: (name: string, run: () => void) => void;

  pick: PickKind | null;
  setPick: (pick: PickKind | null) => void;
  selectedObstacle: string | null;
  setSelectedObstacle: (name: string | null) => void;

  goal: GoalForm;
  setGoal: (update: (goal: GoalForm) => GoalForm) => void;

  start: StartForm;
  setStart: (update: (start: StartForm) => StartForm) => void;
  /** The live reposition destination, world frame. Separate from `start`:
   *  one is consumed at launch, the other moves a body that is already up. */
  repos: StartForm;
  setRepos: (update: (repos: StartForm) => StartForm) => void;
  /** Dispatch a live reposition of the body (world frame). */
  sendReposition: (x: number, y: number, yaw: number) => void;

  timeMode: 'abs' | 'rel';
  setTimeMode: (mode: 'abs' | 'rel') => void;
  selectedMarker: { type: 'm' | 'e'; index: number } | null;
  setSelectedMarker: (marker: { type: 'm' | 'e'; index: number } | null) => void;

  startRobot: string;
  setStartRobot: (robot: string) => void;
  startStack: (robot?: string) => void;
  /** `force: true` cancels a running mission first; see Gateway.stop_stack. */
  stopStack: (force?: boolean) => void;
  /** Stop, WAIT for the gateway's answer, then start `robot`. */
  stopThenStart: (robot: string) => void;
  /** Whether the next mission records a bag and computes metrics. */
  recordBag: boolean;
  setRecordBag: (value: boolean) => void;
  startMission: () => void;
  cancelMission: () => void;
  lifecycle: RequestRecord | null;

  blockReason: string;
}
