import type { Screen } from '../App';
import type { RalSource } from '../client/source';
import type { UiState } from '../client/store';
import type {
  FaultResponse, Reference, RobotProfile, ScenarioSummary,
} from '../client/types';
import type { Layers } from './draw';
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

  pick: 'obs' | 'kid' | 'goal' | null;
  setPick: (pick: 'obs' | 'kid' | 'goal' | null) => void;
  selectedObstacle: string | null;
  setSelectedObstacle: (name: string | null) => void;

  goal: GoalForm;
  setGoal: (update: (goal: GoalForm) => GoalForm) => void;

  timeMode: 'abs' | 'rel';
  setTimeMode: (mode: 'abs' | 'rel') => void;
  selectedMarker: { type: 'm' | 'e'; index: number } | null;
  setSelectedMarker: (marker: { type: 'm' | 'e'; index: number } | null) => void;

  startRobot: string;
  setStartRobot: (robot: string) => void;
  startStack: (robot?: string) => void;
  stopStack: () => void;
  startMission: () => void;
  cancelMission: () => void;
  lifecycle: RequestRecord | null;

  blockReason: string;
}
