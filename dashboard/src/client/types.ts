// The view model the gateway publishes (DASHBOARD_UI_SPEC.md section 4).
//
// Every `| null` here is load-bearing: null means the robot did not report
// the value, and the UI must say so rather than draw a zero. Nothing in this
// file is optional for convenience.

export interface Pose2D { x: number; y: number; yaw: number }

export interface TruthMsg {
  stamp_sim_s: number;
  frame: string;
  pose: Pose2D;
  twist: { linear_x: number | null; angular_z: number | null; known: boolean };
}

export interface EstimateMsg {
  stamp_sim_s: number;
  frame: string;
  pose: { x: number; y: number; yaw_rad: number };
  /** null where the estimator publishes no pose covariance at all. */
  covariance_trace: number | null;
  label: string;
}

export interface ScanMsg {
  stamp_sim_s: number;
  frame: string;
  angle_min: number;
  angle_increment: number;
  range_min: number;
  range_max: number;
  /** null is a no-return: +inf, NaN or outside the sensor window. */
  ranges: (number | null)[];
}

export interface PlanMsg {
  stamp_sim_s: number;
  frame: string;
  poses: { x: number; y: number }[];
}

export interface CmdMsg {
  stamp_sim_s: number;
  stamp_source: 'received';
  linear_x: number;
  angular_z: number;
}

export interface FeedbackMsg {
  stamp_sim_s: number;
  distance_remaining: number;
  number_of_recoveries: number;
}

export interface WpMsg {
  stamp_sim_s: number;
  stamp_source: 'received';
  /** uint16 METRES, truncating: n means anywhere in [n, n+1). */
  wp_dist: number;
}

export interface MavrosStateMsg {
  stamp_sim_s: number;
  stamp_source: 'received';
  connected: boolean;
  armed: boolean;
  guided: boolean;
  mode: string;
  system_status: number;
}

export interface AlignMsg {
  stamp_sim_s: number;
  parent_frame: string;
  child_frame: string;
  x: number;
  y: number;
  yaw: number;
  residual_m: number;
  residual_yaw_rad: number;
  source: string;
}

export interface FaultEventMsg {
  stamp_sim_s: number;
  id: string;
  kind: string;
  injection_point: string;
  target: string;
  state: 'requested' | 'activated' | 'rejected' | 'cleared';
  detail: string;
}

export interface ActiveMsg { stamp_sim_s: number; active: FaultEventMsg[] }

export interface MissionMsg {
  stamp_sim_s: number;
  experiment_id: string;
  state: string;
  goal: { frame_id: string; x: number; y: number; yaw_rad: number };
  outcome: string;
  detail: string;
}

export interface BumperMsg {
  stamp_sim_s: number;
  stamp_source: 'received';
  count: number;
  pairs: string[];
}

export interface GridMsg {
  frame: string;
  resolution: number;
  width: number;
  height: number;
  origin: Pose2D;
  data_b64: string;
  source?: string;
  map_sha256?: string;
}

export interface BringupRow { topic: string; seen: boolean; at_wall_s: number | null }

export interface StackMsg {
  state: 'idle' | 'starting' | 'ready' | 'failed' | 'stopping';
  robot: string | null;
  world: string | null;
  seed: number;
  spawn: Pose2D | null;
  started_wall_utc: string | null;
  wall_elapsed_s: number | null;
  wall_limit_s: number | null;
  bringup: BringupRow[];
  detail: string;
  log_path: string | null;
  prepared: boolean;
  prepare_record: Record<string, unknown>;
  mission_active: boolean;
  bumper_available: boolean | null;
  bumper_reason: string;
}

export interface RtfMsg { world: string; samples: string; source: string }

export interface HelloMsg {
  world: string;
  repo_root: string;
  ros_domain_id: string;
  source: 'live' | 'mock';
}

// -- reference ---------------------------------------------------------------

export interface CatalogueEntry {
  category: string;
  service: string;
  srv_type: string;
  injection_point: string;
  target_rule: string;
  lifetime: 'keyed_set' | 'singleton' | 'oneshot';
  cleared_by: string | null;
  alters_truth: boolean;
  honesty_note: string;
  param_ranges: Record<string, [number | null, number | null]>;
}

export interface RobotProfile {
  robot: string;
  model: string;
  stack_launch: string;
  spawn: { x: number; y: number; z: number; yaw: number; z_source: string };
  estimate: {
    topic: string; type: string; frame: string; label: string; tf_source: string;
    uncertainty: { kind: string; converged_below?: number; topic?: string };
  };
  command: { kind: string; topic: string; label: string };
  mission_driver: string;
  plant: {
    max_linear_m_s: number; max_angular_rad_s: number; min_turn_radius_m: number;
    source: string;
  };
  footprint: { radius_m: number; source: string };
  goal_tolerance_m: number;
  kidnap_z_m: number;
  bringup_wall_limit_s: number;
  bringup_topics: { topic: string; type: string }[];
  record_topics: string[];
  faults: string[];
  unsupported?: Record<string, string>;
  estimator_label: string;
  mobility_note?: string;
  arrival_estimate_tolerance_m?: number;
  autopilot_parm?: string;
  profile_path: string;
}

export interface WorldBox {
  name: string;
  x: number; y: number; z: number; yaw: number;
  sx: number; sy: number; sz: number;
}

export interface Doorway {
  id: string; label: string; x: number; y: number; width_m: number; between: string[];
}

export interface WorldGeometry {
  source: string;
  boxes: WorldBox[];
  doorways: Doorway[];
}

export interface ScenarioSummary {
  file: string;
  experiment_id: string | null;
  robot: string | null;
  world: string | null;
  seed: number | null;
  timeout_sec: number | null;
  initial_pose: { x: number; y: number; yaw: number } | null;
  goal: { x: number; y: number; yaw: number } | null;
  interventions: {
    id: string; type: string; at_sec: number; params: Record<string, unknown>;
  }[];
}

export interface Reference {
  world: string;
  catalogue: { faults: Record<string, CatalogueEntry> };
  profiles: Record<string, RobotProfile>;
  map: {
    yaml_path: string; resolution: number; width: number; height: number;
    origin: number[]; occupied_thresh: number; free_thresh: number; mode: string;
    map_sha256: string;
  };
  world_to_map: Pose2D;
  alignment_path: string;
  world_geometry: WorldGeometry;
  scenarios: ScenarioSummary[];
  frames: { frame: string; established_by: string; used_for: string }[];
}

// -- runs --------------------------------------------------------------------

export interface MetricValue {
  value: number | string | null;
  unit: string;
  source?: string;
  definition?: string;
  reason: string | null;
}

export interface RunEntry {
  id: string;
  scenario: string;
  robot: string | null;
  world: string | null;
  seed: number | null;
  version: number | null;
  exit: number | null;
  state: string;
  outcome: string;
  detail: string;
  sim: [number | null, number | null];
  wall: [string | null, string | null];
  prov: Record<string, unknown>;
  partial: boolean;
  note: string;
  scorable: boolean;
}

export interface RunDetail extends RunEntry {
  metrics: Record<string, MetricValue> | null;
  meta: Record<string, unknown> | null;
  interventions: Record<string, unknown>[];
  preflight: Record<string, unknown> | null;
  preflight_initial: Record<string, unknown> | null;
  bumper_liveness: Record<string, unknown> | null;
  bag: [string, string, number][] | null;
  bag_duration_s: number | null;
  scenario_yaml: string | null;
  log_tail: string | null;
  bag_record_log_tail: string | null;
  has_plot: boolean;
  rtf_samples?: string;
  goal_map?: number[];
  goal_world?: number[];
  initial_map?: number[];
  initial_world?: number[];
}

export interface FaultResponse {
  fault_id: string;
  success: boolean;
  message: string;
}
