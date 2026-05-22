export type Point = {
  x: number;
  y: number;
};

export type Pose = {
  x_m: number;
  y_m: number;
  yaw_rad: number;
};

export type ManualConfig = {
  maxSpeed: number;
  accel: number;
  leftKey: string;
  rightKey: string;
  wsUrl: string;
  scene: {
    field: {
      widthM: number;
      heightM: number;
    };
    anchors: Record<string, Point>;
    referencePath: Point[];
    officialTrack: Point[][];
    sensorLocalPositions: Point[];
  };
};

export type ObservationPayload = {
  type: "observation";
  sequence_id: number;
  sim_time_s: number;
  pose: Pose;
  line_sensor_darkness: number[];
  sensor_world_positions: Point[];
  yaw_deg: number;
  yaw_rate_deg_s: number;
  progress_m: number;
  remaining_distance_m: number;
  completed_events: string[];
  reached_goal: boolean;
  cross_track_error_m: number;
  heading_error_rad: number;
  longitudinal_velocity_m_s: number;
  yaw_rate_rad_s: number;
};

export type WheelTargets = {
  left: number;
  right: number;
};
