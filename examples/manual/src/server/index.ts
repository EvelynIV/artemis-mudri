import http from "node:http";
import path from "node:path";
import { fileURLToPath } from "node:url";
import process from "node:process";
import dotenv from "dotenv";
import grpc from "@grpc/grpc-js";
import protoLoader from "@grpc/proto-loader";
import { WebSocketServer, type WebSocket } from "ws";
import { nextWheelTargets } from "../shared/control";
import { ANCHORS, buildOfficialTrack, buildReferencePath, FIELD_HEIGHT_M, FIELD_WIDTH_M, SENSOR_LOCAL_POSITIONS } from "../shared/scene";
import type { ManualConfig, ObservationPayload, Point } from "../shared/types";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const REPO_ROOT = path.resolve(__dirname, "../../../..");

dotenv.config({ path: path.join(REPO_ROOT, ".env") });
dotenv.config({ path: path.resolve(__dirname, "../../.env") });

const PROTO_ROOT = path.join(REPO_ROOT, "src");
const VEHICLE_PROTO = path.join(PROTO_ROOT, "artemis_mudri/protos/simulation/v1/vehicle_simulation.proto");

type RuntimeConfig = {
  target: string;
  webHost: string;
  webPort: number;
  maxSpeed: number;
  accel: number;
  controlPeriodS: number;
  maxTimeS: number;
  leftKey: string;
  rightKey: string;
};

type ControlState = {
  leftSpeed: number;
  rightSpeed: number;
  leftKeyPressed: boolean;
  rightKeyPressed: boolean;
};

function envNumber(name: string, defaultValue: number) {
  const value = process.env[name];
  if (!value) return defaultValue;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : defaultValue;
}

function runtimeConfig(): RuntimeConfig {
  return {
    target: process.env.ARTEMIS_MANUAL_TARGET ?? "127.0.0.1:50051",
    webHost: process.env.ARTEMIS_MANUAL_WEB_HOST ?? "127.0.0.1",
    webPort: envNumber("ARTEMIS_MANUAL_WEB_PORT", 8765),
    maxSpeed: envNumber("ARTEMIS_MANUAL_MAX_SPEED", 20),
    accel: envNumber("ARTEMIS_MANUAL_ACCEL", 80),
    controlPeriodS: envNumber("ARTEMIS_MANUAL_CONTROL_PERIOD", 0.02),
    maxTimeS: envNumber("ARTEMIS_MANUAL_MAX_TIME", 120),
    leftKey: (process.env.ARTEMIS_MANUAL_LEFT_KEY ?? "j").trim().toLowerCase(),
    rightKey: (process.env.ARTEMIS_MANUAL_RIGHT_KEY ?? "l").trim().toLowerCase()
  };
}

function buildFrontendConfig(config: RuntimeConfig): ManualConfig {
  return {
    maxSpeed: config.maxSpeed,
    accel: config.accel,
    leftKey: config.leftKey,
    rightKey: config.rightKey,
    wsUrl: `ws://${config.webHost}:${config.webPort}/ws`,
    scene: {
      field: {
        widthM: FIELD_WIDTH_M,
        heightM: FIELD_HEIGHT_M
      },
      anchors: ANCHORS,
      referencePath: buildReferencePath(),
      officialTrack: buildOfficialTrack(),
      sensorLocalPositions: SENSOR_LOCAL_POSITIONS
    }
  };
}

function sensorWorldPositions(pose: { x_m?: number; y_m?: number; yaw_rad?: number }): Point[] {
  const x = Number(pose.x_m ?? 0);
  const y = Number(pose.y_m ?? 0);
  const yaw = Number(pose.yaw_rad ?? 0);
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  return SENSOR_LOCAL_POSITIONS.map((sensor) => ({
    x: x + sensor.x * cosYaw - sensor.y * sinYaw,
    y: y + sensor.x * sinYaw + sensor.y * cosYaw
  }));
}

function observationPayload(observation: any): ObservationPayload {
  const pose = observation.pose ?? {};
  return {
    type: "observation",
    sequence_id: Number(observation.sequence_id ?? 0),
    sim_time_s: Number(observation.sim_time_s ?? 0),
    pose: {
      x_m: Number(pose.x_m ?? 0),
      y_m: Number(pose.y_m ?? 0),
      yaw_rad: Number(pose.yaw_rad ?? 0)
    },
    line_sensor_darkness: (observation.line_sensor_darkness ?? []).map(Number),
    sensor_world_positions: sensorWorldPositions(pose),
    yaw_deg: Number(observation.imu?.yaw_deg ?? 0),
    yaw_rate_deg_s: Number(observation.imu?.yaw_rate_deg_s ?? 0),
    progress_m: Number(observation.path_progress?.progress_m ?? 0),
    remaining_distance_m: Number(observation.path_progress?.remaining_distance_m ?? 0),
    completed_events: observation.path_progress?.completed_events ?? [],
    reached_goal: Boolean(observation.path_progress?.reached_goal ?? false),
    cross_track_error_m: Number(observation.oracle?.cross_track_error_m ?? 0),
    heading_error_rad: Number(observation.oracle?.heading_error_rad ?? 0),
    longitudinal_velocity_m_s: Number(observation.kinematics?.longitudinal_velocity_m_s ?? 0),
    yaw_rate_rad_s: Number(observation.kinematics?.yaw_rate_rad_s ?? 0)
  };
}

function loadGrpcClient(target: string): any {
  const packageDefinition = protoLoader.loadSync(VEHICLE_PROTO, {
    keepCase: true,
    longs: Number,
    enums: String,
    defaults: true,
    oneofs: true,
    includeDirs: [PROTO_ROOT]
  });
  const descriptor = grpc.loadPackageDefinition(packageDefinition) as any;
  return new descriptor.artemis_mudri.simulation.v1.VehicleSimulationService(
    target,
    grpc.credentials.createInsecure()
  );
}

function broadcast(clients: Set<WebSocket>, payload: unknown) {
  const message = JSON.stringify(payload);
  for (const client of clients) {
    if (client.readyState === client.OPEN) {
      client.send(message);
    }
  }
}

function startGrpcLoop(config: RuntimeConfig, state: ControlState, clients: Set<WebSocket>, stopFlag: { stop: boolean }) {
  const client = loadGrpcClient(config.target);
  const stream = client.StreamEpisode();
  let sequenceId = 0;

  const queueControl = () => {
    const nextTargets = nextWheelTargets(
      {
        left: state.leftSpeed,
        right: state.rightSpeed
      },
      {
        leftKeyPressed: state.leftKeyPressed,
        rightKeyPressed: state.rightKeyPressed
      },
      {
        accel: config.accel,
        maxSpeed: config.maxSpeed,
        dt: config.controlPeriodS
      }
    );
    state.leftSpeed = nextTargets.left;
    state.rightSpeed = nextTargets.right;
    stream.write({
      control_command: {
        sequence_id: sequenceId,
        rear_left_target_speed: state.leftSpeed,
        rear_right_target_speed: state.rightSpeed
      }
    });
    sequenceId += 1;
  };

  stream.on("data", (response: any) => {
    if (response.started) {
      console.log(`Episode started time_limit=${response.started.time_limit_s} control_period=${response.started.control_period_s}`);
      return;
    }
    if (response.observation) {
      broadcast(clients, observationPayload(response.observation));
      if (stopFlag.stop) {
        stream.write({ stop: { reason: "manual_stop" } });
      } else {
        queueControl();
      }
      return;
    }
    if (response.finished) {
      console.log(`Episode finished reason=${response.finished.reason}`);
      stopFlag.stop = true;
      stream.end();
      return;
    }
    if (response.error) {
      console.error(`Simulation error: ${response.error}`);
      stopFlag.stop = true;
      stream.end();
    }
  });

  stream.on("error", (error: Error) => {
    console.error(`gRPC stream error: ${error.message}`);
  });
  stream.write({
    start: {
      max_time_s: config.maxTimeS,
      control_period_s: config.controlPeriodS
    }
  });
}

function main() {
  const config = runtimeConfig();
  const state: ControlState = {
    leftSpeed: 0,
    rightSpeed: 0,
    leftKeyPressed: false,
    rightKeyPressed: false
  };
  const stopFlag = { stop: false };
  const clients = new Set<WebSocket>();
  const frontendConfig = buildFrontendConfig(config);

  const server = http.createServer((request, response) => {
    if (request.method === "GET" && request.url === "/config.json") {
      const body = JSON.stringify(frontendConfig);
      response.writeHead(200, {
        "Content-Type": "application/json",
        "Content-Length": Buffer.byteLength(body)
      });
      response.end(body);
      return;
    }
    if (request.method === "POST" && request.url === "/stop") {
      stopFlag.stop = true;
      response.writeHead(200, { "Content-Type": "application/json" });
      response.end(JSON.stringify({ ok: true }));
      return;
    }
    response.writeHead(404);
    response.end();
  });
  const wss = new WebSocketServer({ server, path: "/ws" });
  wss.on("connection", (socket) => {
    clients.add(socket);
    socket.on("message", (rawMessage) => {
      const payload = JSON.parse(String(rawMessage));
      if (payload.type === "stop") {
        state.leftKeyPressed = false;
        state.rightKeyPressed = false;
        state.leftSpeed = 0;
        state.rightSpeed = 0;
        stopFlag.stop = true;
        return;
      }
      state.leftKeyPressed = Boolean(payload.left_pressed);
      state.rightKeyPressed = Boolean(payload.right_pressed);
    });
    socket.on("close", () => {
      clients.delete(socket);
    });
  });
  server.listen(config.webPort, config.webHost, () => {
    console.log(`Manual backend listening on http://${config.webHost}:${config.webPort}`);
    console.log(`React dev server should be available at http://127.0.0.1:5173`);
    console.log(`Connecting gRPC target ${config.target}`);
    startGrpcLoop(config, state, clients, stopFlag);
  });
}

main();
