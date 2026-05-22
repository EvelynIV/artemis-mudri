import { useEffect, useRef } from "react";
import type { ManualConfig, ObservationPayload, Point, Pose } from "../shared/types";

type RobotCanvasProps = {
  config: ManualConfig;
  observation: ObservationPayload | null;
  history: Point[];
};

const FIELD_PADDING_PX = 36;
const CAR_LENGTH_M = 0.18;
const CAR_WIDTH_M = 0.11;

function drawPolyline(
  ctx: CanvasRenderingContext2D,
  points: Point[],
  project: (point: Point) => Point,
  strokeStyle: string,
  lineWidth: number
) {
  if (points.length < 2) return;
  ctx.beginPath();
  const first = project(points[0]);
  ctx.moveTo(first.x, first.y);
  for (const point of points.slice(1)) {
    const next = project(point);
    ctx.lineTo(next.x, next.y);
  }
  ctx.strokeStyle = strokeStyle;
  ctx.lineWidth = lineWidth;
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.stroke();
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  sensorPositions: Point[],
  darkness: number[],
  project: (point: Point) => Point,
  scale: number
) {
  const center = project({ x: pose.x_m, y: pose.y_m });
  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(-pose.yaw_rad);

  ctx.fillStyle = "#ffffff";
  ctx.strokeStyle = "#17202a";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(
    -0.5 * CAR_LENGTH_M * scale,
    -0.5 * CAR_WIDTH_M * scale,
    CAR_LENGTH_M * scale,
    CAR_WIDTH_M * scale,
    8
  );
  ctx.fill();
  ctx.stroke();

  ctx.fillStyle = "#d9480f";
  ctx.beginPath();
  ctx.moveTo(0.5 * CAR_LENGTH_M * scale + 10, 0);
  ctx.lineTo(0.5 * CAR_LENGTH_M * scale - 8, -8);
  ctx.lineTo(0.5 * CAR_LENGTH_M * scale - 8, 8);
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  for (const [index, sensor] of sensorPositions.entries()) {
    const value = Math.max(0, Math.min(1, darkness[index] ?? 0));
    const screen = project(sensor);
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, 4 + value * 6, 0, Math.PI * 2);
    ctx.fillStyle = `rgba(31, 122, 92, ${0.25 + value * 0.75})`;
    ctx.fill();
    ctx.strokeStyle = "#145840";
    ctx.lineWidth = 1;
    ctx.stroke();
  }
}

export default function RobotCanvas({ config, observation, history }: RobotCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.floor(rect.width * ratio));
    canvas.height = Math.max(1, Math.floor(rect.height * ratio));

    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    ctx.clearRect(0, 0, rect.width, rect.height);

    const field = config.scene.field;
    const scale = Math.min(
      (rect.width - FIELD_PADDING_PX * 2) / field.widthM,
      (rect.height - FIELD_PADDING_PX * 2) / field.heightM
    );
    const offsetX = (rect.width - field.widthM * scale) / 2;
    const offsetY = (rect.height - field.heightM * scale) / 2;
    const project = (point: Point): Point => ({
      x: offsetX + point.x * scale,
      y: offsetY + (field.heightM - point.y) * scale
    });

    ctx.fillStyle = "#f8fafc";
    ctx.fillRect(0, 0, rect.width, rect.height);

    const topLeft = project({ x: 0, y: field.heightM });
    ctx.fillStyle = "#ffffff";
    ctx.strokeStyle = "#aeb8c2";
    ctx.lineWidth = 2;
    ctx.fillRect(topLeft.x, topLeft.y, field.widthM * scale, field.heightM * scale);
    ctx.strokeRect(topLeft.x, topLeft.y, field.widthM * scale, field.heightM * scale);

    for (const track of config.scene.officialTrack) {
      drawPolyline(ctx, track, project, "#14181f", 11);
    }
    drawPolyline(ctx, history, project, "#e67700", 3);

    for (const [name, point] of Object.entries(config.scene.anchors)) {
      const screen = project(point);
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, 8, 0, Math.PI * 2);
      ctx.fillStyle = "#ffffff";
      ctx.fill();
      ctx.strokeStyle = "#1d64b7";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = "#17202a";
      ctx.font = "600 13px Inter, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(name, screen.x, screen.y - 18);
    }

    if (observation) {
      drawRobot(
        ctx,
        observation.pose,
        observation.sensor_world_positions,
        observation.line_sensor_darkness,
        project,
        scale
      );
    }
  }, [config, observation, history]);

  return <canvas className="robot-canvas" ref={canvasRef} />;
}
