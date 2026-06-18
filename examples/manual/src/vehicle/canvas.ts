import type { Point, Pose } from "../geometry/types";
import { VEHICLE_LENGTH_M, VEHICLE_WIDTH_M } from "./model";

export type ProjectPoint = (point: Point) => Point;

export function drawVehicle(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  project: ProjectPoint,
  scale: number
) {
  const center = project({ x: pose.xM, y: pose.yM });
  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(-pose.yawRad);

  const length = Math.min(VEHICLE_LENGTH_M * scale * 0.82, 46);
  const width = Math.min(VEHICLE_WIDTH_M * scale * 0.82, 28);
  const left = -0.5 * length;
  const right = 0.5 * length;
  const top = -0.5 * width;
  const bottom = 0.5 * width;

  ctx.fillStyle = "#1f211d";
  ctx.strokeStyle = "#fbfbf9";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(right, 0);
  ctx.lineTo(right - 0.24 * length, top);
  ctx.lineTo(left, top);
  ctx.lineTo(left, bottom);
  ctx.lineTo(right - 0.24 * length, bottom);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = "#fbfbf9";
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  ctx.moveTo(left + 0.22 * length, 0);
  ctx.lineTo(right - 0.18 * length, 0);
  ctx.stroke();

  ctx.fillStyle = "#fbfbf9";
  ctx.beginPath();
  ctx.arc(0, 0, Math.max(2.2, width * 0.09), 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

export function drawVehicleSensors(
  ctx: CanvasRenderingContext2D,
  pose: Pose,
  sensorLocalPositions: Point[],
  darkness: number[],
  project: ProjectPoint,
  scale: number
) {
  const cosYaw = Math.cos(pose.yawRad);
  const sinYaw = Math.sin(pose.yawRad);
  const visualForwardOffsetM = VEHICLE_LENGTH_M * 0.32;
  const radius = Math.max(1.4, Math.min(2.1, scale * 0.0048));

  for (const [index, sensor] of sensorLocalPositions.entries()) {
    const value = Math.max(0, Math.min(1, darkness[index] ?? 0));
    const visualLocal = {
      x: Math.min(sensor.x, visualForwardOffsetM),
      y: sensor.y
    };
    const world = {
      x: pose.xM + visualLocal.x * cosYaw - visualLocal.y * sinYaw,
      y: pose.yM + visualLocal.x * sinYaw + visualLocal.y * cosYaw
    };
    const screen = project(world);
    const channel = Math.round(168 + value * 87);
    ctx.beginPath();
    ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
    ctx.fillStyle = `rgb(${channel}, ${channel}, ${channel})`;
    ctx.fill();
    ctx.strokeStyle = value > 0.55 ? "#ffffff" : "#8c949e";
    ctx.lineWidth = value > 0.55 ? 1.1 : 0.7;
    ctx.stroke();
  }
}
