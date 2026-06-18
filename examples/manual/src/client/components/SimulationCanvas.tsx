import { useEffect, useRef } from "react";
import type { Point } from "../../geometry/types";
import type { ManualConfig, ObservationSnapshot } from "../../protocol/types";
import { drawVehicle, drawVehicleSensors } from "../../vehicle/canvas";

type SimulationCanvasProps = {
  config: ManualConfig;
  observation: ObservationSnapshot | null;
  history: Point[];
};

const VIEW_PADDING_PX = 56;
const SENSOR_TRACK_COLOR = "#242520";
const HUMAN_REFERENCE_COLOR = "#1d5fc1";
const HUMAN_REFERENCE_TEXT = "#174a92";

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

export function SimulationCanvas({ config, observation, history }: SimulationCanvasProps) {
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

    const bounds = sceneBounds(config, observation, history);
    const boundsWidth = Math.max(0.01, bounds.maxX - bounds.minX);
    const boundsHeight = Math.max(0.01, bounds.maxY - bounds.minY);
    const safePadding = Math.min(VIEW_PADDING_PX, Math.max(28, Math.min(rect.width, rect.height) * 0.08));
    const scale = Math.min(
      (rect.width - safePadding * 2) / boundsWidth,
      (rect.height - safePadding * 2) / boundsHeight
    );
    const offsetX = (rect.width - boundsWidth * scale) / 2;
    const offsetY = (rect.height - boundsHeight * scale) / 2;
    const project = (point: Point): Point => ({
      x: offsetX + (point.x - bounds.minX) * scale,
      y: offsetY + (bounds.maxY - point.y) * scale
    });

    ctx.fillStyle = "#fbfaf7";
    ctx.fillRect(0, 0, rect.width, rect.height);
    drawPaperGrid(ctx, rect.width, rect.height);

    for (const track of config.scene.officialTrack) {
      drawPolyline(ctx, track, project, SENSOR_TRACK_COLOR, 10);
    }
    drawPolyline(ctx, config.scene.referencePath, project, HUMAN_REFERENCE_COLOR, 2);
    drawPolyline(ctx, history, project, "#8f5d2c", 2.5);

    if (observation) {
      drawVehicle(ctx, observation.pose, project, scale);
      drawVehicleSensors(
        ctx,
        observation.pose,
        config.vehicle.sensorLocalPositions,
        observation.lineSensorDarkness,
        project,
        scale
      );
    }

    for (const [name, point] of Object.entries(config.scene.anchors)) {
      const screen = project(point);
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, 5.5, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(29, 95, 193, 0.1)";
      ctx.fill();
      ctx.strokeStyle = HUMAN_REFERENCE_COLOR;
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillStyle = "rgba(251, 250, 247, 0.9)";
      ctx.fillRect(screen.x - 11, screen.y - 30, 22, 16);
      ctx.fillStyle = HUMAN_REFERENCE_TEXT;
      ctx.font = "600 12px Inter, sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText(name, screen.x, screen.y - 22);
    }
  }, [config, observation, history]);

  return <canvas className="robot-canvas" ref={canvasRef} />;
}

function sceneBounds(config: ManualConfig, observation: ObservationSnapshot | null, history: Point[]) {
  const points = [
    ...config.scene.referencePath,
    ...config.scene.officialTrack.flat(),
    ...Object.values(config.scene.anchors),
    ...history,
    ...(observation ? [{ x: observation.pose.xM, y: observation.pose.yM }] : [])
  ];
  const xs = points.map((point) => point.x);
  const ys = points.map((point) => point.y);
  return {
    minX: Math.min(...xs),
    maxX: Math.max(...xs),
    minY: Math.min(...ys),
    maxY: Math.max(...ys)
  };
}

function drawPaperGrid(ctx: CanvasRenderingContext2D, width: number, height: number) {
  ctx.save();
  ctx.strokeStyle = "rgba(17, 24, 39, 0.045)";
  ctx.lineWidth = 1;
  const spacing = 40;
  for (let x = 0; x <= width; x += spacing) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y <= height; y += spacing) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  ctx.restore();
}
