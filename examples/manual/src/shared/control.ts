import type { WheelTargets } from "./types";

export type ManualInputState = {
  leftKeyPressed: boolean;
  rightKeyPressed: boolean;
};

export type ManualControlConfig = {
  accel: number;
  maxSpeed: number;
  dt: number;
};

function clamp(value: number, max: number) {
  return Math.max(0, Math.min(max, value));
}

export function nextWheelTargets(
  current: WheelTargets,
  input: ManualInputState,
  config: ManualControlConfig
): WheelTargets {
  if (input.leftKeyPressed && input.rightKeyPressed) {
    const speed = clamp(Math.max(current.left, current.right) + config.accel * config.dt, config.maxSpeed);
    return { left: speed, right: speed };
  }
  if (input.leftKeyPressed) {
    return {
      left: 0,
      right: clamp(current.right + config.accel * config.dt, config.maxSpeed)
    };
  }
  if (input.rightKeyPressed) {
    return {
      left: clamp(current.left + config.accel * config.dt, config.maxSpeed),
      right: 0
    };
  }
  return { left: 0, right: 0 };
}
