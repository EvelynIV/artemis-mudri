import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import type { ManualConfig } from "../../protocol/types";
import { nextWheelTargets, type WheelTargets } from "../../vehicle/model";

type ControlPressedState = {
  leftPressed: boolean;
  rightPressed: boolean;
};

type UseDriveInputOptions = {
  config: ManualConfig | null;
  enabled: boolean;
  sendControl: (control: ControlPressedState, force?: boolean) => void;
  requestStop: () => void;
  onActivity?: (message: string) => void;
};

export type ControlSource = "keyboard" | "button" | "released";

export type ManualControlFeedback = {
  source: ControlSource;
  released: boolean;
};

export function useDriveInput({ config, enabled, sendControl, requestStop, onActivity }: UseDriveInputOptions) {
  const [pressed, setPressed] = useState<Set<string>>(() => new Set());
  const [targets, setTargets] = useState<WheelTargets>({ left: 0, right: 0 });
  const [feedback, setFeedback] = useState<ManualControlFeedback>({
    source: "released",
    released: true
  });
  const lastFrameRef = useRef(performance.now());
  const pressedRef = useRef(pressed);

  const controlPressed = useMemo(() => ({
    left: config ? pressed.has(config.leftKey) : false,
    right: config ? pressed.has(config.rightKey) : false
  }), [config, pressed]);

  const sendPressedState = useCallback((nextPressed: Set<string>, force = false) => {
    if (!config) return;
    sendControl({
      leftPressed: nextPressed.has(config.leftKey),
      rightPressed: nextPressed.has(config.rightKey)
    }, force);
  }, [config, sendControl]);

  const setPressedKey = useCallback((key: string, value: boolean, source: ControlSource = "button") => {
    if (!enabled && value) return;
    setPressed((current) => {
      const next = new Set(current);
      if (value) {
        next.add(key);
      } else {
        next.delete(key);
      }
      pressedRef.current = next;
      setFeedback({
        source: next.size > 0 ? source : "released",
        released: next.size === 0
      });
      if (current.size === 0 && next.size > 0) {
        onActivity?.(`${source === "keyboard" ? "键盘" : "按钮"}接管控制`);
      }
      if (current.size > 0 && next.size === 0) {
        onActivity?.("控制已释放");
      }
      window.setTimeout(() => sendPressedState(next, true), 0);
      return next;
    });
  }, [enabled, onActivity, sendPressedState]);

  useEffect(() => {
    pressedRef.current = pressed;
  }, [pressed]);

  useEffect(() => {
    if (enabled || pressedRef.current.size === 0) return;
    const next = new Set<string>();
    setPressed(next);
    pressedRef.current = next;
    setFeedback({ source: "released", released: true });
    sendPressedState(next, true);
    onActivity?.("仿真未运行，控制已释放");
  }, [enabled, onActivity, sendPressedState]);

  useEffect(() => {
    if (!config) return;
    let animationId = 0;
    const tick = (now: number) => {
      const dt = Math.min(0.1, (now - lastFrameRef.current) / 1000);
      lastFrameRef.current = now;
      const active = pressedRef.current;
      setTargets((current) => nextWheelTargets(
        current,
        {
          leftKeyPressed: active.has(config.leftKey),
          rightKeyPressed: active.has(config.rightKey)
        },
        {
          accel: config.accel,
          maxSpeed: config.maxSpeed,
          dt
        }
      ));
      animationId = requestAnimationFrame(tick);
    };
    animationId = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(animationId);
  }, [config]);

  useEffect(() => {
    if (!config) return;
    const clearPressed = () => {
      const next = new Set<string>();
      const hadActiveControl = pressedRef.current.size > 0;
      setPressed(next);
      pressedRef.current = next;
      setFeedback({ source: "released", released: true });
      if (hadActiveControl) {
        onActivity?.("窗口失焦，控制已释放");
      }
      sendPressedState(next, true);
    };

    const keydown = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key === config.leftKey || key === config.rightKey) {
        event.preventDefault();
        if (!enabled) return;
        setPressedKey(key, true, "keyboard");
      }
      if (key === "q" || key === "escape") {
        requestStop();
      }
    };

    const keyup = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key !== config.leftKey && key !== config.rightKey) return;
      event.preventDefault();
      setPressedKey(key, false, "keyboard");
    };

    window.addEventListener("keydown", keydown);
    window.addEventListener("keyup", keyup);
    window.addEventListener("blur", clearPressed);
    return () => {
      window.removeEventListener("keydown", keydown);
      window.removeEventListener("keyup", keyup);
      window.removeEventListener("blur", clearPressed);
    };
  }, [config, enabled, onActivity, requestStop, sendPressedState, setPressedKey]);

  return {
    feedback,
    controlPressed,
    setPressedKey,
    targets
  };
}
