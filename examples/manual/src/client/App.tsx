import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import RobotCanvas from "./RobotCanvas";
import { nextWheelTargets } from "../shared/control";
import type { ManualConfig, ObservationPayload, Point, WheelTargets } from "../shared/types";

const MAX_HISTORY_POINTS = 500;

async function loadConfig(): Promise<ManualConfig> {
  const response = await fetch("/config.json");
  if (!response.ok) {
    throw new Error(`配置加载失败：${response.status}`);
  }
  return response.json() as Promise<ManualConfig>;
}

function formatNumber(value: number, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "-";
}

export default function App() {
  const [config, setConfig] = useState<ManualConfig | null>(null);
  const [status, setStatus] = useState("正在加载控制配置...");
  const [pressed, setPressed] = useState<Set<string>>(() => new Set());
  const [targets, setTargets] = useState<WheelTargets>({ left: 0, right: 0 });
  const [observation, setObservation] = useState<ObservationPayload | null>(null);
  const [history, setHistory] = useState<Point[]>([]);
  const socketRef = useRef<WebSocket | null>(null);
  const lastFrameRef = useRef(performance.now());
  const lastSendRef = useRef(0);
  const pressedRef = useRef(pressed);
  const configRef = useRef<ManualConfig | null>(null);

  useEffect(() => {
    pressedRef.current = pressed;
  }, [pressed]);

  useEffect(() => {
    configRef.current = config;
  }, [config]);

  const sendPressedState = useCallback((force = false) => {
    const activeConfig = configRef.current;
    if (!activeConfig) return;
    const now = performance.now();
    if (!force && now - lastSendRef.current < 20) return;
    lastSendRef.current = now;
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) return;
    socket.send(JSON.stringify({
      left_pressed: pressedRef.current.has(activeConfig.leftKey),
      right_pressed: pressedRef.current.has(activeConfig.rightKey)
    }));
  }, []);

  const setPressedKey = useCallback((key: string, value: boolean) => {
    setPressed((current) => {
      const next = new Set(current);
      if (value) {
        next.add(key);
      } else {
        next.delete(key);
      }
      pressedRef.current = next;
      return next;
    });
    window.setTimeout(() => sendPressedState(true), 0);
  }, [sendPressedState]);

  useEffect(() => {
    loadConfig()
      .then((loaded) => {
        setConfig(loaded);
        setStatus(`页面获得焦点后，按住 ${loaded.leftKey.toUpperCase()} / ${loaded.rightKey.toUpperCase()} 或按住按钮控制速度。`);
      })
      .catch((error) => setStatus(String(error)));
  }, []);

  useEffect(() => {
    if (!config) return;
    let stopped = false;
    let reconnectTimer: number | null = null;
    let activeSocket: WebSocket | null = null;

    const connect = () => {
      if (stopped) return;
      const socket = new WebSocket(config.wsUrl);
      activeSocket = socket;
      socketRef.current = socket;
      socket.addEventListener("open", () => {
        setStatus("WebSocket 已连接");
        sendPressedState(true);
      });
      socket.addEventListener("message", (event) => {
        const payload = JSON.parse(event.data) as ObservationPayload;
        if (payload.type !== "observation") return;
        setObservation(payload);
        setHistory((current) => {
          const next = [...current, { x: payload.pose.x_m, y: payload.pose.y_m }];
          return next.slice(Math.max(0, next.length - MAX_HISTORY_POINTS));
        });
      });
      socket.addEventListener("close", () => {
        if (stopped) return;
        setStatus("WebSocket 已断开，正在重连...");
        reconnectTimer = window.setTimeout(connect, 500);
      });
      socket.addEventListener("error", () => {
        setStatus("WebSocket 连接错误");
      });
    };

    connect();
    return () => {
      stopped = true;
      if (reconnectTimer !== null) window.clearTimeout(reconnectTimer);
      activeSocket?.close();
    };
  }, [config, sendPressedState]);

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
      setPressed(new Set());
      pressedRef.current = new Set();
      sendPressedState(true);
    };

    const keydown = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key === config.leftKey || key === config.rightKey) {
        event.preventDefault();
        setPressedKey(key, true);
      }
      if (key === "q" || key === "escape") {
        if (socketRef.current?.readyState === WebSocket.OPEN) {
          socketRef.current.send(JSON.stringify({ type: "stop" }));
        } else {
          fetch("/stop", { method: "POST", keepalive: true });
        }
        setStatus("已请求停止");
      }
    };

    const keyup = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key !== config.leftKey && key !== config.rightKey) return;
      event.preventDefault();
      setPressedKey(key, false);
    };

    window.addEventListener("keydown", keydown);
    window.addEventListener("keyup", keyup);
    window.addEventListener("blur", clearPressed);
    return () => {
      window.removeEventListener("keydown", keydown);
      window.removeEventListener("keyup", keyup);
      window.removeEventListener("blur", clearPressed);
    };
  }, [config, sendPressedState, setPressedKey]);

  const controlPressed = useMemo(() => ({
    left: config ? pressed.has(config.leftKey) : false,
    right: config ? pressed.has(config.rightKey) : false
  }), [config, pressed]);

  if (!config) {
    return <main className="app-shell"><div className="loading">{status}</div></main>;
  }

  return (
    <main className="app-shell">
      <section className="viewer-surface">
        <RobotCanvas config={config} observation={observation} history={history} />
      </section>
      <aside className="control-panel">
        <header>
          <h1>Artemis 手动操控台</h1>
          <div className="status-line">{status}</div>
        </header>
        <div className="drive-buttons">
          <button
            className={controlPressed.left ? "active" : ""}
            onPointerDown={(event) => {
              event.currentTarget.setPointerCapture(event.pointerId);
              setPressedKey(config.leftKey, true);
            }}
            onPointerUp={() => setPressedKey(config.leftKey, false)}
            onPointerCancel={() => setPressedKey(config.leftKey, false)}
          >
            <span>左转</span>
            <strong>{config.leftKey.toUpperCase()}</strong>
          </button>
          <button
            className={controlPressed.right ? "active" : ""}
            onPointerDown={(event) => {
              event.currentTarget.setPointerCapture(event.pointerId);
              setPressedKey(config.rightKey, true);
            }}
            onPointerUp={() => setPressedKey(config.rightKey, false)}
            onPointerCancel={() => setPressedKey(config.rightKey, false)}
          >
            <span>右转</span>
            <strong>{config.rightKey.toUpperCase()}</strong>
          </button>
        </div>
        <div className="meter">
          <span>左轮目标</span>
          <progress value={targets.left} max={config.maxSpeed} />
          <strong>{formatNumber(targets.left)}</strong>
        </div>
        <div className="meter">
          <span>右轮目标</span>
          <progress value={targets.right} max={config.maxSpeed} />
          <strong>{formatNumber(targets.right)}</strong>
        </div>
        <div className="telemetry-grid">
          <div><span>时间</span><strong>{formatNumber(observation?.sim_time_s ?? 0)} s</strong></div>
          <div><span>航向</span><strong>{formatNumber(observation?.yaw_deg ?? 0, 1)}°</strong></div>
          <div><span>进度</span><strong>{formatNumber(observation?.progress_m ?? 0)} m</strong></div>
          <div><span>剩余</span><strong>{formatNumber(observation?.remaining_distance_m ?? 0)} m</strong></div>
          <div><span>横向误差</span><strong>{formatNumber(observation?.cross_track_error_m ?? 0, 3)} m</strong></div>
          <div><span>纵向速度</span><strong>{formatNumber(observation?.longitudinal_velocity_m_s ?? 0, 3)} m/s</strong></div>
        </div>
      </aside>
    </main>
  );
}
