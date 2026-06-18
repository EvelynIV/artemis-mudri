import { useState, type ReactNode } from "react";
import type { ManualConfig, ObservationSnapshot, RuntimeStatus, SimulationEventSnapshot } from "../../protocol/types";
import type { ControlSource } from "../hooks/useDriveInput";
import type { WheelTargets } from "../../vehicle/model";

type ActivityLogEntry = {
  id: number;
  time: string;
  message: string;
};

type SimulationEventLogEntry = SimulationEventSnapshot & {
  id: string;
  simTimeS: number;
  sequenceId: number;
};

type EventTone = "neutral" | "warning" | "error";

type EventMetadataItem = {
  label: string;
  value: string;
};

type EventDetailsItem = {
  key: string;
  value: string;
};

type EventPresentation = {
  key: string;
  title: string;
  source: string;
  sequence: string;
  timestamp: string;
  metadata: EventMetadataItem[];
  details: EventDetailsItem[];
  tone: EventTone;
};

type ControlPanelProps = {
  config: ManualConfig;
  controlPressed: {
    left: boolean;
    right: boolean;
  };
  targets: WheelTargets;
  setPressedKey: (key: string, value: boolean, source?: ControlSource) => void;
  requestStop: () => void;
  disabled: boolean;
  disabledReason?: string;
};

function formatNumber(value: number, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : "-";
}

function valueWithUnit(value: number, unit: string, digits = 2) {
  return (
    <>
      {formatNumber(value, digits)} <span>{unit}</span>
    </>
  );
}

type TelemetryPanelProps = {
  activityLog: ActivityLogEntry[];
  runtimeStatus: RuntimeStatus;
  observation: ObservationSnapshot | null;
  simulationEventLog: SimulationEventLogEntry[];
};

export function TelemetryPanel({ activityLog, runtimeStatus, observation, simulationEventLog }: TelemetryPanelProps) {
  return (
    <aside className="telemetry-panel">
      <div className="pane-header">
        <div>
          <h2>实时遥测</h2>
          <p>车辆运动、轨迹状态和诊断信息</p>
        </div>
      </div>
      <div className="telemetry-sections">
        <TelemetrySection title="车辆运动" defaultOpen>
          <TelemetryRow label="航向">{valueWithUnit(observation?.yawDeg ?? 0, "°", 1)}</TelemetryRow>
          <TelemetryRow label="纵向速度">{valueWithUnit(observation?.longitudinalVelocityMS ?? 0, "m/s", 3)}</TelemetryRow>
          <TelemetryRow label="角速度">{valueWithUnit(observation?.yawRateRadS ?? 0, "rad/s", 3)}</TelemetryRow>
        </TelemetrySection>
        <TelemetrySection title="轨迹状态" defaultOpen>
          <TelemetryRow label="进度">{valueWithUnit(observation?.progressM ?? 0, "m")}</TelemetryRow>
          <TelemetryRow label="剩余">{valueWithUnit(observation?.remainingDistanceM ?? 0, "m")}</TelemetryRow>
          <TelemetryRow label="横向误差" emphasis>{valueWithUnit(observation?.crossTrackErrorM ?? 0, "m", 3)}</TelemetryRow>
        </TelemetrySection>
        <TelemetrySection title="会话信息" defaultOpen>
          <TelemetryRow label="已运行时间">{valueWithUnit(observation?.simTimeS ?? 0, "s")}</TelemetryRow>
          <TelemetryRow label="累计事件">{simulationEventLog.length}</TelemetryRow>
        </TelemetrySection>
        <SimulationEventSection
          entries={simulationEventLog.slice(0, 5)}
          runtimeStatus={runtimeStatus}
        />
        <TelemetrySection title="诊断信息">
          <TelemetryRow label="Seq">{observation?.sequenceId ?? "-"}</TelemetryRow>
          <TelemetryRow label="状态">{runtimeStatus}</TelemetryRow>
          <TelemetryRow label="本帧事件">{observation?.completedEvents.length ?? 0}</TelemetryRow>
        </TelemetrySection>
        <ActivitySection entries={activityLog.slice(0, 5)} />
      </div>
    </aside>
  );
}

export function DriveControls({
  config,
  controlPressed,
  targets,
  setPressedKey,
  requestStop,
  disabled,
  disabledReason
}: ControlPanelProps) {
  return (
    <footer className="manual-controlbar">
      <div className="drive-buttons">
        <button
          className={controlPressed.left ? "active" : ""}
          disabled={disabled}
          onPointerDown={(event) => {
            if (disabled) return;
            event.currentTarget.setPointerCapture(event.pointerId);
            setPressedKey(config.leftKey, true, "button");
          }}
          onPointerUp={() => setPressedKey(config.leftKey, false, "button")}
          onPointerCancel={() => setPressedKey(config.leftKey, false, "button")}
        >
          <span>左转</span>
          <kbd>{config.leftKey.toUpperCase()}</kbd>
        </button>
        <button
          className={controlPressed.right ? "active" : ""}
          disabled={disabled}
          onPointerDown={(event) => {
            if (disabled) return;
            event.currentTarget.setPointerCapture(event.pointerId);
            setPressedKey(config.rightKey, true, "button");
          }}
          onPointerUp={() => setPressedKey(config.rightKey, false, "button")}
          onPointerCancel={() => setPressedKey(config.rightKey, false, "button")}
        >
          <span>右转</span>
          <kbd>{config.rightKey.toUpperCase()}</kbd>
        </button>
      </div>
      <div className="drive-meters">
        <div className="meter">
          <span>左轮目标</span>
          <input type="range" min={0} max={config.maxSpeed} step={0.01} value={targets.left} tabIndex={-1} onChange={() => undefined} />
          <strong>{formatNumber(targets.left)}</strong>
        </div>
        <div className="meter">
          <span>右轮目标</span>
          <input type="range" min={0} max={config.maxSpeed} step={0.01} value={targets.right} tabIndex={-1} onChange={() => undefined} />
          <strong>{formatNumber(targets.right)}</strong>
        </div>
        {disabledReason ? <p className="control-note">控制不可用：{disabledReason}</p> : <p className="control-note">松开按键或窗口失焦后自动释放控制。</p>}
      </div>
      <button className="stop-button" type="button" onClick={requestStop}>停止</button>
    </footer>
  );
}

function TelemetrySection({
  title,
  children,
  defaultOpen = false
}: {
  title: string;
  children: ReactNode;
  defaultOpen?: boolean;
}) {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  return (
    <details className="telemetry-section" open={isOpen} onToggle={(event) => setIsOpen(event.currentTarget.open)}>
      <summary>{title}</summary>
      <dl>{children}</dl>
    </details>
  );
}

function TelemetryRow({
  label,
  children,
  emphasis = false
}: {
  label: string;
  children: ReactNode;
  emphasis?: boolean;
}) {
  return (
    <div className={emphasis ? "telemetry-row is-emphasis" : "telemetry-row"}>
      <dt>{label}</dt>
      <dd>{children}</dd>
    </div>
  );
}

function ActivitySection({ entries }: { entries: ActivityLogEntry[] }) {
  const [isOpen, setIsOpen] = useState(false);
  return (
    <details className="activity-section" open={isOpen} onToggle={(event) => setIsOpen(event.currentTarget.open)}>
      <summary>行为记录</summary>
      <ol>
        {entries.map((entry) => (
          <li key={entry.id}>
            <time>{entry.time}</time>
            <span>{entry.message}</span>
          </li>
        ))}
      </ol>
    </details>
  );
}

function SimulationEventSection({
  entries,
  runtimeStatus
}: {
  entries: SimulationEventLogEntry[];
  runtimeStatus: RuntimeStatus;
}) {
  const [isOpen, setIsOpen] = useState(true);
  const presentations = entries.map(toEventPresentation);
  return (
    <details className="simulation-event-section" open={isOpen} onToggle={(event) => setIsOpen(event.currentTarget.open)}>
      <summary>仿真事件</summary>
      {presentations.length === 0 ? (
        <p className="event-empty">{emptyEventText(runtimeStatus)}</p>
      ) : (
        <ol className="simulation-event-list">
          {presentations.map((event, index) => (
            <li key={event.key} className={index === 0 ? `event-item is-latest is-${event.tone}` : `event-item is-${event.tone}`}>
              <details className="event-details">
                <summary className="event-summary-row">
                  <span className="event-summary-main">
                    <strong>{event.title}</strong>
                    <span>
                      {event.timestamp} · {event.source}
                      {event.metadata.map((item) => (
                        <span key={item.label}> · {item.label} {item.value}</span>
                      ))}
                    </span>
                  </span>
                  <span className="event-summary-sequence">{event.sequence}</span>
                </summary>
                <dl>
                  {event.details.map((item) => (
                    <div key={item.key}>
                      <dt>{item.key}</dt>
                      <dd>{item.value}</dd>
                    </div>
                  ))}
                </dl>
              </details>
            </li>
          ))}
        </ol>
      )}
    </details>
  );
}

function toEventPresentation(event: SimulationEventLogEntry): EventPresentation {
  const timestamp = readEventTimestamp(event);
  return {
    key: getEventKey(event),
    title: getEventTitle(event),
    source: `${event.namespace || "unknown"}/${event.type || "unknown"}`,
    sequence: `Seq ${event.sequenceId}`,
    timestamp: `${formatNumber(timestamp, 2)} s`,
    metadata: getEventMetadata(event),
    details: getEventDetails(event),
    tone: getEventTone(event)
  };
}

function getEventKey(event: SimulationEventLogEntry) {
  return event.id || `${event.eventId}:${event.sequenceId}:${event.name}`;
}

function getEventTitle(event: SimulationEventLogEntry) {
  if (event.namespace === "path" && event.type === "checkpoint" && event.name) {
    return `检查点 ${event.name}`;
  }
  return event.name || event.type || "未知事件";
}

function getEventMetadata(event: SimulationEventLogEntry): EventMetadataItem[] {
  const metadata: EventMetadataItem[] = [];
  const pathIndex = event.metrics.path_index;
  if (Number.isFinite(pathIndex)) {
    metadata.push({ label: "路径索引", value: formatMetricValue(pathIndex) });
  }
  return metadata;
}

function getEventDetails(event: SimulationEventLogEntry): EventDetailsItem[] {
  const details: EventDetailsItem[] = [
    { key: "event_id", value: String(event.eventId) },
    { key: "step_id", value: String(event.stepId) },
    { key: "severity", value: event.severity || "info" },
    { key: "namespace", value: event.namespace || "unknown" },
    { key: "type", value: event.type || "unknown" }
  ];
  for (const [key, value] of Object.entries(event.metrics)) {
    details.push({ key: `metrics.${key}`, value: formatMetricValue(value) });
  }
  for (const [key, value] of Object.entries(event.labels)) {
    details.push({ key: `labels.${key}`, value });
  }
  return details;
}

function getEventTone(event: SimulationEventLogEntry): EventTone {
  if (event.severity === "error" || event.severity === "critical") return "error";
  if (event.severity === "warning" || event.severity === "warn") return "warning";
  return "neutral";
}

function readEventTimestamp(event: SimulationEventLogEntry) {
  const timestamp = event.metrics.timestamp_s;
  return Number.isFinite(timestamp) ? timestamp : event.simTimeS;
}

function formatMetricValue(value: number) {
  if (!Number.isFinite(value)) return "-";
  return Number.isInteger(value) ? String(value) : formatNumber(value, 2);
}

function emptyEventText(runtimeStatus: RuntimeStatus) {
  if (runtimeStatus === "running") return "仿真运行中，等待首个事件。";
  return "启动仿真后，检查点、路径和控制事件会在这里实时出现。";
}
