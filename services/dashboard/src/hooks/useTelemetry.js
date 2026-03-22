import { useTelemetryContext } from '../context/TelemetryContext';

/**
 * Focused hook for telemetry and system status.
 * Wraps TelemetryContext so components can subscribe only to telemetry-related state.
 */
export function useTelemetry() {
  const { telemetry, setTelemetry, systemStatus, setSystemStatus } = useTelemetryContext();
  return { telemetry, setTelemetry, systemStatus, setSystemStatus };
}
