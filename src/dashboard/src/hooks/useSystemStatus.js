import { useTelemetryContext } from '../context/TelemetryContext';

/**
 * Focused hook for system status only (safety, navigation, vision, can, websocket).
 * Avoids re-renders when only telemetry values change.
 */
export function useSystemStatus() {
  const { systemStatus } = useTelemetryContext();
  return { systemStatus };
}
