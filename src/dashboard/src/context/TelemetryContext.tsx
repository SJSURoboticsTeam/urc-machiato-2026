import React, { createContext, useContext, useState, useEffect } from 'react';
import { useROSContext } from './ROSContext';
import { useDebouncedCallback } from '../utils/debounce';
import { validateTelemetryPayload, validateSimulationMessage } from '../utils/validation';
import type { Telemetry, SystemStatus } from '../types/telemetry';
import type { TelemetryContextValue } from '../types/context';

const DEFAULT_TELEMETRY: Telemetry = {
  battery: 85,
  gps: { satellites: 12, hdop: 1.2, position: { lat: 38.406, lon: -110.792 } },
  speed: 0.0,
  temperature: 28,
  timestamp: Date.now(),
  imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0 },
  environment: { temperature: 25, humidity: 30, visibility: 1.0, dust_density: 0.0 }
};

const TelemetryContext = createContext<TelemetryContextValue | null>(null);

export function useTelemetryContext(): TelemetryContextValue {
  const context = useContext(TelemetryContext);
  if (!context) {
    throw new Error('useTelemetryContext must be used within TelemetryContextProvider');
  }
  return context;
}

interface TelemetryContextProviderProps {
  children: React.ReactNode;
}

export function TelemetryContextProvider({ children }: TelemetryContextProviderProps) {
  const { isConnected } = useROSContext();
  const [telemetry, setTelemetryState] = useState<Telemetry>(DEFAULT_TELEMETRY);
  const [systemStatus, setSystemStatus] = useState<SystemStatus>({
    safety: 'ready',
    navigation: 'ok',
    vision: 'ready',
    can: 'mock',
    websocket: isConnected ? 'connected' : 'disconnected'
  });

  const setTelemetry = useDebouncedCallback((next: Telemetry | ((prev: Telemetry) => Telemetry)) => {
    if (typeof next === 'function') {
      setTelemetryState((prev) => {
        const updated = next(prev);
        const result = validateTelemetryPayload(updated);
        return result.valid ? result.sanitized : prev;
      });
    } else {
      const result = validateTelemetryPayload(next);
      if (result.valid) setTelemetryState(result.sanitized);
    }
  }, 100);

  useEffect(() => {
    setSystemStatus((prev) => ({
      ...prev,
      websocket: isConnected ? 'connected' : 'disconnected'
    }));
  }, [isConnected]);

  useEffect(() => {
    let ws: WebSocket | null = null;
    let reconnectTimeout: ReturnType<typeof setTimeout> | null = null;
    const reconnectDelay = 2000;

    const connectToSimulation = () => {
      if (ws?.readyState === WebSocket.OPEN) return;
      ws = new WebSocket('ws://localhost:8766');

      ws.onopen = () => {
        setSystemStatus((prev) => ({ ...prev, can: 'operational' }));
      };

      ws.onmessage = (event: MessageEvent) => {
        try {
          const data = JSON.parse(event.data as string);
          const result = validateSimulationMessage(data);
          if (!result.valid || !result.sanitized) return;
          const simData = result.sanitized as Record<string, unknown>;
          const gps = simData.gps as Record<string, unknown> | undefined;
          const rover = simData.rover as { velocity?: number[] } | undefined;
          const env = simData.environment as Record<string, unknown> | undefined;
          const imu = simData.imu as Record<string, unknown> | undefined;
          setTelemetry((prev) => ({
            ...prev,
            battery: prev.battery,
            gps: {
              satellites: (gps?.satellites as number) ?? prev.gps.satellites,
              hdop: (gps?.hdop as number) ?? prev.gps.hdop,
              position: {
                lat: (gps?.latitude as number) ?? prev.gps.position.lat,
                lon: (gps?.longitude as number) ?? prev.gps.position.lon
              }
            },
            speed: rover?.velocity
              ? Math.sqrt(rover.velocity[0] ** 2 + rover.velocity[1] ** 2)
              : prev.speed,
            temperature: (env?.temperature as number) ?? (imu?.temperature as number) ?? prev.temperature,
            timestamp: Date.now(),
            imu: imu ? { ...prev.imu, ...imu } : prev.imu,
            environment: env ? { ...prev.environment, ...env } : prev.environment
          }));
        } catch (err) {
          console.warn('Failed to parse simulation data:', err);
        }
      };

      ws.onclose = () => {
        setSystemStatus((prev) => ({ ...prev, can: 'disconnected' }));
        reconnectTimeout = setTimeout(connectToSimulation, reconnectDelay);
      };

      ws.onerror = () => {
        setSystemStatus((prev) => ({ ...prev, can: 'error' }));
      };
    };

    connectToSimulation();
    return () => {
      if (ws) ws.close();
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
    };
  }, []);

  const value: TelemetryContextValue = {
    telemetry,
    setTelemetry,
    systemStatus,
    setSystemStatus
  };

  return (
    <TelemetryContext.Provider value={value}>
      {children}
    </TelemetryContext.Provider>
  );
}
