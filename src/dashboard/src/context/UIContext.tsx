import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useTelemetryContext } from './TelemetryContext';
import type { ActiveMission } from '../types/stateMachine';
import type { UIContextValue, UIContextAlert } from '../types/context';

const UIContext = createContext<UIContextValue | null>(null);

export function useUIContext(): UIContextValue {
  const context = useContext(UIContext);
  if (!context) {
    throw new Error('useUIContext must be used within UIContextProvider');
  }
  return context;
}

interface UIContextProviderProps {
  children: React.ReactNode;
}

export function UIContextProvider({ children }: UIContextProviderProps) {
  const { telemetry, systemStatus } = useTelemetryContext();
  const [isOnline, setIsOnline] = useState(typeof navigator !== 'undefined' ? navigator.onLine : true);
  const [offlineData, setOfflineData] = useState<Record<string, unknown> | null>(null);
  const [activeMission, setActiveMission] = useState<ActiveMission | null>(null);
  const [runningTests, setRunningTests] = useState(0);
  const [alerts, setAlerts] = useState<UIContextAlert[]>([]);
  const errorCount = alerts.filter((a) => a.type === 'error').length;

  useEffect(() => {
    if (typeof window === 'undefined') return;
    const handleOnline = () => {
      setIsOnline(true);
      if (offlineData) setOfflineData(null);
    };
    const handleOffline = () => setIsOnline(false);
    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);
    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, [offlineData]);

  const cacheDataForOffline = useCallback((data: Record<string, unknown>) => {
    if (!isOnline) {
      setOfflineData((prev) => ({ ...prev, ...data, timestamp: Date.now() }));
    }
  }, [isOnline]);

  useEffect(() => {
    const newAlerts: UIContextAlert[] = [];
    if (telemetry.gps?.hdop != null && telemetry.gps.hdop > 2.0) {
      newAlerts.push({
        id: 'gps_drift',
        type: 'warning',
        message: 'GPS drift detected',
        component: 'navigation',
        timestamp: Date.now()
      });
    }
    if (telemetry.battery < 20) {
      newAlerts.push({
        id: 'low_battery',
        type: 'error',
        message: 'Low battery warning',
        component: 'power',
        timestamp: Date.now()
      });
    }
    if (systemStatus?.navigation === 'degraded') {
      newAlerts.push({
        id: 'nav_degraded',
        type: 'warning',
        message: 'Navigation system degraded',
        component: 'navigation',
        timestamp: Date.now()
      });
    }
    setAlerts(newAlerts);
  }, [telemetry, systemStatus]);

  const value: UIContextValue = {
    alerts,
    setAlerts,
    errorCount,
    activeMission,
    setActiveMission,
    runningTests,
    setRunningTests,
    isOnline,
    offlineData,
    cacheDataForOffline
  };

  return (
    <UIContext.Provider value={value}>
      {children}
    </UIContext.Provider>
  );
}
