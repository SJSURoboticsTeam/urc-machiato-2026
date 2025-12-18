import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { useROS } from '../hooks/useROS';
import { useStateMachine } from '../hooks/useStateMachine';

/**
 * System Context Provider
 *
 * Manages global system state, telemetry, and mission data for the entire application.
 * Provides context-aware data to all components based on current system state.
 */
const SystemContext = createContext(null);

export const useSystemContext = () => {
  const context = useContext(SystemContext);
  if (!context) {
    throw new Error('useSystemContext must be used within SystemContextProvider');
  }
  return context;
};

export const SystemContextProvider = ({ children }) => {
  // ROS connection
  const { ros, isConnected, connectionStatus } = useROS();

  // Online/Offline state management
  const [isOnline, setIsOnline] = useState(navigator.onLine);
  const [offlineData, setOfflineData] = useState(null);

  // Monitor online/offline status
  useEffect(() => {
    const handleOnline = () => {
      setIsOnline(true);
      // Attempt to restore connection and sync offline data
      if (offlineData) {
        console.log('Back online - syncing offline data');
        // In production, this would sync cached data
        setOfflineData(null);
      }
    };

    const handleOffline = () => {
      setIsOnline(false);
      console.log('Connection lost - switching to offline mode');
    };

    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, [offlineData]);

  // Cache important data when going offline
  const cacheDataForOffline = useCallback((data) => {
    if (!isOnline) {
      setOfflineData(prev => ({ ...prev, ...data, timestamp: Date.now() }));
    }
  }, [isOnline]);

  // State machine
  const {
    currentState,
    currentSubstate,
    requestStateTransition,
    isTransitioning
  } = useStateMachine(ros);

  // System telemetry - now connected to real simulation
  const [telemetry, setTelemetry] = useState({
    battery: 85,
    gps: { satellites: 12, hdop: 1.2, position: { lat: 38.406, lon: -110.792 } },
    speed: 0.0,
    temperature: 28,
    timestamp: Date.now(),
    imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0 },
    environment: { temperature: 25, humidity: 30, visibility: 1.0, dust_density: 0.0 }
  });

  // System status
  const [systemStatus, setSystemStatus] = useState({
    safety: 'ready',
    navigation: 'ok',
    vision: 'ready',
    can: 'mock', // Always mock for now
    websocket: isConnected ? 'connected' : 'disconnected'
  });

  // Active mission
  const [activeMission, setActiveMission] = useState(null);

  // Alerts and warnings
  const [alerts, setAlerts] = useState([]);

  // Error count for debug tab badge
  const [errorCount, setErrorCount] = useState(0);

  // Test status for testing tab badge
  const [runningTests, setRunningTests] = useState(0);

  // Connect to real simulation data
  useEffect(() => {
    let ws = null;
    let reconnectTimeout = null;
    const reconnectDelay = 2000;

    const connectToSimulation = () => {
      if (ws && ws.readyState === WebSocket.OPEN) return;

      console.log('Connecting to simulation bridge...');
      ws = new WebSocket('ws://localhost:8766');

      ws.onopen = () => {
        console.log('✅ Connected to simulation framework');
        setSystemStatus(prev => ({ ...prev, can: 'operational' }));
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          if (data.type === 'simulation_update') {
            const simData = data.simulation_data;
            setTelemetry(prev => ({
              battery: prev.battery, // Keep existing battery for now (not from simulation)
              gps: {
                satellites: simData.gps?.satellites || prev.gps.satellites,
                hdop: simData.gps?.hdop || prev.gps.hdop,
                position: {
                  lat: simData.gps?.latitude || prev.gps.position.lat,
                  lon: simData.gps?.longitude || prev.gps.position.lon
                }
              },
              speed: simData.rover?.velocity ? Math.sqrt(
                simData.rover.velocity[0]**2 + simData.rover.velocity[1]**2
              ) : prev.speed,
              temperature: simData.environment?.temperature || simData.imu?.temperature || prev.temperature,
              timestamp: Date.now(),
              imu: simData.imu || prev.imu,
              environment: simData.environment || prev.environment
            }));
          }
        } catch (error) {
          console.warn('Failed to parse simulation data:', error);
        }
      };

      ws.onclose = () => {
        console.log('❌ Disconnected from simulation');
        setSystemStatus(prev => ({ ...prev, can: 'disconnected' }));

        // Attempt reconnection
        reconnectTimeout = setTimeout(() => {
          connectToSimulation();
        }, reconnectDelay);
      };

      ws.onerror = (error) => {
        console.error('Simulation WebSocket error:', error);
        setSystemStatus(prev => ({ ...prev, can: 'error' }));
      };
    };

    // Initial connection
    connectToSimulation();

    // Cleanup
    return () => {
      if (ws) ws.close();
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
    };
  }, []);

  // Update system status when connection changes
  useEffect(() => {
    setSystemStatus(prev => ({
      ...prev,
      websocket: isConnected ? 'connected' : 'disconnected'
    }));
  }, [isConnected]);

  // Update alerts based on system state
  useEffect(() => {
    const newAlerts = [];

    if (telemetry.gps.hdop > 2.0) {
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

    if (systemStatus.navigation === 'degraded') {
      newAlerts.push({
        id: 'nav_degraded',
        type: 'warning',
        message: 'Navigation system degraded',
        component: 'navigation',
        timestamp: Date.now()
      });
    }

    setAlerts(newAlerts);
    setErrorCount(newAlerts.filter(a => a.type === 'error').length);
  }, [telemetry, systemStatus]);

  // Get system state badge info
  const getStateBadge = useCallback(() => {
    const stateConfig = {
      'BOOT': { label: 'BOOT', color: 'blue', emoji: '🔵' },
      'IDLE': { label: 'IDLE', color: 'green', emoji: '🟢' },
      'AUTONOMOUS': { label: 'AUTONOMOUS', color: 'cyan', emoji: '🔵' },
      'TELEOPERATION': { label: 'TELEOP', color: 'yellow', emoji: '🟡' },
      'SAFESTOP': { label: 'SAFE STOP', color: 'orange', emoji: '🟠' },
      'SAFETY': { label: 'SAFETY', color: 'red', emoji: '🔴' },
      'SHUTDOWN': { label: 'SHUTDOWN', color: 'gray', emoji: '⚫' }
    };

    return stateConfig[currentState] || { label: currentState, color: 'gray', emoji: '⚪' };
  }, [currentState]);

  // Emergency stop handler
  const handleEmergencyStop = useCallback(async () => {
    try {
      await requestStateTransition('SAFETY', 'Emergency stop activated from UI');
    } catch (error) {
      console.error('Emergency stop failed:', error);
    }
  }, [requestStateTransition]);

  const value = {
    // Connection
    ros,
    isConnected,
    connectionStatus,

    // State
    currentState,
    currentSubstate,
    requestStateTransition,
    isTransitioning,
    getStateBadge,

    // Telemetry
    telemetry,
    setTelemetry,

    // System status
    systemStatus,
    setSystemStatus,

    // Mission
    activeMission,
    setActiveMission,

    // Alerts
    alerts,
    setAlerts,
    errorCount,

    // Testing
    runningTests,
    setRunningTests,

    // Offline support
    isOnline,
    offlineData,
    cacheDataForOffline,

    // Actions
    handleEmergencyStop
  };

  return (
    <SystemContext.Provider value={value}>
      {children}
    </SystemContext.Provider>
  );
};
