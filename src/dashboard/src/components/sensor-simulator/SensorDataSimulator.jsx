import React, { useState, useEffect, useCallback } from 'react';
import { ImuControls } from './ImuControls';
import { GpsControls } from './GpsControls';
import { BatteryControls } from './BatteryControls';
import { MotorControls } from './MotorControls';

/**
 * Simplified Sensor Data Simulator Component
 *
 * Broken into smaller, focused components for easier maintenance.
 */
export const SensorDataSimulator = ({
  onDataChange,
  isConnected,
  websocketUrl = 'ws://localhost:8765'
}) => {
  const [sensorData, setSensorData] = useState({
    imu: {
      accel_x: 0.0, accel_y: 0.0, accel_z: 9.81,
      gyro_x: 0.0, gyro_y: 0.0, gyro_z: 0.0,
      temperature: 25.0
    },
    gps: {
      latitude: 38.406, longitude: -110.792, altitude: 1500.0,
      heading: 0.0, speed: 0.0, satellites: 12, hdop: 0.8
    },
    battery: {
      voltage: 24.0, current: 5.0, charge_level: 85.0
    },
    motors: [
      { position: 0.0, velocity: 0.0 },
      { position: 0.0, velocity: 0.0 },
      { position: 0.0, velocity: 0.0 },
      { position: 0.0, velocity: 0.0 }
    ]
  });

  const [isPublishing, setIsPublishing] = useState(false);

  // Update parent when data changes
  useEffect(() => {
    onDataChange?.(sensorData);
  }, [sensorData, onDataChange]);

  // Auto-publish data when connected
  useEffect(() => {
    if (isConnected && !isPublishing) {
      setIsPublishing(true);
      const interval = setInterval(() => {
        onDataChange?.(sensorData);
      }, 100); // 10Hz publishing

      return () => {
        clearInterval(interval);
        setIsPublishing(false);
      };
    }
  }, [isConnected, sensorData, onDataChange, isPublishing]);

  const updateImu = useCallback((imuData) => {
    setSensorData(prev => ({ ...prev, imu: imuData }));
  }, []);

  const updateGps = useCallback((gpsData) => {
    setSensorData(prev => ({ ...prev, gps: gpsData }));
  }, []);

  const updateBattery = useCallback((batteryData) => {
    setSensorData(prev => ({ ...prev, battery: batteryData }));
  }, []);

  const updateMotors = useCallback((motorData) => {
    setSensorData(prev => ({ ...prev, motors: motorData }));
  }, []);

  return (
    <div className="space-y-6 p-4 bg-zinc-900 rounded-lg">
      <div className="flex items-center justify-between">
        <h2 className="text-lg font-semibold text-zinc-200">Sensor Simulator</h2>
        <div className={`px-2 py-1 rounded text-xs font-medium ${
          isConnected ? 'bg-green-900 text-green-300' : 'bg-red-900 text-red-300'
        }`}>
          {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
        </div>
      </div>

      {/* Mock data warning */}
      <div className="bg-yellow-900/20 border border-yellow-800 rounded p-3">
        <div className="text-sm text-yellow-400 font-medium mb-1">
          ⚠️ Mock Sensor Data
        </div>
        <div className="text-xs text-yellow-300">
          This simulates sensor inputs for testing. Not connected to real hardware.
        </div>
      </div>

      {/* Sensor controls in a grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <ImuControls imuData={sensorData.imu} onChange={updateImu} />
        <GpsControls gpsData={sensorData.gps} onChange={updateGps} />
        <BatteryControls batteryData={sensorData.battery} onChange={updateBattery} />
        <MotorControls motorData={sensorData.motors} onChange={updateMotors} />
      </div>

      {/* Publishing status */}
      <div className="text-xs text-zinc-500 text-center">
        {isPublishing ? '📡 Publishing at 10Hz' : '⏸️ Not publishing'}
      </div>
    </div>
  );
};
