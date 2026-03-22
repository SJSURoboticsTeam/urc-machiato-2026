import React from 'react';

/**
 * IMU Sensor Controls Component
 * Handles accelerometer, gyroscope, and temperature simulation
 */
export const ImuControls = ({ imuData, onChange }) => {
  const updateImu = (field, value) => {
    onChange({
      ...imuData,
      [field]: parseFloat(value) || 0
    });
  };

  return (
    <div className="space-y-4">
      <h3 className="text-sm font-semibold text-zinc-200">IMU Sensors</h3>

      {/* Accelerometer */}
      <div className="space-y-2">
        <label className="text-xs text-zinc-400">Accelerometer (m/s²)</label>
        <div className="grid grid-cols-3 gap-2">
          <div>
            <label className="text-xs text-zinc-500">X</label>
            <input
              type="number"
              step="0.01"
              value={imuData.accel_x}
              onChange={(e) => updateImu('accel_x', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-500">Y</label>
            <input
              type="number"
              step="0.01"
              value={imuData.accel_y}
              onChange={(e) => updateImu('accel_y', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-500">Z</label>
            <input
              type="number"
              step="0.01"
              value={imuData.accel_z}
              onChange={(e) => updateImu('accel_z', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
        </div>
      </div>

      {/* Gyroscope */}
      <div className="space-y-2">
        <label className="text-xs text-zinc-400">Gyroscope (°/s)</label>
        <div className="grid grid-cols-3 gap-2">
          <div>
            <label className="text-xs text-zinc-500">X</label>
            <input
              type="number"
              step="0.1"
              value={imuData.gyro_x}
              onChange={(e) => updateImu('gyro_x', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-500">Y</label>
            <input
              type="number"
              step="0.1"
              value={imuData.gyro_y}
              onChange={(e) => updateImu('gyro_y', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-500">Z</label>
            <input
              type="number"
              step="0.1"
              value={imuData.gyro_z}
              onChange={(e) => updateImu('gyro_z', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
        </div>
      </div>

      {/* Temperature */}
      <div className="space-y-2">
        <label className="text-xs text-zinc-400">Temperature (°C)</label>
        <input
          type="number"
          step="0.1"
          value={imuData.temperature}
          onChange={(e) => updateImu('temperature', e.target.value)}
          className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
        />
      </div>
    </div>
  );
};
