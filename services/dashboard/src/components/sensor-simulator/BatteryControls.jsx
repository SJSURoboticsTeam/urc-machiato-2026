import React from 'react';

/**
 * Battery Sensor Controls Component
 * Handles battery voltage, current, and charge simulation
 */
export const BatteryControls = ({ batteryData, onChange }) => {
  const updateBattery = (field, value) => {
    onChange({
      ...batteryData,
      [field]: parseFloat(value) || 0
    });
  };

  return (
    <div className="space-y-4">
      <h3 className="text-sm font-semibold text-zinc-200">Battery System</h3>

      {/* Voltage & Current */}
      <div className="space-y-2">
        <div className="grid grid-cols-2 gap-2">
          <div>
            <label className="text-xs text-zinc-400">Voltage (V)</label>
            <input
              type="number"
              step="0.1"
              min="0"
              max="30"
              value={batteryData.voltage}
              onChange={(e) => updateBattery('voltage', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-400">Current (A)</label>
            <input
              type="number"
              step="0.1"
              min="-50"
              max="50"
              value={batteryData.current}
              onChange={(e) => updateBattery('current', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
        </div>
      </div>

      {/* Charge Level */}
      <div className="space-y-2">
        <label className="text-xs text-zinc-400">Charge Level (%)</label>
        <input
          type="range"
          min="0"
          max="100"
          value={batteryData.charge_level}
          onChange={(e) => updateBattery('charge_level', e.target.value)}
          className="w-full"
        />
        <div className="text-xs text-zinc-500 text-center">
          {batteryData.charge_level}%
        </div>
      </div>

      {/* Status Display */}
      <div className="p-2 bg-zinc-800 rounded text-xs">
        <div className="text-zinc-400">Power: {(batteryData.voltage * batteryData.current).toFixed(1)}W</div>
        <div className={`font-medium ${
          batteryData.charge_level > 20 ? 'text-green-400' :
          batteryData.charge_level > 10 ? 'text-yellow-400' : 'text-red-400'
        }`}>
          Status: {
            batteryData.charge_level > 20 ? 'Good' :
            batteryData.charge_level > 10 ? 'Low' : 'Critical'
          }
        </div>
      </div>
    </div>
  );
};
