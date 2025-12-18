import React from 'react';

/**
 * Motor Encoder Controls Component
 * Handles motor position and velocity simulation
 */
export const MotorControls = ({ motorData, onChange }) => {
  const updateMotor = (motorId, field, value) => {
    const updatedMotors = [...motorData];
    updatedMotors[motorId] = {
      ...updatedMotors[motorId],
      [field]: parseFloat(value) || 0
    };
    onChange(updatedMotors);
  };

  return (
    <div className="space-y-4">
      <h3 className="text-sm font-semibold text-zinc-200">Motor Encoders</h3>

      {motorData.map((motor, index) => (
        <div key={index} className="space-y-2 p-3 bg-zinc-800 rounded">
          <h4 className="text-xs font-medium text-zinc-300">Motor {index + 1}</h4>

          <div className="grid grid-cols-2 gap-2">
            <div>
              <label className="text-xs text-zinc-400">Position (rad)</label>
              <input
                type="number"
                step="0.01"
                value={motor.position}
                onChange={(e) => updateMotor(index, 'position', e.target.value)}
                className="w-full px-2 py-1 bg-zinc-900 border border-zinc-600 rounded text-xs text-zinc-200"
              />
            </div>
            <div>
              <label className="text-xs text-zinc-400">Velocity (rad/s)</label>
              <input
                type="number"
                step="0.01"
                value={motor.velocity}
                onChange={(e) => updateMotor(index, 'velocity', e.target.value)}
                className="w-full px-2 py-1 bg-zinc-900 border border-zinc-600 rounded text-xs text-zinc-200"
              />
            </div>
          </div>

          {/* Simple visualization */}
          <div className="flex items-center space-x-2 mt-2">
            <div className="text-xs text-zinc-500">Speed:</div>
            <div className="flex-1 bg-zinc-700 rounded-full h-2">
              <div
                className="bg-blue-500 h-2 rounded-full transition-all duration-200"
                style={{
                  width: `${Math.min(Math.abs(motor.velocity) * 10, 100)}%`,
                  backgroundColor: motor.velocity > 0 ? '#3b82f6' : '#ef4444'
                }}
              />
            </div>
            <div className="text-xs text-zinc-500">
              {motor.velocity > 0 ? '+' : ''}{motor.velocity.toFixed(1)}
            </div>
          </div>
        </div>
      ))}

      {/* Quick controls */}
      <div className="flex space-x-2">
        <button
          onClick={() => onChange(motorData.map(() => ({ position: 0, velocity: 0 })))}
          className="px-3 py-1 bg-zinc-700 hover:bg-zinc-600 text-zinc-200 text-xs rounded"
        >
          Stop All
        </button>
        <button
          onClick={() => onChange(motorData.map(() => ({ position: 0, velocity: 1.0 })))}
          className="px-3 py-1 bg-zinc-700 hover:bg-zinc-600 text-zinc-200 text-xs rounded"
        >
          Forward
        </button>
      </div>
    </div>
  );
};
