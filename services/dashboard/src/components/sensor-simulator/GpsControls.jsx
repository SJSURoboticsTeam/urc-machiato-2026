import React from 'react';

/**
 * GPS Sensor Controls Component
 * Handles GPS position, heading, speed simulation
 */
export const GpsControls = ({ gpsData, onChange }) => {
  const updateGps = (field, value) => {
    onChange({
      ...gpsData,
      [field]: parseFloat(value) || 0
    });
  };

  return (
    <div className="space-y-4">
      <h3 className="text-sm font-semibold text-zinc-200">GPS Position</h3>

      {/* Position */}
      <div className="space-y-2">
        <div className="grid grid-cols-2 gap-2">
          <div>
            <label className="text-xs text-zinc-400">Latitude</label>
            <input
              type="number"
              step="0.001"
              value={gpsData.latitude}
              onChange={(e) => updateGps('latitude', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-400">Longitude</label>
            <input
              type="number"
              step="0.001"
              value={gpsData.longitude}
              onChange={(e) => updateGps('longitude', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
        </div>
      </div>

      {/* Altitude & Heading */}
      <div className="space-y-2">
        <div className="grid grid-cols-2 gap-2">
          <div>
            <label className="text-xs text-zinc-400">Altitude (m)</label>
            <input
              type="number"
              step="0.1"
              value={gpsData.altitude}
              onChange={(e) => updateGps('altitude', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-400">Heading (°)</label>
            <input
              type="number"
              step="1"
              min="0"
              max="360"
              value={gpsData.heading}
              onChange={(e) => updateGps('heading', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
        </div>
      </div>

      {/* Speed & Quality */}
      <div className="space-y-2">
        <div className="grid grid-cols-3 gap-2">
          <div>
            <label className="text-xs text-zinc-400">Speed (m/s)</label>
            <input
              type="number"
              step="0.1"
              min="0"
              value={gpsData.speed}
              onChange={(e) => updateGps('speed', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-400">Satellites</label>
            <input
              type="number"
              min="0"
              max="20"
              value={gpsData.satellites}
              onChange={(e) => updateGps('satellites', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
          <div>
            <label className="text-xs text-zinc-400">HDOP</label>
            <input
              type="number"
              step="0.1"
              min="0.5"
              max="10"
              value={gpsData.hdop}
              onChange={(e) => updateGps('hdop', e.target.value)}
              className="w-full px-2 py-1 bg-zinc-800 border border-zinc-600 rounded text-xs text-zinc-200"
            />
          </div>
        </div>
      </div>
    </div>
  );
};
