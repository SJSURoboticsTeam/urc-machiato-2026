import React from 'react';
import { CheckCircle2, XCircle, AlertTriangle, Wifi, WifiOff } from 'lucide-react';
import { getStatusClasses } from '../../utils/uiUtils';

/**
 * Component Status Overview Component
 * Shows the current status of all system components
 */
export const ComponentStatusOverview = ({
  systemStatus,
  isConnected
}) => {
  const components = [
    { id: 'safety', name: 'Safety System', icon: '🛡️', description: 'Emergency stops and monitoring' },
    { id: 'navigation', name: 'Navigation', icon: '🧭', description: 'Path planning and waypoint navigation' },
    { id: 'vision', name: 'Computer Vision', icon: '👁️', description: 'ArUco detection and object recognition' },
    { id: 'can', name: 'CAN Bus', icon: '🔌', description: 'Sensor data communication' },
    { id: 'websocket', name: 'WebSocket', icon: '🌐', description: 'Real-time data streaming' },
    { id: 'frontend', name: 'Frontend', icon: '💻', description: 'User interface and controls' }
  ];

  const getStatusIcon = (status) => {
    switch (status) {
      case 'operational':
      case 'connected':
        return <CheckCircle2 className="w-5 h-5 text-green-600" />;
      case 'disconnected':
        return <XCircle className="w-5 h-5 text-red-600" />;
      case 'mock':
        return <AlertTriangle className="w-5 h-5 text-orange-600" />;
      default:
        return <WifiOff className="w-5 h-5 text-gray-600" />;
    }
  };

  const getStatusBadge = (status) => {
    const baseClasses = "px-2 py-1 rounded-full text-xs font-medium";
    switch (status) {
      case 'operational':
        return `${baseClasses} bg-green-100 text-green-800`;
      case 'connected':
        return `${baseClasses} bg-green-100 text-green-800`;
      case 'disconnected':
        return `${baseClasses} bg-red-100 text-red-800`;
      case 'mock':
        return `${baseClasses} bg-orange-100 text-orange-800`;
      case 'demo':
        return `${baseClasses} bg-yellow-100 text-yellow-800`;
      default:
        return `${baseClasses} bg-gray-100 text-gray-800`;
    }
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-xl font-semibold text-gray-900">System Overview</h2>
          <p className="text-sm text-gray-600 mt-1">
            Real-time status of all URC 2026 system components
          </p>
        </div>
        <div className={`flex items-center gap-2 px-3 py-2 rounded-lg ${getStatusClasses(isConnected ? 'connected' : 'disconnected')}`}>
          {isConnected ? <Wifi className="w-4 h-4" /> : <WifiOff className="w-4 h-4" />}
          <span className="text-sm font-medium">
            {isConnected ? 'ROS2 Connected' : 'ROS2 Disconnected'}
          </span>
        </div>
      </div>

      {/* Component Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {components.map(component => {
          const status = systemStatus[component.id] || 'unknown';
          return (
            <div key={component.id} className="bg-white rounded-lg border border-gray-200 p-4 hover:shadow-md transition-shadow">
              <div className="flex items-start justify-between mb-3">
                <div className="flex items-center gap-3">
                  <span className="text-2xl">{component.icon}</span>
                  <div>
                    <h3 className="font-medium text-gray-900">{component.name}</h3>
                    <p className="text-sm text-gray-600">{component.description}</p>
                  </div>
                </div>
                {getStatusIcon(status)}
              </div>

              <div className="flex items-center justify-between">
                <span className="text-sm text-gray-500">Status</span>
                <span className={getStatusBadge(status)}>
                  {status.charAt(0).toUpperCase() + status.slice(1)}
                </span>
              </div>
            </div>
          );
        })}
      </div>

      {/* System Health Summary */}
      <div className="bg-white rounded-lg border border-gray-200 p-6">
        <h3 className="text-lg font-semibold text-gray-900 mb-4">System Health</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <div className="text-center">
            <div className="text-2xl font-bold text-green-600">
              {Object.values(systemStatus).filter(s => s === 'operational' || s === 'connected').length}
            </div>
            <div className="text-sm text-gray-600">Operational</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-orange-600">
              {Object.values(systemStatus).filter(s => s === 'mock').length}
            </div>
            <div className="text-sm text-gray-600">Mock/Test</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-red-600">
              {Object.values(systemStatus).filter(s => s === 'disconnected').length}
            </div>
            <div className="text-sm text-gray-600">Disconnected</div>
          </div>
          <div className="text-center">
            <div className="text-2xl font-bold text-gray-600">
              {Object.values(systemStatus).filter(s => s === 'unknown').length}
            </div>
            <div className="text-sm text-gray-600">Unknown</div>
          </div>
        </div>
      </div>
    </div>
  );
};
