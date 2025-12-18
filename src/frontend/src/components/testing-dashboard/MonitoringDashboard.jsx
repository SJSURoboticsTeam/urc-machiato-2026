import React, { useState, useEffect, useCallback } from 'react';
import { Activity, Wifi, WifiOff, AlertTriangle, CheckCircle, Clock } from 'lucide-react';
import { useROS } from '../../hooks/useROS';

export const MonitoringDashboard = () => {
  const { ros, isConnected } = useROS();
  const [systemHealth, setSystemHealth] = useState({
    communication: { status: 'unknown', lastUpdate: null },
    websocket: { status: 'unknown', lastUpdate: null },
    ros2: { status: 'unknown', lastUpdate: null },
    bridges: []
  });
  const [alerts, setAlerts] = useState([]);

  // Subscribe to communication health
  useEffect(() => {
    if (!ros || !isConnected) return;

    const healthSubscriber = new window.ROSLIB.Topic({
      ros: ros,
      name: '/system/communication_health',
      messageType: 'std_msgs/String'
    });

    healthSubscriber.subscribe((message) => {
      try {
        const healthData = JSON.parse(message.data);
        updateSystemHealth(healthData);
      } catch (error) {
        console.error('Failed to parse health data:', error);
      }
    });

    return () => {
      healthSubscriber.unsubscribe();
    };
  }, [ros, isConnected]);

  const updateSystemHealth = useCallback((healthData) => {
    const now = Date.now();
    const newHealth = { ...systemHealth };

    // Update communication status
    newHealth.communication = {
      status: healthData.failover_active ? 'degraded' : 'healthy',
      lastUpdate: now,
      channel: healthData.current_channel,
      failover: healthData.failover_active
    };

    // Update WebSocket status
    newHealth.websocket = {
      status: healthData.websocket_healthy ? 'healthy' : 'unhealthy',
      lastUpdate: now,
      age: healthData.websocket_age_seconds
    };

    // Update ROS2 status
    newHealth.ros2 = {
      status: isConnected ? 'healthy' : 'unhealthy',
      lastUpdate: now
    };

    setSystemHealth(newHealth);

    // Generate alerts
    const newAlerts = [];
    if (healthData.failover_active) {
      newAlerts.push({
        id: 'failover_active',
        level: 'warning',
        message: 'Communication failover active - using ROS2 direct',
        timestamp: now
      });
    }

    if (!healthData.websocket_healthy) {
      newAlerts.push({
        id: 'websocket_down',
        level: 'error',
        message: 'WebSocket bridge unhealthy',
        timestamp: now
      });
    }

    setAlerts(newAlerts);
  }, [systemHealth, isConnected]);

  const getStatusColor = (status) => {
    switch (status) {
      case 'healthy': return 'text-green-400';
      case 'degraded': return 'text-yellow-400';
      case 'unhealthy': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  const getStatusIcon = (status) => {
    switch (status) {
      case 'healthy': return <CheckCircle className="w-5 h-5" />;
      case 'degraded': return <AlertTriangle className="w-5 h-5" />;
      case 'unhealthy': return <WifiOff className="w-5 h-5" />;
      default: return <Clock className="w-5 h-5" />;
    }
  };

  const formatTime = (timestamp) => {
    if (!timestamp) return 'Never';
    const age = Date.now() - timestamp;
    if (age < 1000) return 'Just now';
    if (age < 60000) return `${Math.floor(age / 1000)}s ago`;
    return `${Math.floor(age / 60000)}m ago`;
  };

  return (
    <div className="space-y-6">
      <div className="flex items-center gap-3">
        <Activity className="w-6 h-6 text-blue-400" />
        <h2 className="text-xl font-bold text-zinc-200">System Monitoring</h2>
      </div>

      {/* Status Overview */}
      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div className="bg-zinc-800 p-4 rounded-lg">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-zinc-400">Communication</span>
            <div className={`flex items-center gap-2 ${getStatusColor(systemHealth.communication.status)}`}>
              {getStatusIcon(systemHealth.communication.status)}
            </div>
          </div>
          <div className="text-xs text-zinc-500">
            Channel: {systemHealth.communication.channel || 'unknown'}
            {systemHealth.communication.failover && (
              <span className="text-yellow-400 ml-2">(FAILOVER)</span>
            )}
          </div>
          <div className="text-xs text-zinc-600 mt-1">
            Updated: {formatTime(systemHealth.communication.lastUpdate)}
          </div>
        </div>

        <div className="bg-zinc-800 p-4 rounded-lg">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-zinc-400">WebSocket Bridge</span>
            <div className={`flex items-center gap-2 ${getStatusColor(systemHealth.websocket.status)}`}>
              {getStatusIcon(systemHealth.websocket.status)}
            </div>
          </div>
          <div className="text-xs text-zinc-500">
            Age: {systemHealth.websocket.age ? `${systemHealth.websocket.age.toFixed(1)}s` : 'N/A'}
          </div>
          <div className="text-xs text-zinc-600 mt-1">
            Updated: {formatTime(systemHealth.websocket.lastUpdate)}
          </div>
        </div>

        <div className="bg-zinc-800 p-4 rounded-lg">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-zinc-400">ROS2 Connection</span>
            <div className={`flex items-center gap-2 ${getStatusColor(systemHealth.ros2.status)}`}>
              {getStatusIcon(systemHealth.ros2.status)}
            </div>
          </div>
          <div className="text-xs text-zinc-500">
            Status: {isConnected ? 'Connected' : 'Disconnected'}
          </div>
          <div className="text-xs text-zinc-600 mt-1">
            Updated: {formatTime(systemHealth.ros2.lastUpdate)}
          </div>
        </div>
      </div>

      {/* Active Alerts */}
      {alerts.length > 0 && (
        <div className="bg-zinc-800 p-4 rounded-lg">
          <h3 className="text-lg font-semibold text-zinc-200 mb-3 flex items-center gap-2">
            <AlertTriangle className="w-5 h-5 text-yellow-400" />
            Active Alerts
          </h3>
          <div className="space-y-2">
            {alerts.map((alert) => (
              <div key={alert.id} className={`p-3 rounded flex items-start gap-3 ${
                alert.level === 'error' ? 'bg-red-900/20 border border-red-700' :
                alert.level === 'warning' ? 'bg-yellow-900/20 border border-yellow-700' :
                'bg-blue-900/20 border border-blue-700'
              }`}>
                <AlertTriangle className={`w-4 h-4 mt-0.5 ${
                  alert.level === 'error' ? 'text-red-400' :
                  alert.level === 'warning' ? 'text-yellow-400' :
                  'text-blue-400'
                }`} />
                <div className="flex-1">
                  <div className="text-sm text-zinc-200">{alert.message}</div>
                  <div className="text-xs text-zinc-500 mt-1">
                    {formatTime(alert.timestamp)}
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* System Bridges Status */}
      <div className="bg-zinc-800 p-4 rounded-lg">
        <h3 className="text-lg font-semibold text-zinc-200 mb-3">Bridge Status</h3>
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          {['State Machine', 'Mission Control', 'Simulation', 'Communication'].map((bridge) => (
            <div key={bridge} className="text-center">
              <div className="text-sm font-medium text-zinc-400 mb-1">{bridge}</div>
              <div className="flex items-center justify-center gap-2">
                <div className="w-2 h-2 bg-green-400 rounded-full"></div>
                <span className="text-xs text-zinc-500">Online</span>
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Quick Actions */}
      <div className="bg-zinc-800 p-4 rounded-lg">
        <h3 className="text-lg font-semibold text-zinc-200 mb-3">Quick Actions</h3>
        <div className="flex flex-wrap gap-3">
          <button className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded text-sm transition-colors">
            Test Communication
          </button>
          <button className="px-4 py-2 bg-zinc-600 hover:bg-zinc-700 text-zinc-200 rounded text-sm transition-colors">
            Reset Monitoring
          </button>
          <button className="px-4 py-2 bg-green-600 hover:bg-green-700 text-white rounded text-sm transition-colors">
            Force Failover Test
          </button>
        </div>
      </div>
    </div>
  );
};
