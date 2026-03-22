import React, { useState, useEffect, useCallback } from 'react';
import ROSLIB from '../../utils/rosbridge';
import { useROS } from '../../hooks/useROS';
import { parseAndValidate, communicationHealthSchema } from '../../utils/validationSchemas';
import { getStatusTextColor } from '../../utils/statusUtils';
import { formatRelativeTime } from '../../utils/formatting';
import { Radio, WifiOff, AlertTriangle, CheckCircle, Clock } from 'lucide-react';

/**
 * Communication tab: channels, health, WebSocket/ROS2 status, alerts.
 * Replaces the former Monitor tab.
 */
export function CommunicationTab() {
  const { ros, isConnected } = useROS();
  const [systemHealth, setSystemHealth] = useState({
    communication: { status: 'unknown', lastUpdate: null, channel: null, failover: false },
    websocket: { status: 'unknown', lastUpdate: null, age: null },
    ros2: { status: 'unknown', lastUpdate: null }
  });
  const [alerts, setAlerts] = useState([]);

  const updateSystemHealth = useCallback((healthData) => {
    const now = Date.now();
    setSystemHealth((prev) => {
      const next = { ...prev };
      next.communication = {
        status: healthData.failover_active ? 'degraded' : 'healthy',
        lastUpdate: now,
        channel: healthData.current_channel,
        failover: healthData.failover_active
      };
      next.websocket = {
        status: healthData.websocket_healthy ? 'healthy' : 'unhealthy',
        lastUpdate: now,
        age: healthData.websocket_age_seconds
      };
      next.ros2 = {
        status: isConnected ? 'healthy' : 'unhealthy',
        lastUpdate: now
      };
      return next;
    });
    const newAlerts = [];
    if (healthData.failover_active) {
      newAlerts.push({ id: 'failover_active', level: 'warning', message: 'Communication failover active - using ROS2 direct', timestamp: now });
    }
    if (!healthData.websocket_healthy) {
      newAlerts.push({ id: 'websocket_down', level: 'error', message: 'WebSocket bridge unhealthy', timestamp: now });
    }
    setAlerts(newAlerts);
  }, [isConnected]);

  useEffect(() => {
    if (!ros || !isConnected) return;
    const topic = new ROSLIB.Topic({
      ros,
      name: '/system/communication_health',
      messageType: 'std_msgs/String'
    });
    topic.subscribe((message) => {
      const healthData = parseAndValidate(message?.data, communicationHealthSchema);
      if (healthData) updateSystemHealth(healthData);
    });
    return () => topic.unsubscribe();
  }, [ros, isConnected, updateSystemHealth]);

  const getStatusIcon = (status) => {
    switch (status) {
      case 'healthy': return <CheckCircle className="w-5 h-5 text-emerald-400" />;
      case 'degraded': return <AlertTriangle className="w-5 h-5 text-amber-400" />;
      case 'unhealthy': return <WifiOff className="w-5 h-5 text-red-400" />;
      default: return <Clock className="w-5 h-5 text-zinc-500" />;
    }
  };

  return (
    <div className="p-4 space-y-6">
      <header>
        <h2 className="text-lg font-semibold text-zinc-100 flex items-center gap-2">
          <Radio className="w-5 h-5 text-blue-400" />
          Communication
        </h2>
        <p className="text-sm text-zinc-400 mt-0.5">
          Channel health, WebSocket and ROS2 status, and alerts.
        </p>
      </header>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-zinc-400">Communication</span>
            <div className={getStatusTextColor(systemHealth.communication.status)}>
              {getStatusIcon(systemHealth.communication.status)}
            </div>
          </div>
          <div className="text-xs text-zinc-500">
            Channel: {systemHealth.communication.channel ?? 'unknown'}
            {systemHealth.communication.failover && (
              <span className="text-amber-400 ml-2">(FAILOVER)</span>
            )}
          </div>
          <div className="text-xs text-zinc-600 mt-1">
            Updated: {formatRelativeTime(systemHealth.communication.lastUpdate)}
          </div>
        </div>

        <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-zinc-400">WebSocket Bridge</span>
            <div className={getStatusTextColor(systemHealth.websocket.status)}>
              {getStatusIcon(systemHealth.websocket.status)}
            </div>
          </div>
          <div className="text-xs text-zinc-500">
            Age: {systemHealth.websocket.age != null ? `${systemHealth.websocket.age.toFixed(1)}s` : 'N/A'}
          </div>
          <div className="text-xs text-zinc-600 mt-1">
            Updated: {formatRelativeTime(systemHealth.websocket.lastUpdate)}
          </div>
        </div>

        <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
          <div className="flex items-center justify-between mb-2">
            <span className="text-sm font-medium text-zinc-400">ROS2 Connection</span>
            <div className={getStatusTextColor(systemHealth.ros2.status)}>
              {getStatusIcon(systemHealth.ros2.status)}
            </div>
          </div>
          <div className="text-xs text-zinc-500">
            {isConnected ? 'Connected' : 'Disconnected'}
          </div>
          <div className="text-xs text-zinc-600 mt-1">
            Updated: {formatRelativeTime(systemHealth.ros2.lastUpdate)}
          </div>
        </div>
      </div>

      {alerts.length > 0 && (
        <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
          <h3 className="text-sm font-medium text-zinc-300 mb-3 flex items-center gap-2">
            <AlertTriangle className="w-4 h-4 text-amber-400" />
            Active Alerts
          </h3>
          <ul className="space-y-2">
            {alerts.map((alert) => (
              <li
                key={alert.id}
                className={`p-3 rounded flex items-start gap-3 ${
                  alert.level === 'error' ? 'bg-red-900/20 border border-red-700' :
                  alert.level === 'warning' ? 'bg-amber-900/20 border border-amber-700' :
                  'bg-zinc-800 border border-zinc-700'
                }`}
              >
                <AlertTriangle className={`w-4 h-4 mt-0.5 ${
                  alert.level === 'error' ? 'text-red-400' : 'text-amber-400'
                }`} />
                <div>
                  <div className="text-sm text-zinc-200">{alert.message}</div>
                  <div className="text-xs text-zinc-500 mt-1">{formatRelativeTime(alert.timestamp)}</div>
                </div>
              </li>
            ))}
          </ul>
        </div>
      )}

      {!isConnected && (
        <p className="text-sm text-zinc-500">Connect ROS to see communication health from /system/communication_health.</p>
      )}
    </div>
  );
}
