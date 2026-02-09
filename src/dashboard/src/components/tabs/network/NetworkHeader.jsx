import React, { memo } from 'react';
import { Network, Wifi, Zap, Cpu } from 'lucide-react';
import { getStatusTextColor } from '../../../utils/statusUtils';

/**
 * Network tab header with connection status. Uses centralized status colors.
 */
export const NetworkHeader = memo(function NetworkHeader({ connections }) {
  const wsColor = getStatusTextColor(
    connections.websocket === 'operational' ? 'connected' : 'disconnected'
  );
  const canColor = getStatusTextColor(
    connections.can === 'operational' ? 'connected' : 'degraded'
  );
  const rosColor = getStatusTextColor(
    connections.ros2 === 'operational' ? 'connected' : 'degraded'
  );

  return (
    <div className="bg-zinc-900 border-b border-zinc-800 p-4">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-3">
          <Network className="w-5 h-5 text-cyan-400" />
          <div>
            <h1 className="text-lg font-semibold text-zinc-100">Network Topology</h1>
            <p className="text-sm text-zinc-400">
              Real-time component interactions and data flow
            </p>
          </div>
        </div>

        <div className="flex items-center gap-4 text-xs">
          <div className="flex items-center gap-2">
            <Wifi className={`w-3 h-3 ${wsColor}`} />
            <span className="text-zinc-400">WebSocket</span>
          </div>
          <div className="flex items-center gap-2">
            <Zap className={`w-3 h-3 ${canColor}`} />
            <span className="text-zinc-400">CAN</span>
          </div>
          <div className="flex items-center gap-2">
            <Cpu className={`w-3 h-3 ${rosColor}`} />
            <span className="text-zinc-400">ROS2</span>
          </div>
        </div>
      </div>
    </div>
  );
});
