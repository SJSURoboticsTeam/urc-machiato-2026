import React from 'react';
import { Activity, Zap, Radio, MessageSquare } from 'lucide-react';

/**
 * Communication Metrics Component
 * Shows real-time communication statistics across all channels
 */
export const CommunicationMetrics = ({ commMetrics }) => {
  const getChannelIcon = (channel) => {
    switch (channel) {
      case 'websocket': return <Radio className="w-4 h-4" />;
      case 'ros2': return <Activity className="w-4 h-4" />;
      case 'can': return <Zap className="w-4 h-4" />;
      default: return <MessageSquare className="w-4 h-4" />;
    }
  };

  const getChannelColor = (channel) => {
    switch (channel) {
      case 'websocket': return 'text-blue-400';
      case 'ros2': return 'text-green-400';
      case 'can': return 'text-purple-400';
      default: return 'text-zinc-400';
    }
  };

  const getStatusIndicator = (active, isMock = false) => {
    if (isMock) {
      return <div className="w-2 h-2 bg-yellow-400 rounded-full" title="Mock/Simulated" />;
    }
    return (
      <div className={`w-2 h-2 rounded-full ${active ? 'bg-green-400' : 'bg-red-400'}`} />
    );
  };

  const totalSent = Object.values(commMetrics).reduce((sum, channel) => sum + channel.sent, 0);
  const totalReceived = Object.values(commMetrics).reduce((sum, channel) => sum + channel.received, 0);

  return (
    <div className="space-y-4">
      <h3 className="text-lg font-semibold text-zinc-200 flex items-center gap-2">
        <Activity className="w-5 h-5" />
        Communication Metrics
      </h3>

      {/* Overall stats */}
      <div className="grid grid-cols-2 gap-4">
        <div className="bg-zinc-800 p-3 rounded">
          <div className="text-sm text-zinc-400 mb-1">Total Sent</div>
          <div className="text-2xl font-bold text-blue-400">{totalSent}</div>
        </div>
        <div className="bg-zinc-800 p-3 rounded">
          <div className="text-sm text-zinc-400 mb-1">Total Received</div>
          <div className="text-2xl font-bold text-green-400">{totalReceived}</div>
        </div>
      </div>

      {/* Channel breakdown */}
      <div className="space-y-3">
        <div className="text-sm text-zinc-400">Channel Details</div>
        {Object.entries(commMetrics).map(([channel, metrics]) => (
          <div key={channel} className="bg-zinc-800 p-3 rounded">
            <div className="flex items-center justify-between mb-2">
              <div className="flex items-center gap-2">
                <span className={getChannelColor(channel)}>
                  {getChannelIcon(channel)}
                </span>
                <span className="font-medium text-zinc-200 capitalize">
                  {channel}
                </span>
                {getStatusIndicator(metrics.active, channel === 'can' && metrics.isMock)}
              </div>
              <div className="text-xs text-zinc-500">
                {metrics.active ? 'Active' : 'Inactive'}
              </div>
            </div>

            <div className="grid grid-cols-2 gap-4 text-sm">
              <div>
                <div className="text-zinc-400">Sent</div>
                <div className="text-blue-400 font-mono">{metrics.sent}</div>
              </div>
              <div>
                <div className="text-zinc-400">Received</div>
                <div className="text-green-400 font-mono">{metrics.received}</div>
              </div>
            </div>

            {metrics.lastMsg && (
              <div className="mt-2 pt-2 border-t border-zinc-700">
                <div className="text-xs text-zinc-400 mb-1">Last Message</div>
                <div className="text-xs text-zinc-300 font-mono truncate">
                  {metrics.lastMsg}
                </div>
              </div>
            )}
          </div>
        ))}
      </div>

      {/* Connection status summary */}
      <div className="bg-zinc-800 p-3 rounded">
        <div className="text-sm text-zinc-400 mb-2">Connection Summary</div>
        <div className="flex gap-2">
          {Object.entries(commMetrics).map(([channel, metrics]) => (
            <div key={channel} className="flex items-center gap-1">
              <span className={getChannelColor(channel)}>
                {getChannelIcon(channel)}
              </span>
              <span className="text-xs capitalize">{channel}</span>
              {getStatusIndicator(metrics.active, channel === 'can' && metrics.isMock)}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
