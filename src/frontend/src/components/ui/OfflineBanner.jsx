import React from 'react';
import { WifiOff, RefreshCw } from 'lucide-react';

/**
 * Offline Banner Component
 *
 * Displays when the dashboard loses connection to the rover system.
 * Provides user feedback and options for reconnection.
 */
export const OfflineBanner = ({ isOnline, onRetry, lastSeen }) => {
  if (isOnline) return null;

  const getLastSeenText = () => {
    if (!lastSeen) return 'Never';

    const now = Date.now();
    const diff = now - lastSeen;
    const minutes = Math.floor(diff / 60000);
    const seconds = Math.floor((diff % 60000) / 1000);

    if (minutes > 0) {
      return `${minutes}m ${seconds}s ago`;
    }
    return `${seconds}s ago`;
  };

  return (
    <div className="bg-red-900/90 border-b border-red-500/50 px-4 py-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-3">
          <WifiOff className="w-5 h-5 text-red-400 animate-pulse" />
          <div>
            <p className="text-red-200 font-medium">
              Connection Lost - Operating in Offline Mode
            </p>
            <p className="text-red-300 text-sm">
              Last connected: {getLastSeenText()}
            </p>
          </div>
        </div>

        <div className="flex items-center gap-3">
          <div className="text-xs text-red-300">
            Some features may be limited
          </div>
          <button
            onClick={onRetry}
            className="flex items-center gap-2 bg-red-700 hover:bg-red-600 text-white px-3 py-1 rounded text-sm transition-colors"
          >
            <RefreshCw className="w-4 h-4" />
            Retry Connection
          </button>
        </div>
      </div>

      <div className="mt-2 text-xs text-red-300">
        <p>
          <strong>Limited functionality:</strong> Real-time telemetry unavailable.
          Mission commands will be queued and sent when connection is restored.
        </p>
      </div>
    </div>
  );
};
