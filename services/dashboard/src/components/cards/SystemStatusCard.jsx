import React, { memo } from 'react';
import { CheckCircle2, XCircle, AlertCircle, Loader2 } from 'lucide-react';
import { getStatusTextColor } from '../../utils/statusUtils';

/**
 * SystemStatusCard Component
 *
 * Displays system status with color-coded indicators. Memoized for stable props.
 */
export const SystemStatusCard = memo(function SystemStatusCard({
  systems,
  title = 'System Status',
  showDetails = false
}) {
  const getStatusIcon = (status) => {
    const colorClass = getStatusTextColor(status);
    switch (String(status).toLowerCase()) {
      case 'ok':
      case 'ready':
      case 'operational':
        return <CheckCircle2 className={`w-4 h-4 ${colorClass}`} />;
      case 'degraded':
      case 'warning':
        return <AlertCircle className={`w-4 h-4 ${colorClass}`} />;
      case 'error':
      case 'failed':
        return <XCircle className={`w-4 h-4 ${colorClass}`} />;
      case 'busy':
      case 'active':
        return <Loader2 className={`w-4 h-4 ${colorClass} animate-spin`} />;
      default:
        return <AlertCircle className={`w-4 h-4 ${colorClass}`} />;
    }
  };

  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <h3 className="text-sm font-semibold text-zinc-200 mb-3">{title}</h3>
      <div className="space-y-2">
        {Object.entries(systems).map(([key, status]) => (
          <div key={key} className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              {getStatusIcon(status)}
              <span className={`text-sm capitalize ${getStatusTextColor(status)}`}>{key}</span>
            </div>
            <span className={`text-xs font-medium ${getStatusTextColor(status)}`}>
              {status.toUpperCase()}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
});
