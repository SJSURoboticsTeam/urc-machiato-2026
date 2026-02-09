import { useUIContext } from '../context/UIContext';

/**
 * Focused hook for alerts and error count.
 * Wraps UIContext so components can subscribe only to alert-related state.
 */
export function useAlerts() {
  const { alerts, setAlerts, errorCount } = useUIContext();
  return { alerts, setAlerts, errorCount };
}
