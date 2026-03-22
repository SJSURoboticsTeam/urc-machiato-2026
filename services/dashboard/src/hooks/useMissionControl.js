import { useUIContext } from '../context/UIContext';

/**
 * Focused hook for mission UI state (active mission, setter).
 * Wraps UIContext for mission-related components.
 */
export function useMissionControl() {
  const { activeMission, setActiveMission } = useUIContext();
  return { activeMission, setActiveMission };
}
