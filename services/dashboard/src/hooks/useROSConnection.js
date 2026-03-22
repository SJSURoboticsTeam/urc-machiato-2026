import { useROSContext } from '../context/ROSContext';

/**
 * Focused hook for ROS connection state.
 * Wraps ROSContext for components that only need connection status.
 */
export function useROSConnection() {
  const { ros, isConnected, connectionStatus } = useROSContext();
  return { ros, isConnected, connectionStatus };
}
