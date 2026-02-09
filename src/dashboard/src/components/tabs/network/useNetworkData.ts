import { useState, useEffect, useCallback } from 'react';
import { useROSContext } from '../../../context/ROSContext';
import ROSLIB from '../../../utils/rosbridge';
import {
  parseAndValidate,
  canSensorDataSchema,
  simulationMessageSchema,
  commandUpdateMessageSchema
} from '../../../utils/validationSchemas';
import { INITIAL_NETWORK_NODES } from './networkNodeConfig';
import type { NetworkNodesMap, NetworkConnectionsState, CanDataState } from '../../../types/network';

export interface UseNetworkDataReturn {
  networkNodes: NetworkNodesMap;
  connections: NetworkConnectionsState;
  selectedNode: string | null;
  setSelectedNode: (id: string | null) => void;
  canData: CanDataState;
}

const INITIAL_CONNECTIONS: NetworkConnectionsState = {
  websocket: 'disconnected',
  ros2: 'disconnected',
  can: 'disconnected'
};

const INITIAL_CAN_DATA: CanDataState = {
  imu: null,
  gps: null,
  motor_left: null,
  motor_right: null,
  bus_status: { messages_per_sec: 0, error_count: 0 }
};

export function useNetworkData(): UseNetworkDataReturn {
  const { ros, isConnected } = useROSContext();
  const [networkNodes, setNetworkNodes] = useState<NetworkNodesMap>(INITIAL_NETWORK_NODES);
  const [connections, setConnections] = useState<NetworkConnectionsState>(INITIAL_CONNECTIONS);
  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const [canData, setCanData] = useState<CanDataState>(INITIAL_CAN_DATA);

  const updateNetworkFromSimulation = useCallback((simulationData: Record<string, unknown>) => {
    setNetworkNodes((prev) => ({
      ...prev,
      imu_sensor: {
        ...prev.imu_sensor,
        data: simulationData.imu,
        lastUpdate: Date.now()
      },
      gps_sensor: {
        ...prev.gps_sensor,
        data: simulationData.gps,
        lastUpdate: Date.now()
      },
      battery_sensor: {
        ...prev.battery_sensor,
        data: simulationData.battery,
        lastUpdate: Date.now()
      },
      can_imu: {
        ...prev.can_imu,
        data:
          (simulationData.can as Record<string, unknown>)?.imu ?? {
            status: 'mock',
            value: Math.random() * 10
          },
        lastUpdate: Date.now()
      },
      can_gps: {
        ...prev.can_gps,
        data:
          (simulationData.can as Record<string, unknown>)?.gps ?? {
            status: 'mock',
            lat: 38.4,
            lon: -110.8
          },
        lastUpdate: Date.now()
      },
      can_motor_left: {
        ...prev.can_motor_left,
        data:
          (simulationData.can as Record<string, unknown>)?.motor_left ?? {
            status: 'mock',
            temp: 25 + Math.random() * 10
          },
        lastUpdate: Date.now()
      },
      can_motor_right: {
        ...prev.can_motor_right,
        data:
          (simulationData.can as Record<string, unknown>)?.motor_right ?? {
            status: 'mock',
            temp: 25 + Math.random() * 10
          },
        lastUpdate: Date.now()
      },
      can_bus: {
        ...prev.can_bus,
        data: {
          status: 'operational',
          messages_per_sec: Math.floor(Math.random() * 100),
          lastUpdate: Date.now()
        },
        lastUpdate: Date.now()
      }
    }));
  }, []);

  const updateNetworkFromCommands = useCallback((commandData: Record<string, unknown>) => {
    setNetworkNodes((prev) => ({
      ...prev,
      navigation_controller: {
        ...prev.navigation_controller,
        data: commandData.navigation,
        lastUpdate: Date.now()
      },
      motion_controller: {
        ...prev.motion_controller,
        data: commandData.motion,
        lastUpdate: Date.now()
      },
      left_motor: {
        ...prev.left_motor,
        data: commandData.left_motor,
        lastUpdate: Date.now()
      },
      right_motor: {
        ...prev.right_motor,
        data: commandData.right_motor,
        lastUpdate: Date.now()
      }
    }));
  }, []);

  // Use shared ROS connection from context (dashboard debugging; may move to teleop frontend)
  useEffect(() => {
    if (!ros || !isConnected) {
      setConnections((prev) => ({ ...prev, ros2: 'disconnected', can: 'disconnected' }));
      return;
    }
    setConnections((prev) => ({ ...prev, ros2: 'operational', can: 'operational' }));

    const canDataSubscriber = new ROSLIB.Topic({
      ros: ros as ROSLIB.Ros,
      name: '/can/sensor_data',
      messageType: 'std_msgs/String'
    });

    canDataSubscriber.subscribe((message: { data?: string }) => {
      const data = parseAndValidate(message?.data, canSensorDataSchema) as Record<string, unknown> | null;
      if (!data) return;
      const sensorType = (data.sensor_type as string) ?? 'unknown';
      setCanData((prev) => ({
        ...prev,
        [sensorType]: data,
        bus_status: {
          ...prev.bus_status,
          messages_per_sec: prev.bus_status.messages_per_sec + 1
        }
      }));

      setNetworkNodes((currentNodes) => ({
        ...currentNodes,
        [`can_${sensorType}`]: {
          ...currentNodes[`can_${sensorType}`],
          data,
          lastUpdate: Date.now()
        },
        can_bus: {
          ...currentNodes.can_bus,
          data: {
            status: 'operational',
            messages_per_sec: ((currentNodes.can_bus.data as { messages_per_sec?: number })?.messages_per_sec ?? 0) + 1,
            lastUpdate: Date.now()
          },
          lastUpdate: Date.now()
        }
      }));
    });

    return () => {
      canDataSubscriber.unsubscribe();
    };
  }, [ros, isConnected]);

  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8766');

    ws.onopen = () => {
      setConnections((prev) => ({ ...prev, websocket: 'operational' }));
    };

    ws.onmessage = (event: MessageEvent) => {
      try {
        const raw = typeof event.data === 'string' ? event.data : null;
        if (!raw) return;
        const sim = parseAndValidate(raw, simulationMessageSchema);
        if (sim) {
          updateNetworkFromSimulation((sim as { simulation_data: Record<string, unknown> }).simulation_data ?? {});
          return;
        }
        const cmd = parseAndValidate(raw, commandUpdateMessageSchema);
        if (cmd) {
          updateNetworkFromCommands((cmd as { command_data: Record<string, unknown> }).command_data ?? {});
        }
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    };

    ws.onclose = () => {
      setConnections((prev) => ({ ...prev, websocket: 'disconnected' }));
    };

    return () => ws.close();
  }, [updateNetworkFromSimulation, updateNetworkFromCommands]);

  useEffect(() => {
    const processDataFlow = () => {
      setNetworkNodes((prev) => {
        const updated = { ...prev } as NetworkNodesMap;

        if (updated.imu_sensor.data && !updated.imu_processor.data) {
          updated.imu_processor = {
            ...updated.imu_processor,
            data: {
              processed_imu: updated.imu_sensor.data,
              timestamp: Date.now()
            },
            lastUpdate: Date.now()
          };
        }

        if (
          (updated.imu_sensor.data || updated.gps_sensor.data) &&
          !updated.state_estimator.data
        ) {
          updated.state_estimator = {
            ...updated.state_estimator,
            data: {
              imu: updated.imu_sensor.data,
              gps: updated.gps_sensor.data,
              fused_state: {
                position: updated.gps_sensor.data,
                orientation: (updated.imu_sensor.data as Record<string, unknown>)?.orientation
              },
              timestamp: Date.now()
            },
            lastUpdate: Date.now()
          };
        }

        if (updated.state_estimator.data && !updated.navigation_controller.data) {
          updated.navigation_controller = {
            ...updated.navigation_controller,
            data: {
              current_state: (updated.state_estimator.data as Record<string, unknown>).fused_state,
              target_waypoint: { x: 10, y: 5 },
              navigation_command: {
                heading: Math.atan2(5, 10),
                distance: Math.sqrt(10 * 10 + 5 * 5)
              },
              timestamp: Date.now()
            },
            lastUpdate: Date.now()
          };
        }

        return updated;
      });
    };

    const interval = setInterval(processDataFlow, 100);
    return () => clearInterval(interval);
  }, []);

  return {
    networkNodes,
    connections,
    selectedNode,
    setSelectedNode,
    canData
  };
}
