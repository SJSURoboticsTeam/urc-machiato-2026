import React, { useState, useEffect, useRef } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { Network, Wifi, Zap, Cpu, Settings, Cog } from 'lucide-react';
import ROSLIB from '../../utils/rosbridge';

/**
 * Network Tab Component
 *
 * Node-based network visualization showing real-time data flow
 * between robotic system components
 */
export const NetworkTab = () => {
  const { currentState, systemStatus } = useSystemContext();
  const canvasRef = useRef(null);

  // Network nodes with balanced positioning
  const [networkNodes, setNetworkNodes] = useState({
    // Sensor nodes (generators) - Left side
    imu_sensor: {
      id: 'imu_sensor',
      type: 'generator',
      label: 'IMU Sensor',
      position: { x: 120, y: 120 },
      data: null,
      connections: ['imu_processor', 'state_estimator']
    },
    gps_sensor: {
      id: 'gps_sensor',
      type: 'generator',
      label: 'GPS Sensor',
      position: { x: 120, y: 240 },
      data: null,
      connections: ['navigation_controller', 'state_estimator']
    },
    battery_sensor: {
      id: 'battery_sensor',
      type: 'generator',
      label: 'Battery Monitor',
      position: { x: 120, y: 360 },
      data: null,
      connections: ['power_controller']
    },

    // CAN Bus sensors - Right side
    can_imu: {
      id: 'can_imu',
      type: 'generator',
      label: 'CAN IMU',
      position: { x: 120, y: 480 },
      data: null,
      connections: ['imu_processor', 'can_bus']
    },
    can_gps: {
      id: 'can_gps',
      type: 'generator',
      label: 'CAN GPS',
      position: { x: 120, y: 600 },
      data: null,
      connections: ['navigation_controller', 'can_bus']
    },
    can_motor_left: {
      id: 'can_motor_left',
      type: 'generator',
      label: 'CAN Left Motor',
      position: { x: 120, y: 720 },
      data: null,
      connections: ['left_motor', 'can_bus']
    },
    can_motor_right: {
      id: 'can_motor_right',
      type: 'generator',
      label: 'CAN Right Motor',
      position: { x: 120, y: 840 },
      data: null,
      connections: ['right_motor', 'can_bus']
    },

    // CAN Bus node
    can_bus: {
      id: 'can_bus',
      type: 'bus',
      label: 'CAN Bus',
      position: { x: 320, y: 600 },
      data: { status: 'operational', messages_per_sec: 0 },
      connections: ['mission_control', 'state_machine']
    },

    // Processor nodes - Middle left
    imu_processor: {
      id: 'imu_processor',
      type: 'processor',
      label: 'IMU Processor',
      position: { x: 320, y: 120 },
      data: null,
      connections: ['state_estimator', 'motion_controller']
    },
    state_estimator: {
      id: 'state_estimator',
      type: 'processor',
      label: 'State Estimator',
      position: { x: 320, y: 240 },
      data: null,
      connections: ['navigation_controller', 'motion_controller']
    },

    // Controller nodes - Middle right
    navigation_controller: {
      id: 'navigation_controller',
      type: 'controller',
      label: 'Navigation',
      position: { x: 520, y: 120 },
      data: null,
      connections: ['motion_controller']
    },
    motion_controller: {
      id: 'motion_controller',
      type: 'controller',
      label: 'Motion Control',
      position: { x: 520, y: 240 },
      data: null,
      connections: ['left_motor', 'right_motor']
    },
    power_controller: {
      id: 'power_controller',
      type: 'controller',
      label: 'Power Control',
      position: { x: 520, y: 360 },
      data: null,
      connections: []
    },

    // Actuator nodes - Right side
    left_motor: {
      id: 'left_motor',
      type: 'actuator',
      label: 'Left Motor',
      position: { x: 720, y: 180 },
      data: null,
      connections: []
    },
    right_motor: {
      id: 'right_motor',
      type: 'actuator',
      label: 'Right Motor',
      position: { x: 720, y: 300 },
      data: null,
      connections: []
    }
  });

  // Connection status and data flow
  const [connections, setConnections] = useState({
    websocket: 'disconnected',
    ros2: 'disconnected',
    can: 'disconnected'
  });

  // Selected node for inspection
  const [selectedNode, setSelectedNode] = useState(null);

  // CAN data state
  const [canData, setCanData] = useState({
    imu: null,
    gps: null,
    motor_left: null,
    motor_right: null,
    bus_status: { messages_per_sec: 0, error_count: 0 }
  });

  // ROS2 connection for CAN data
  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    ros.on('connection', () => {
      setConnections(prev => ({ ...prev, ros2: 'operational', can: 'operational' }));
    });

    ros.on('error', () => {
      setConnections(prev => ({ ...prev, ros2: 'disconnected', can: 'disconnected' }));
    });

    ros.on('close', () => {
      setConnections(prev => ({ ...prev, ros2: 'disconnected', can: 'disconnected' }));
    });

    // Subscribe to CAN sensor data
    const canDataSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: '/can/sensor_data',
      messageType: 'std_msgs/String'
    });

    canDataSubscriber.subscribe((message) => {
      try {
        const data = JSON.parse(message.data);
        setCanData(prev => ({
          ...prev,
          [data.sensor_type]: data,
          bus_status: {
            ...prev.bus_status,
            messages_per_sec: prev.bus_status.messages_per_sec + 1
          }
        }));

        // Update network nodes with real CAN data
        setNetworkNodes(currentNodes => ({
          ...currentNodes,
          [`can_${data.sensor_type}`]: {
            ...currentNodes[`can_${data.sensor_type}`],
            data: data,
            lastUpdate: Date.now()
          },
          can_bus: {
            ...currentNodes.can_bus,
            data: {
              status: 'operational',
              messages_per_sec: currentNodes.can_bus.data.messages_per_sec + 1,
              lastUpdate: Date.now()
            },
            lastUpdate: Date.now()
          }
        }));
      } catch (e) {
        console.error('Error parsing CAN data:', e);
      }
    });

    return () => {
      canDataSubscriber.unsubscribe();
      ros.close();
    };
  }, []);

  // Real-time data updates for network nodes
  useEffect(() => {
    const ws = new WebSocket('ws://localhost:8766');

    ws.onopen = () => {
      setConnections(prev => ({ ...prev, websocket: 'operational' }));
    };

    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === 'simulation_update') {
          updateNetworkFromSimulation(message.simulation_data);
        } else if (message.type === 'command_update') {
          updateNetworkFromCommands(message.command_data);
        }
      } catch (error) {
        console.error('Failed to parse WebSocket message:', error);
      }
    };

    ws.onclose = () => {
      setConnections(prev => ({ ...prev, websocket: 'disconnected' }));
    };

    return () => ws.close();
  }, []);

  // Update network nodes based on simulation data (generators)
  const updateNetworkFromSimulation = (simulationData) => {
    setNetworkNodes(prev => ({
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

      // CAN data from simulation (placeholder - will be updated via ROS2)
      can_imu: {
        ...prev.can_imu,
        data: simulationData.can?.imu || { status: 'mock', value: Math.random() * 10 },
        lastUpdate: Date.now()
      },
      can_gps: {
        ...prev.can_gps,
        data: simulationData.can?.gps || { status: 'mock', lat: 38.4, lon: -110.8 },
        lastUpdate: Date.now()
      },
      can_motor_left: {
        ...prev.can_motor_left,
        data: simulationData.can?.motor_left || { status: 'mock', temp: 25 + Math.random() * 10 },
        lastUpdate: Date.now()
      },
      can_motor_right: {
        ...prev.can_motor_right,
        data: simulationData.can?.motor_right || { status: 'mock', temp: 25 + Math.random() * 10 },
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
  };

  // Update network nodes based on commands (controllers/actuators)
  const updateNetworkFromCommands = (commandData) => {
    setNetworkNodes(prev => ({
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
  };

  // Process data through the network (simulate data flow)
  useEffect(() => {
    const processDataFlow = () => {
      setNetworkNodes(prev => {
        const updated = { ...prev };

        // IMU processor receives from IMU sensor
        if (updated.imu_sensor.data && !updated.imu_processor.data) {
          updated.imu_processor.data = {
            processed_imu: updated.imu_sensor.data,
            timestamp: Date.now()
          };
          updated.imu_processor.lastUpdate = Date.now();
        }

        // State estimator receives from multiple sources
        if ((updated.imu_sensor.data || updated.gps_sensor.data) && !updated.state_estimator.data) {
          updated.state_estimator.data = {
            imu: updated.imu_sensor.data,
            gps: updated.gps_sensor.data,
            fused_state: {
              position: updated.gps_sensor.data,
              orientation: updated.imu_sensor.data?.orientation
            },
            timestamp: Date.now()
          };
          updated.state_estimator.lastUpdate = Date.now();
        }

        // Navigation controller receives from state estimator
        if (updated.state_estimator.data && !updated.navigation_controller.data) {
          updated.navigation_controller.data = {
            current_state: updated.state_estimator.data.fused_state,
            target_waypoint: { x: 10, y: 5 }, // Example target
            navigation_command: {
              heading: Math.atan2(5, 10), // Calculate heading to target
              distance: Math.sqrt(10*10 + 5*5)
            },
            timestamp: Date.now()
          };
          updated.navigation_controller.lastUpdate = Date.now();
        }

        return updated;
      });
    };

    const interval = setInterval(processDataFlow, 100); // Process data flow every 100ms
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="h-full flex flex-col bg-zinc-950">
      {/* Header */}
      <div className="bg-zinc-900 border-b border-zinc-800 p-4">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <Network className="w-5 h-5 text-cyan-400" />
            <div>
              <h1 className="text-lg font-semibold text-zinc-100">Network Topology</h1>
              <p className="text-sm text-zinc-400">Real-time component interactions and data flow</p>
            </div>
          </div>

          {/* System Status */}
          <div className="flex items-center gap-4 text-xs">
            <div className="flex items-center gap-2">
              <Wifi className={`w-3 h-3 ${connections.websocket === 'operational' ? 'text-green-400' : 'text-red-400'}`} />
              <span className="text-zinc-400">WebSocket</span>
            </div>
            <div className="flex items-center gap-2">
              <Zap className={`w-3 h-3 ${connections.can === 'operational' ? 'text-green-400' : 'text-yellow-400'}`} />
              <span className="text-zinc-400">CAN</span>
            </div>
            <div className="flex items-center gap-2">
              <Cpu className={`w-3 h-3 ${connections.ros2 === 'operational' ? 'text-green-400' : 'text-yellow-400'}`} />
              <span className="text-zinc-400">ROS2</span>
            </div>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex">
        {/* Network Visualization */}
        <div className="flex-1 relative">
          <div className="absolute inset-0 bg-gradient-to-br from-zinc-900 to-zinc-950">
            <svg
              ref={canvasRef}
              className="w-full h-full"
              style={{ minHeight: '500px' }}
            >
              {/* Render connections first (behind nodes) */}
              {Object.values(networkNodes).map(node =>
                node.connections.map(targetId => {
                  const targetNode = networkNodes[targetId];
                  if (!targetNode) return null;

                  return (
                    <NetworkConnection
                      key={`${node.id}-${targetId}`}
                      from={node.position}
                      to={targetNode.position}
                      active={node.data && targetNode.data}
                    />
                  );
                })
              )}

              {/* Render nodes */}
              {Object.values(networkNodes).map(node => (
                <NetworkNode
                  key={node.id}
                  node={node}
                  isSelected={selectedNode === node.id}
                  onClick={() => setSelectedNode(selectedNode === node.id ? null : node.id)}
                />
              ))}
            </svg>

            {/* Legend */}
            <div className="absolute top-4 left-4 bg-zinc-800/90 rounded-lg p-3 border border-zinc-600 backdrop-blur-sm">
              <h4 className="text-sm font-medium text-zinc-200 mb-2">Node Types</h4>
              <div className="space-y-1 text-xs">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-green-500 rounded-full"></div>
                  <span className="text-zinc-300">Generator (Sensors)</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
                  <span className="text-zinc-300">Processor</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-purple-500 rounded-full"></div>
                  <span className="text-zinc-300">Controller (Commands)</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-orange-500 rounded-full"></div>
                  <span className="text-zinc-300">Actuator</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 bg-cyan-500 rounded-full"></div>
                  <span className="text-zinc-300">CAN Bus</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Node Inspector Panel */}
        <div className="w-80 bg-zinc-900 border-l border-zinc-800 flex flex-col">
          <div className="p-4 border-b border-zinc-800">
            <h3 className="text-sm font-semibold text-zinc-200 flex items-center gap-2">
              <Settings className="w-4 h-4" />
              Node Inspector
            </h3>
            <p className="text-xs text-zinc-400 mt-1">
              {selectedNode ? 'Click nodes to inspect data' : 'Select a node to view details'}
            </p>
          </div>

          <div className="flex-1 overflow-y-auto p-4">
            {selectedNode ? (
              <NodeInspector node={networkNodes[selectedNode]} />
            ) : (
              <div className="text-center py-8 text-zinc-500">
                <Network className="w-8 h-8 mx-auto mb-3 opacity-50" />
                <p className="text-sm">Select a node to inspect</p>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

// Network Node Component
const NetworkNode = ({ node, isSelected, onClick }) => {
  const getNodeColor = (type) => {
    const colors = {
      generator: 'fill-green-500 stroke-green-400',
      processor: 'fill-blue-500 stroke-blue-400',
      controller: 'fill-purple-500 stroke-purple-400',
      actuator: 'fill-orange-500 stroke-orange-400',
      bus: 'fill-cyan-500 stroke-cyan-400'  // CAN bus specific color
    };
    return colors[type] || 'fill-gray-500 stroke-gray-400';
  };

  const isActive = node.data && node.lastUpdate && (Date.now() - node.lastUpdate) < 2000;

  return (
    <g
      className="cursor-pointer transition-transform hover:scale-110"
      onClick={onClick}
    >
      {/* Node circle */}
      <circle
        cx={node.position.x}
        cy={node.position.y}
        r="28"
        className={`${getNodeColor(node.type)} transition-all duration-300 ${
          isSelected ? 'stroke-2 stroke-cyan-400' : 'stroke-1'
        } ${isActive ? 'animate-pulse' : ''} filter drop-shadow-lg`}
      />

      {/* Selection ring */}
      {isSelected && (
        <circle
          cx={node.position.x}
          cy={node.position.y}
          r="35"
          className="fill-none stroke-cyan-400 stroke-2 animate-ping"
        />
      )}

      {/* Node label */}
      <text
        x={node.position.x}
        y={node.position.y + 42}
        textAnchor="middle"
        className="text-xs fill-zinc-300 font-medium pointer-events-none"
      >
        {node.label}
      </text>

      {/* Data indicator */}
      {node.data && (
        <circle
          cx={node.position.x + 24}
          cy={node.position.y - 24}
          r="5"
          className="fill-green-400 animate-ping opacity-75"
        />
      )}

      {/* Connection count indicator */}
      {node.connections.length > 0 && (
        <text
          x={node.position.x + 20}
          y={node.position.y - 15}
          textAnchor="middle"
          className="text-xs fill-zinc-400 font-bold"
        >
          {node.connections.length}
        </text>
      )}
    </g>
  );
};

// Network Connection Component
const NetworkConnection = ({ from, to, active }) => {
  const midX = (from.x + to.x) / 2;
  const midY = (from.y + to.y) / 2;

  return (
    <g>
      {/* Connection line */}
      <line
        x1={from.x}
        y1={from.y}
        x2={to.x}
        y2={to.y}
        className={`stroke-2 transition-colors duration-300 ${
          active ? 'stroke-green-400' : 'stroke-zinc-600'
        }`}
        strokeWidth="2"
        markerEnd="url(#arrowhead)"
      />

      {/* Data flow indicator */}
      {active && (
        <circle
          cx={midX}
          cy={midY}
          r="4"
          className="fill-blue-400 animate-bounce"
          style={{
            animationDuration: '1.5s',
            animationDirection: 'alternate'
          }}
        />
      )}

      {/* Arrow marker definition */}
      <defs>
        <marker
          id="arrowhead"
          markerWidth="12"
          markerHeight="8"
          refX="11"
          refY="4"
          orient="auto"
        >
          <polygon
            points="0 0, 12 4, 0 8"
            className="fill-zinc-400"
          />
        </marker>
      </defs>
    </g>
  );
};

// Node Inspector Component
const NodeInspector = ({ node }) => {
  const getTypeColor = (type) => {
    const colors = {
      generator: 'border-green-500/20 bg-green-500/5',
      processor: 'border-blue-500/20 bg-blue-500/5',
      controller: 'border-purple-500/20 bg-purple-500/5',
      actuator: 'border-orange-500/20 bg-orange-500/5'
    };
    return colors[type] || 'border-gray-500/20 bg-gray-500/5';
  };

  const getTypeIcon = (type) => {
    const icons = {
      generator: '📡',
      processor: '⚙️',
      controller: '🎛️',
      actuator: '🔧'
    };
    return icons[type] || '🔧';
  };

  return (
    <div className={`rounded-lg border p-4 ${getTypeColor(node.type)}`}>
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <span className="text-lg">{getTypeIcon(node.type)}</span>
          <h4 className="text-sm font-semibold text-zinc-200">{node.label}</h4>
        </div>
        <div className={`w-3 h-3 rounded-full ${node.data ? 'bg-green-400' : 'bg-zinc-500'}`}></div>
      </div>

      <div className="space-y-3">
        <div className="text-xs text-zinc-400">
          <div>Type: <span className="text-zinc-300 capitalize">{node.type}</span></div>
          <div>Connections: <span className="text-zinc-300">{node.connections.length}</span></div>
          {node.lastUpdate && (
            <div>Last Update: <span className="text-zinc-300">
              {new Date(node.lastUpdate).toLocaleTimeString()}
            </span></div>
          )}
        </div>

        {node.data ? (
          <div className="bg-zinc-800/50 rounded p-3">
            <h5 className="text-xs font-medium text-zinc-300 mb-2">Live Data</h5>
            <DataDisplay data={node.data} compact={true} />
          </div>
        ) : (
          <div className="bg-zinc-800/50 rounded p-3 text-center">
            <div className="text-zinc-500 text-sm">No data available</div>
          </div>
        )}

        <div className="bg-zinc-800/50 rounded p-3">
          <h5 className="text-xs font-medium text-zinc-300 mb-2">Connections</h5>
          <div className="space-y-1">
            {node.connections.map(connId => (
              <div key={connId} className="text-xs text-zinc-400 flex items-center gap-2">
                <div className="w-1.5 h-1.5 bg-zinc-500 rounded-full"></div>
                {connId.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
              </div>
            ))}
            {node.connections.length === 0 && (
              <div className="text-xs text-zinc-500 italic">No outgoing connections</div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

// Data Display Helper Component
const DataDisplay = ({ data, compact = false }) => {
  if (!data || typeof data !== 'object') {
    return <span className="text-green-400 font-mono text-sm">{String(data)}</span>;
  }

  const formatValue = (value) => {
    if (typeof value === 'number') {
      return compact ? value.toFixed(2) : value.toFixed(4);
    }
    return String(value);
  };

  const renderValue = (key, value, depth = 0) => {
    if (typeof value === 'object' && value !== null && depth < (compact ? 2 : 3)) {
      return (
        <div key={key} className={`${depth > 0 ? 'ml-3' : ''}`}>
          {key && <div className="text-xs text-zinc-400 font-medium mb-1">{key}:</div>}
          <div className="space-y-0.5">
            {Object.entries(value).slice(0, compact ? 4 : undefined).map(([k, v]) =>
              renderValue(k, v, depth + 1)
            )}
          </div>
        </div>
      );
    }

    return (
      <div key={key} className="flex justify-between text-xs py-0.5 px-2 bg-zinc-800/30 rounded">
        <span className="text-zinc-400">{key}:</span>
        <span className="text-green-400 font-mono">{formatValue(value)}</span>
      </div>
    );
  };

  return (
    <div className="space-y-1 max-h-40 overflow-y-auto">
      {Object.entries(data).slice(0, compact ? 6 : undefined).map(([key, value]) =>
        renderValue(key, value, 0)
      )}
    </div>
  );
};
