import React, { useState, useEffect, useRef } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { UI_CONSTANTS } from '../../constants/uiConstants';
import ROSLIB from '../../utils/rosbridge';

/**
 * Three-Column Testing Dashboard
 *
 * Visualizes the complete data flow: CAN → WebSocket → ROS2
 * Shows real-time data from simulation system through all layers
 */
export const ThreeColumnTestingDashboard = () => {
  const { currentState, systemStatus } = useSystemContext();

  // Real-time data from different sources
  const [canData, setCanData] = useState(null);
  const [websocketData, setWebsocketData] = useState(null);
  const [rosData, setRosData] = useState({});

  // Connection status
  const [connections, setConnections] = useState({
    websocket: 'disconnected',
    ros2: 'disconnected'
  });

  // ROS connection refs
  const rosRef = useRef(null);
  const rosSubscribersRef = useRef({});

  // WebSocket connection for simulation data
  useEffect(() => {
    let ws = null;
    let reconnectTimeout = null;
    let reconnectAttempts = 0;
    const maxReconnectAttempts = 5;
    const reconnectDelay = 2000; // 2 seconds

    const connect = () => {
      if (ws && ws.readyState === WebSocket.OPEN) return;

      console.log(`Attempting WebSocket connection to localhost:8766 (attempt ${reconnectAttempts + 1})`);
      ws = new WebSocket('ws://localhost:8766');

      ws.onopen = () => {
        console.log('WebSocket connected to simulation backend');
        setConnections(prev => ({ ...prev, websocket: 'operational' }));
        reconnectAttempts = 0; // Reset on successful connection
      };

      ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === 'simulation_update') {
            setCanData(message.simulation_data);
            setWebsocketData(message.simulation_data);
          }
        } catch (error) {
          console.error('Failed to parse WebSocket message:', error);
        }
      };

      ws.onclose = () => {
        console.log('WebSocket disconnected from simulation backend');
        setConnections(prev => ({ ...prev, websocket: 'disconnected' }));

        // Attempt to reconnect if not at max attempts
        if (reconnectAttempts < maxReconnectAttempts) {
          reconnectAttempts++;
          reconnectTimeout = setTimeout(() => {
            connect();
          }, reconnectDelay);
        } else {
          console.warn('Max WebSocket reconnection attempts reached');
          setConnections(prev => ({ ...prev, websocket: 'error' }));
        }
      };

      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setConnections(prev => ({ ...prev, websocket: 'error' }));
      };
    };

    // Initial connection attempt with a small delay
    const initialTimeout = setTimeout(() => {
      connect();
    }, 500);

    return () => {
      clearTimeout(initialTimeout);
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      if (ws) ws.close();
    };
  }, []);

  // ROS2 connection for real-time ROS topic data
  useEffect(() => {
    const ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });
    rosRef.current = ros;

    ros.on('connection', () => {
      console.log('Connected to ROS Bridge.');
      setConnections(prev => ({ ...prev, ros2: 'operational' }));

      // Subscribe to ROS topics
      const imuTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/imu/data',
        messageType: 'sensor_msgs/Imu'
      });
      imuTopic.subscribe((message) => {
        console.log('Received IMU data from ROS:', message);
        setRosData(prev => ({ ...prev, imu: message }));
      });
      rosSubscribersRef.current.imu = imuTopic;

      const gpsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/gps/fix',
        messageType: 'sensor_msgs/NavSatFix'
      });
      gpsTopic.subscribe((message) => {
        console.log('Received GPS data from ROS:', message);
        setRosData(prev => ({ ...prev, gps: message }));
      });
      rosSubscribersRef.current.gps = gpsTopic;

      const batteryTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/battery/state',
        messageType: 'sensor_msgs/BatteryState'
      });
      batteryTopic.subscribe((message) => {
        console.log('Received Battery data from ROS:', message);
        setRosData(prev => ({ ...prev, battery: message }));
      });
      rosSubscribersRef.current.battery = batteryTopic;

      const odomTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
      });
      odomTopic.subscribe((message) => {
        console.log('Received Odometry data from ROS:', message);
        setRosData(prev => ({ ...prev, odom: message }));
      });
      rosSubscribersRef.current.odom = odomTopic;

    });

    ros.on('error', (error) => {
      console.error('Error connecting to ROS Bridge:', error);
      setConnections(prev => ({ ...prev, ros2: 'error' }));
    });

    ros.on('close', () => {
      console.log('Disconnected from ROS Bridge.');
      setConnections(prev => ({ ...prev, ros2: 'disconnected' }));
    });

    return () => {
      if (rosRef.current) {
        rosRef.current.close();
      }
      for (const topicName in rosSubscribersRef.current) {
        if (rosSubscribersRef.current[topicName]) {
          rosSubscribersRef.current[topicName].unsubscribe();
        }
      }
    };
  }, []);

  return (
    <div className="w-full bg-gray-950 text-white p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="mb-8">
          <h1 className="text-3xl font-bold text-white mb-2">
            Three-Column Testing Dashboard
          </h1>
          <p className="text-gray-400">
            Real-time visualization of data flow: CAN Simulation → WebSocket Bridge → ROS2 Topics
          </p>

          {/* System Status */}
          <div className="mt-4 flex items-center space-x-4">
            <StatusIndicator
              label="WebSocket"
              status={connections.websocket}
              color="blue"
            />
            <StatusIndicator
              label="ROS2 Bridge"
              status={connections.ros2}
              color="purple"
            />
            <StatusIndicator
              label="System State"
              status={currentState?.toLowerCase() || 'unknown'}
              color="green"
            />
          </div>
        </div>

        {/* Three Columns */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 min-h-0">

          {/* CAN Column */}
          <div className="bg-gray-900 rounded-xl border border-green-500/20 overflow-auto">
            <div className="bg-gradient-to-r from-green-500/10 to-green-600/10 p-4 border-b border-green-500/20">
              <div className="flex items-center justify-between">
                <h2 className="text-xl font-semibold text-green-400">CAN Bus Simulator</h2>
                <div className="flex items-center space-x-2">
                  <div className="w-2 h-2 bg-green-400 rounded-full animate-pulse"></div>
                  <span className="text-sm text-green-300">Generating</span>
                </div>
              </div>
              <p className="text-sm text-gray-400 mt-1">
                Raw simulation data from sensors and motors
              </p>
            </div>

            <div className="p-4 space-y-4">
              {canData ? (
                <>
                  <DataSection title="IMU Sensor" data={canData.imu} color="green" />
                  <DataSection title="GPS Position" data={canData.gps} color="green" />
                  <CANMessagesSection messages={canData.can_messages} />
                  <DataSection title="Rover State" data={canData.rover} color="green" />
                </>
              ) : (
                <div className="text-center py-8 text-gray-500">
                  <div className="animate-spin w-8 h-8 border-2 border-green-400 border-t-transparent rounded-full mx-auto mb-4"></div>
                  Waiting for simulation data...
                </div>
              )}
            </div>
          </div>

          {/* WebSocket Column */}
          <div className="bg-gray-900 rounded-xl border border-blue-500/20 overflow-auto">
            <div className="bg-gradient-to-r from-blue-500/10 to-blue-600/10 p-4 border-b border-blue-500/20">
              <div className="flex items-center justify-between">
                <h2 className="text-xl font-semibold text-blue-400">WebSocket Bridge</h2>
                <StatusBadge status={connections.websocket} />
              </div>
              <p className="text-sm text-gray-400 mt-1">
                Data received via WebSocket from CAN simulator
              </p>
            </div>

            <div className="p-4 space-y-4">
              {websocketData ? (
                <>
                  <DataSection title="IMU (WebSocket)" data={websocketData.imu} color="blue" />
                  <DataSection title="GPS (WebSocket)" data={websocketData.gps} color="blue" />
                  <div className="text-xs text-gray-500 text-center py-2">
                    CAN messages forwarded via WebSocket
                  </div>
                  <CANMessagesSection messages={websocketData.can_messages} />
                </>
              ) : (
                <div className="text-center py-8 text-gray-500">
                  <div className="w-8 h-8 border-2 border-blue-400 border-t-transparent rounded-full mx-auto mb-4 animate-spin"></div>
                  Waiting for WebSocket data...
                </div>
              )}
            </div>
          </div>

          {/* ROS Column */}
          <div className="bg-gray-900 rounded-xl border border-purple-500/20 overflow-auto">
            <div className="bg-gradient-to-r from-purple-500/10 to-purple-600/10 p-4 border-b border-purple-500/20">
              <div className="flex items-center justify-between">
                <h2 className="text-xl font-semibold text-purple-400">ROS2 Topics</h2>
                <StatusBadge status={connections.ros2} />
              </div>
              <p className="text-sm text-gray-400 mt-1">
                ROS2 topics receiving data from WebSocket bridge
              </p>
            </div>

            <div className="p-4 space-y-4">
              <DataSection title="IMU Data" data={rosData.imu} color="purple" />
              <DataSection title="GPS Data" data={rosData.gps} color="purple" />
              <DataSection title="Battery Data" data={rosData.battery} color="purple" />
              <DataSection title="Odometry Data" data={rosData.odom} color="purple" />
            </div>
          </div>

        </div>

        {/* Footer with data flow visualization */}
        <div className="mt-8 bg-gray-900 rounded-xl p-6 border border-gray-700">
          <h3 className="text-lg font-semibold mb-4">Data Flow Visualization</h3>
          <div className="flex items-center justify-center space-x-8">
            <div className="text-center">
              <div className="w-12 h-12 bg-green-500 rounded-full flex items-center justify-center mb-2">
                <span className="text-white font-bold">CAN</span>
              </div>
              <div className="text-sm text-gray-400">Simulation Data</div>
            </div>

            <div className="flex items-center">
              <div className="w-8 h-0.5 bg-blue-400"></div>
              <div className="w-4 h-4 bg-blue-400 rotate-45 transform translate-x-1"></div>
            </div>

            <div className="text-center">
              <div className="w-12 h-12 bg-blue-500 rounded-full flex items-center justify-center mb-2">
                <span className="text-white font-bold">WS</span>
              </div>
              <div className="text-sm text-gray-400">WebSocket Bridge</div>
            </div>

            <div className="flex items-center">
              <div className="w-8 h-0.5 bg-purple-400"></div>
              <div className="w-4 h-4 bg-purple-400 rotate-45 transform translate-x-1"></div>
            </div>

            <div className="text-center">
              <div className="w-12 h-12 bg-purple-500 rounded-full flex items-center justify-center mb-2">
                <span className="text-white font-bold">ROS</span>
              </div>
              <div className="text-sm text-gray-400">ROS2 Topics</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

// Helper Components
const StatusIndicator = ({ label, status, color }) => {
  const colorClasses = {
    green: 'bg-green-500',
    blue: 'bg-blue-500',
    purple: 'bg-purple-500',
    red: 'bg-red-500'
  };

  return (
    <div className="flex items-center space-x-2">
      <div className={`w-2 h-2 rounded-full ${colorClasses[color] || 'bg-gray-500'}`}></div>
      <span className="text-sm text-gray-300">{label}: {status}</span>
    </div>
  );
};

const StatusBadge = ({ status }) => {
  const statusConfig = {
    operational: { color: 'bg-green-500', text: 'Connected' },
    connected: { color: 'bg-green-500', text: 'Connected' },
    disconnected: { color: 'bg-red-500', text: 'Disconnected' },
    error: { color: 'bg-red-500', text: 'Error' },
    unknown: { color: 'bg-yellow-500', text: 'Unknown' }
  };

  const config = statusConfig[status] || statusConfig.unknown;

  return (
    <div className={`px-2 py-1 rounded-full text-xs font-medium ${config.color} text-white`}>
      {config.text}
    </div>
  );
};

const DataSection = ({ title, data, color = 'gray' }) => {
  if (!data) return null;

  const colorClasses = {
    green: 'border-green-500/20',
    blue: 'border-blue-500/20',
    purple: 'border-purple-500/20',
    gray: 'border-gray-500/20'
  };

  return (
    <div className={`bg-gray-800 rounded-lg p-3 border ${colorClasses[color]}`}>
      <h4 className="text-sm font-medium text-gray-300 mb-2">{title}</h4>
      <DataDisplay data={data} compact={true} />
    </div>
  );
};

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
          {key && <div className="text-xs text-gray-400 font-medium mb-1">{key}:</div>}
          <div className="space-y-0.5">
            {Object.entries(value).slice(0, compact ? 4 : undefined).map(([k, v]) =>
              renderValue(k, v, depth + 1)
            )}
          </div>
        </div>
      );
    }

    return (
      <div key={key} className="flex justify-between text-xs py-0.5 px-2">
        <span className="text-gray-400">{key}:</span>
        <span className="text-green-400 font-mono">{formatValue(value)}</span>
      </div>
    );
  };

  return (
    <div className="space-y-1">
      {Object.entries(data).slice(0, compact ? 6 : undefined).map(([key, value]) =>
        renderValue(key, value, 0)
      )}
    </div>
  );
};

const CANMessagesSection = ({ messages }) => {
  if (!messages || messages.length === 0) {
    return (
      <div className="bg-gray-800 rounded-lg p-3 border border-gray-500/20">
        <h4 className="text-sm font-medium text-gray-300 mb-2">CAN Messages</h4>
        <div className="text-xs text-gray-500">No CAN messages</div>
      </div>
    );
  }

  return (
    <div className="bg-gray-800 rounded-lg p-3 border border-green-500/20">
      <h4 className="text-sm font-medium text-green-300 mb-2">
        CAN Messages ({messages.length})
      </h4>
      <div className="space-y-2 max-h-32 overflow-y-auto">
        {messages.slice(0, 3).map((msg, index) => (
          <div key={index} className="bg-gray-700 rounded p-2">
            <div className="flex justify-between text-xs mb-1">
              <span className="text-green-400">ID: 0x{msg.id?.toString(16)}</span>
              <span className="text-gray-400">{msg.type}</span>
            </div>
            <DataDisplay data={msg.data} compact={true} />
          </div>
        ))}
        {messages.length > 3 && (
          <div className="text-xs text-gray-500 text-center py-1">
            +{messages.length - 3} more messages
          </div>
        )}
      </div>
    </div>
  );
};
