import React, { useState, useEffect } from 'react';
import { Database, Eye, EyeOff, RefreshCw } from 'lucide-react';
import { UI_CONSTANTS } from '../../constants/uiConstants';
import { generateMockGPS, generateMockIMU, generateMockCmdVel, generateMockBattery } from '../../utils/mockDataUtils';

/**
 * Topic Data Viewer Component
 * Displays real-time ROS2 topic data for monitoring
 */
export const TopicDataViewer = () => {
  const [selectedTopic, setSelectedTopic] = useState(null);
  const [topicData, setTopicData] = useState({});
  const [showRawData, setShowRawData] = useState(false);

  // Available topics (simulated - replace with real ROS2 topic list)
  const availableTopics = [
    { name: '/state_machine/current_state', type: 'String', rate: '1 Hz', source: 'ros2' },
    { name: '/gps/fix', type: 'NavSatFix', rate: '10 Hz', source: 'can' },
    { name: '/imu/data', type: 'Imu', rate: '100 Hz', source: 'can' },
    { name: '/cmd_vel', type: 'Twist', rate: '20 Hz', source: 'ros2' },
    { name: '/camera/image_raw', type: 'Image', rate: '30 Hz', source: 'ros2' },
    { name: '/battery/status', type: 'BatteryState', rate: '1 Hz', source: 'can' }
  ];

  // Simulate topic data updates
  useEffect(() => {
    if (!selectedTopic) return;

    const interval = setInterval(() => {
      const mockData = generateMockData(selectedTopic);
      setTopicData(prev => ({
        ...prev,
        [selectedTopic.name]: {
          data: mockData,
          timestamp: Date.now(),
          seq: (prev[selectedTopic.name]?.seq || 0) + 1
        }
      }));
    }, UI_CONSTANTS.TOPIC_UPDATE_INTERVAL);

    return () => clearInterval(interval);
  }, [selectedTopic]);

  const generateMockData = (topic) => {
    switch (topic.name) {
      case '/state_machine/current_state':
        return { data: 'READY' };
      case '/gps/fix':
        return generateMockGPS();
      case '/imu/data':
        return generateMockIMU();
      case '/cmd_vel':
        return generateMockCmdVel();
      case '/battery/status':
        return generateMockBattery();
      default:
        return { data: 'Mock data for ' + topic.name };
    }
  };

  const getSourceColor = (source) => {
    switch (source) {
      case 'ros2': return 'text-green-400';
      case 'can': return 'text-purple-400';
      case 'websocket': return 'text-blue-400';
      default: return 'text-zinc-400';
    }
  };

  const formatData = (data) => {
    if (showRawData) {
      return JSON.stringify(data, null, 2);
    }

    // Pretty format for common types
    if (typeof data === 'object' && data !== null) {
      return Object.entries(data).map(([key, value]) => (
        <div key={key} className="flex justify-between">
          <span className="text-zinc-400">{key}:</span>
          <span className="text-zinc-200 font-mono">
            {typeof value === 'number' ? value.toFixed(3) : String(value)}
          </span>
        </div>
      ));
    }

    return <span className="font-mono">{String(data)}</span>;
  };

  return (
    <div className="space-y-4">
      <h3 className="text-lg font-semibold text-zinc-200 flex items-center gap-2">
        <Database className="w-5 h-5" />
        Topic Data Viewer
      </h3>

      {/* Topic selector */}
      <div className="space-y-2">
        <div className="text-sm text-zinc-400">Available Topics</div>
        <div className="grid grid-cols-1 gap-2 max-h-48 overflow-y-auto">
          {availableTopics.map(topic => (
            <button
              key={topic.name}
              onClick={() => setSelectedTopic(topic)}
              className={`p-2 text-left rounded transition-colors ${
                selectedTopic?.name === topic.name
                  ? 'bg-blue-900 border border-blue-500 text-blue-200'
                  : 'bg-zinc-800 hover:bg-zinc-700 text-zinc-300'
              }`}
            >
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <span className={`text-xs px-1 rounded ${getSourceColor(topic.source)}`}>
                    {topic.source.toUpperCase()}
                  </span>
                  <span className="font-mono text-sm">{topic.name}</span>
                </div>
                <div className="text-xs text-zinc-500">
                  {topic.rate}
                </div>
              </div>
              <div className="text-xs text-zinc-500 mt-1">
                {topic.type}
              </div>
            </button>
          ))}
        </div>
      </div>

      {/* Selected topic data */}
      {selectedTopic && (
        <div className="space-y-2">
          <div className="flex items-center justify-between">
            <h4 className="text-sm font-medium text-zinc-300">
              {selectedTopic.name}
            </h4>
            <button
              onClick={() => setShowRawData(!showRawData)}
              className="text-xs text-zinc-400 hover:text-zinc-200 flex items-center gap-1"
            >
              {showRawData ? <EyeOff className="w-3 h-3" /> : <Eye className="w-3 h-3" />}
              {showRawData ? 'Pretty' : 'Raw'}
            </button>
          </div>

          <div className="bg-zinc-800 p-3 rounded">
            {topicData[selectedTopic.name] ? (
              <div className="space-y-2">
                <div className="flex justify-between text-xs text-zinc-500">
                  <span>Seq: {topicData[selectedTopic.name].seq}</span>
                  <span>{new Date(topicData[selectedTopic.name].timestamp).toLocaleTimeString()}</span>
                </div>
                <div className={`text-sm ${showRawData ? 'font-mono text-xs' : ''}`}>
                  {formatData(topicData[selectedTopic.name].data)}
                </div>
              </div>
            ) : (
              <div className="text-center text-zinc-500 py-4">
                <RefreshCw className="w-6 h-6 animate-spin mx-auto mb-2" />
                Waiting for data...
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};
