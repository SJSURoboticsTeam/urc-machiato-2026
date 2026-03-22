import React, { useState, useEffect } from 'react';
import { SafetyTestPanel } from '../SafetyTestPanel';
import { CameraPlaceholder } from '../CameraPlaceholder';
import { SensorDataSimulator } from '../sensor-simulator/SensorDataSimulator';
import { UI_CONSTANTS } from '../../constants/uiConstants';
import { generateTestExecutionTime, shouldTestSucceed } from '../../utils/mockDataUtils';
import { ComponentStatusOverview } from './ComponentStatusOverview';
import { ComponentTestRunner } from './ComponentTestRunner';

/**
 * Simplified Component Test Dashboard
 *
 * Broken into smaller, focused components for easier maintenance.
 * Provides centralized access to component testing and monitoring.
 */
export const ComponentTestDashboard = ({
  onRunTest,
  onStateTransition,
  isConnected,
  demoMode,
  currentState,
  rosInstance
}) => {
  const [activeTab, setActiveTab] = useState('overview');
  const [systemStatus, setSystemStatus] = useState({
    safety: 'unknown',
    navigation: 'unknown',
    vision: 'unknown',
    can: 'mock',
    websocket: 'unknown',
    frontend: 'connected'
  });

  const [testHistory, setTestHistory] = useState([]);
  const [isRunningTests, setIsRunningTests] = useState(false);

  // Component definitions with test capabilities
  const componentTests = {
    safety: {
      name: 'Safety System',
      description: 'Emergency stops, recovery procedures, and safety monitoring',
      status: systemStatus.safety,
      tests: ['software_estop', 'safety_recovery', 'safety_monitoring'],
      priority: 1
    },
    navigation: {
      name: 'Navigation',
      description: 'Waypoint navigation, path planning, and obstacle avoidance',
      status: systemStatus.navigation,
      tests: ['waypoint_navigation', 'path_planning', 'obstacle_avoidance'],
      priority: 2
    },
    vision: {
      name: 'Computer Vision',
      description: 'ArUco detection, object recognition, and visual processing',
      status: systemStatus.vision,
      tests: ['aruco_detection', 'object_recognition', 'visual_processing'],
      priority: 3
    },
    can: {
      name: 'CAN Bus',
      description: 'Sensor data communication and hardware interfacing',
      status: systemStatus.can,
      tests: ['can_communication', 'sensor_data', 'hardware_interface'],
      priority: 3
    }
  };

  // Status monitoring
  useEffect(() => {
    const updateStatus = () => {
      setSystemStatus(prev => ({
        ...prev,
        websocket: isConnected ? 'connected' : 'disconnected',
        safety: demoMode ? 'demo' : 'operational',
        navigation: demoMode ? 'demo' : 'operational',
        vision: demoMode ? 'demo' : 'operational'
      }));
    };

    updateStatus();
    const interval = setInterval(updateStatus, 5000);
    return () => clearInterval(interval);
  }, [isConnected, demoMode]);

  // Test execution
  const runComponentTest = async (componentId, testId) => {
    const test = {
      id: `${componentId}_${testId}_${Date.now()}`,
      component: componentId,
      test: testId,
      status: 'running',
      timestamp: Date.now(),
      priority: componentTests[componentId].priority
    };

    setTestHistory(prev => [...prev, test]);

    try {
      // Simulate test execution
      await new Promise(resolve => setTimeout(resolve, generateTestExecutionTime()));

      // Mock test results
      const success = shouldTestSucceed();
      setTestHistory(prev =>
        prev.map(t =>
          t.id === test.id
            ? { ...t, status: success ? 'passed' : 'failed' }
            : t
        )
      );

      if (onRunTest) {
        onRunTest(componentId, testId, success);
      }
    } catch (error) {
      setTestHistory(prev =>
        prev.map(t =>
          t.id === test.id
            ? { ...t, status: 'failed', error: error.message }
            : t
        )
      );
    }
  };

  const runFullSystemTest = async () => {
    setIsRunningTests(true);

    const components = Object.keys(componentTests);
    for (const componentId of components) {
      const component = componentTests[componentId];
      for (const testId of component.tests.slice(0, 2)) {
        await runComponentTest(componentId, testId);
        await new Promise(resolve => setTimeout(resolve, 500));
      }
    }

    setIsRunningTests(false);
  };


  // Tab content renderer
  const renderTabContent = () => {
    switch (activeTab) {
      case 'overview':
        return (
          <ComponentStatusOverview
            systemStatus={systemStatus}
            isConnected={isConnected}
          />
        );
      case 'safety':
        return <SafetyTestPanel />;
      case 'sensors':
        return <SensorDataSimulator isConnected={isConnected} />;
      case 'vision':
        return <CameraPlaceholder />;
      case 'navigation':
        return (
          <div className="text-center py-12">
            <div className="text-gray-400 mb-4">🧭</div>
            <h3 className="text-lg font-medium text-gray-900 mb-2">Navigation Testing</h3>
            <p className="text-gray-600">Navigation tests would be implemented here</p>
          </div>
        );
      case 'history':
        return (
          <ComponentTestRunner
            componentTests={componentTests}
            runComponentTest={runComponentTest}
            runFullSystemTest={runFullSystemTest}
            testHistory={testHistory}
            isRunningTests={isRunningTests}
            isConnected={isConnected}
          />
        );
      default:
        return null;
    }
  };

  const tabs = [
    { id: 'overview', name: 'Overview', icon: '📊' },
    { id: 'safety', name: 'Safety System', icon: '🛡️' },
    { id: 'sensors', name: 'Sensors & CAN', icon: '🔌' },
    { id: 'vision', name: 'Computer Vision', icon: '👁️' },
    { id: 'navigation', name: 'Navigation', icon: '🧭' },
    { id: 'history', name: 'Test History', icon: '📋' }
  ];

  return (
    <div className="min-h-screen bg-gray-50">
      {/* Header */}
      <div className="bg-white shadow-sm border-b">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <div>
              <h1 className="text-2xl font-bold text-gray-900">
                🧪 Component Test Dashboard
              </h1>
              <p className="text-sm text-gray-600 mt-1">
                Test and validate all URC 2026 system components
              </p>
            </div>
            <div className={`px-3 py-1 rounded-full text-sm font-medium ${isConnected ? 'text-green-600 bg-green-100' : 'text-red-600 bg-red-100'}`}>
              {isConnected ? '🟢 ROS2 Connected' : '🔴 ROS2 Disconnected'}
            </div>
          </div>
        </div>
      </div>

      {/* Navigation Tabs */}
      <div className="bg-white shadow-sm">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <nav className="flex space-x-8">
            {tabs.map(tab => (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id)}
                className={`py-4 px-1 border-b-2 font-medium text-sm ${
                  activeTab === tab.id
                    ? 'border-blue-500 text-blue-600'
                    : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
                }`}
              >
                {tab.icon} {tab.name}
              </button>
            ))}
          </nav>
        </div>
      </div>

      {/* Main Content */}
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        {renderTabContent()}
      </div>
    </div>
  );
};
