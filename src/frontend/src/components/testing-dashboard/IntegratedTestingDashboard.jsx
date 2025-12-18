import React, { useState } from 'react';
import { useSystemContext } from '../../context/SystemContext';
import { MessageTester } from './MessageTester';
import { StateTransitionTester } from './StateTransitionTester';
import { TopicDataViewer } from './TopicDataViewer';
import { CommunicationMetrics } from './CommunicationMetrics';
import { ThreeColumnTestingDashboard } from './ThreeColumnTestingDashboard';
import { DashboardErrorBoundary } from '../ui/ErrorBoundary';
import { TestTube2, MessageSquare, GitBranch, Database, Activity, Columns3 } from 'lucide-react';
import { UI_CONSTANTS } from '../../constants/uiConstants';

/**
 * Simplified Integrated Testing Dashboard
 *
 * Broken into smaller, focused components for easier maintenance.
 * Provides human testing interface for system integration verification.
 */
export const IntegratedTestingDashboard = () => {
  const {
    currentState,
    systemStatus,
    isConnected,
    telemetry,
    requestStateTransition
  } = useSystemContext();

  // Shared state for message testing
  const [messageHistory, setMessageHistory] = useState([]);
  const [flashingChannel, setFlashingChannel] = useState(null);
  const [lastMessageFlow, setLastMessageFlow] = useState(null);

  // State transition testing
  const [stateTransitionResult, setStateTransitionResult] = useState(null);

  // Communication metrics
  const [commMetrics, setCommMetrics] = useState({
    websocket: { sent: 0, received: 0, lastMsg: null, active: false },
    ros2: { sent: 0, received: 0, lastMsg: null, active: false },
    can: { sent: 0, received: 0, lastMsg: null, active: false, isMock: true }
  });

  const [activeTab, setActiveTab] = useState(UI_CONSTANTS.TESTING_TABS[0].id);

  const tabs = UI_CONSTANTS.TESTING_TABS;

  const renderTabContent = () => {
    switch (activeTab) {
      case 'three-column':
        return <ThreeColumnTestingDashboard />;
      case 'messages':
        return (
          <MessageTester
            messageHistory={messageHistory}
            setMessageHistory={setMessageHistory}
            setFlashingChannel={setFlashingChannel}
            setLastMessageFlow={setLastMessageFlow}
            commMetrics={commMetrics}
            setCommMetrics={setCommMetrics}
          />
        );
      case 'states':
        return (
          <StateTransitionTester
            currentState={currentState}
            requestStateTransition={requestStateTransition}
            stateTransitionResult={stateTransitionResult}
            setStateTransitionResult={setStateTransitionResult}
          />
        );
      case 'topics':
        return <TopicDataViewer />;
      case 'metrics':
        return <CommunicationMetrics commMetrics={commMetrics} />;
      default:
        return null;
    }
  };

  return (
    <DashboardErrorBoundary>
      <div className="min-h-screen bg-zinc-950 text-zinc-100">
        <div className="container mx-auto p-6 space-y-6">
        {/* Header */}
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <TestTube2 className="w-8 h-8 text-blue-400" />
            <div>
              <h1 className="text-2xl font-bold text-zinc-100">
                Integrated Testing Dashboard
              </h1>
              <p className="text-zinc-400">
                Human verification interface for system integration testing
              </p>
            </div>
          </div>

          {/* System status */}
          <div className="flex items-center gap-4">
            <div className={`px-3 py-1 rounded-full text-sm font-medium ${
              isConnected
                ? 'bg-green-900/50 text-green-300 border border-green-700'
                : 'bg-red-900/50 text-red-300 border border-red-700'
            }`}>
              {isConnected ? '🟢 Connected' : '🔴 Disconnected'}
            </div>
            <div className="px-3 py-1 bg-zinc-800 rounded text-sm">
              State: <span className="font-mono text-blue-400">{currentState}</span>
            </div>
          </div>
        </div>

        {/* Tab navigation */}
        <div className="flex gap-1 bg-zinc-900 p-1 rounded-lg">
          {tabs.map(tab => {
            const Icon = tab.icon;
            return (
              <button
                key={tab.id}
                onClick={() => setActiveTab(tab.id)}
                className={`flex items-center gap-2 px-4 py-2 rounded text-sm font-medium transition-colors ${
                  activeTab === tab.id
                    ? 'bg-blue-600 text-white'
                    : 'text-zinc-300 hover:text-zinc-100 hover:bg-zinc-800'
                }`}
              >
                <Icon className="w-4 h-4" />
                {tab.label}
              </button>
            );
          })}
        </div>

        {/* Tab content */}
        <div className="bg-zinc-900 rounded-lg p-6">
          {renderTabContent()}
        </div>

        {/* Footer info */}
        <div className="text-center text-xs text-zinc-500">
          Testing Dashboard - Verify system integration through manual testing
        </div>
      </div>
    </div>
    </DashboardErrorBoundary>
  );
};
