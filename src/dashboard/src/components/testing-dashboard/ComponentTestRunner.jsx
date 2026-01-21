import React, { useState } from 'react';
import { Play, CheckCircle2, XCircle, Clock, Zap } from 'lucide-react';
import { UI_CONSTANTS } from '../../constants/uiConstants';
import { getPriorityClasses } from '../../utils/uiUtils';

/**
 * Component Test Runner Component
 * Handles running tests for individual components or the full system
 */
export const ComponentTestRunner = ({
  componentTests,
  runComponentTest,
  runFullSystemTest,
  testHistory,
  isRunningTests,
  isConnected
}) => {
  const [selectedComponent, setSelectedComponent] = useState(null);

  const getStatusIcon = (status) => {
    switch (status) {
      case 'passed': return <CheckCircle2 className="w-4 h-4 text-green-600" />;
      case 'failed': return <XCircle className="w-4 h-4 text-red-600" />;
      case 'running': return <Clock className="w-4 h-4 text-blue-600 animate-spin" />;
      default: return <Clock className="w-4 h-4 text-gray-400" />;
    }
  };

  const getPriorityBadge = (priority) => getPriorityClasses(priority);

  const recentTests = testHistory.slice(-UI_CONSTANTS.RECENT_TESTS_LIMIT).reverse();

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-xl font-semibold text-gray-900">Test Runner</h2>
          <p className="text-sm text-gray-600 mt-1">
            Run automated tests for system components
          </p>
        </div>
        <button
          onClick={runFullSystemTest}
          disabled={isRunningTests || !isConnected}
          className={`flex items-center gap-2 px-4 py-2 rounded-md text-sm font-medium ${
            isRunningTests || !isConnected
              ? 'bg-gray-300 text-gray-500 cursor-not-allowed'
              : 'bg-blue-600 text-white hover:bg-blue-700'
          }`}
        >
          <Zap className="w-4 h-4" />
          {isRunningTests ? 'Running Tests...' : 'Run Full System Test'}
        </button>
      </div>

      {/* Component Test Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {Object.entries(componentTests).map(([componentId, component]) => (
          <div key={componentId} className="bg-white rounded-lg border border-gray-200 p-4">
            <div className="flex items-center justify-between mb-3">
              <div className="flex items-center gap-3">
                <div className={`w-3 h-3 rounded-full ${
                  component.status === 'operational' ? 'bg-green-500' :
                  component.status === 'mock' ? 'bg-orange-500' :
                  'bg-red-500'
                }`} />
                <div>
                  <h3 className="font-medium text-gray-900">{component.name}</h3>
                  <p className="text-sm text-gray-600">{component.description}</p>
                </div>
              </div>
              <span className={`px-2 py-1 rounded-full text-xs font-medium ${
                component.status === 'operational' ? 'bg-green-100 text-green-800' :
                component.status === 'mock' ? 'bg-orange-100 text-orange-800' :
                'bg-red-100 text-red-800'
              }`}>
                {component.status}
              </span>
            </div>

            {/* Test Actions */}
            <div className="space-y-2">
              {component.tests.slice(0, 3).map(testId => (
                <button
                  key={testId}
                  onClick={() => runComponentTest(componentId, testId)}
                  disabled={!isConnected}
                  className="w-full text-left px-3 py-2 rounded border border-gray-200 hover:border-blue-300 hover:bg-blue-50 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  <div className="flex items-center justify-between">
                    <span className="text-sm font-medium text-gray-700 capitalize">
                      {testId.replace(/_/g, ' ')}
                    </span>
                    <Play className="w-4 h-4 text-gray-400" />
                  </div>
                </button>
              ))}
            </div>
          </div>
        ))}
      </div>

      {/* Recent Test Results */}
      {recentTests.length > 0 && (
        <div className="bg-white rounded-lg border border-gray-200 p-4">
          <h3 className="text-lg font-semibold text-gray-900 mb-4">Recent Test Results</h3>
          <div className="space-y-2">
            {recentTests.map((test, index) => (
              <div key={index} className="flex items-center justify-between p-3 bg-gray-50 rounded">
                <div className="flex items-center gap-3">
                  {getStatusIcon(test.status)}
                  <div>
                    <div className="font-medium text-gray-900">
                      {test.component} - {test.test}
                    </div>
                    <div className="text-sm text-gray-600">
                      {new Date(test.timestamp).toLocaleTimeString()}
                    </div>
                  </div>
                </div>
                <div className="flex items-center gap-2">
                  {test.priority && (
                    <span className={`px-2 py-1 rounded-full text-xs font-medium ${getPriorityBadge(test.priority)}`}>
                      P{test.priority}
                    </span>
                  )}
                  <span className={`text-sm font-medium ${
                    test.status === 'passed' ? 'text-green-600' :
                    test.status === 'failed' ? 'text-red-600' :
                    'text-gray-600'
                  }`}>
                    {test.status}
                  </span>
                </div>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};
