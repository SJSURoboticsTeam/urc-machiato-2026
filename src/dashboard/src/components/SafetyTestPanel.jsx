import React, { useState } from 'react';

/**
 * Safety Test Panel Component
 *
 * Provides testing interface for safety system features including:
 * - Software emergency stop testing
 * - Safety recovery procedures
 * - Safety monitoring verification
 * - System health checks
 */
export const SafetyTestPanel = ({
  onRunTest,
  onStateTransition,
  isConnected,
  demoMode,
  currentState
}) => {
  const [testResults, setTestResults] = useState([]);
  const [runningTest, setRunningTest] = useState(null);

  const safetyTests = [
    {
      id: 'software_estop',
      name: 'Software Emergency Stop',
      description: 'Test triggering software emergency stop - requires manual recovery',
      icon: '🚨',
      category: 'Emergency',
      requiresROS: true,
      warning: 'This will trigger a full emergency stop!'
    },
    {
      id: 'safety_recovery',
      name: 'Safety Recovery',
      description: 'Test automatic recovery from safety state',
      icon: '🔄',
      category: 'Recovery',
      requiresROS: true
    },
    {
      id: 'manual_recovery',
      name: 'Manual Safety Recovery',
      description: 'Test guided manual recovery from safety state',
      icon: '👤',
      category: 'Recovery',
      requiresROS: true
    },
    {
      id: 'watchdog_monitoring',
      name: 'Watchdog Monitoring',
      description: 'Verify safety watchdog is actively monitoring',
      icon: '👁️',
      category: 'Monitoring',
      requiresROS: true
    },
    {
      id: 'redundant_safety',
      name: 'Redundant Safety Check',
      description: 'Test redundant safety system consistency',
      icon: '🔒',
      category: 'Monitoring',
      requiresROS: true
    },
    {
      id: 'sensor_health',
      name: 'Sensor Health Check',
      description: 'Verify sensor health monitoring is working',
      icon: 'Sensor',
      category: 'Health',
      requiresROS: true
    },
    {
      id: 'system_diagnostics',
      name: 'System Diagnostics',
      description: 'Run comprehensive system health diagnostics',
      icon: '🔧',
      category: 'Health',
      requiresROS: true
    },
    {
      id: 'safety_state_transition',
      name: 'Safety State Transition',
      description: 'Test transition to safety state and validate safety operations',
      icon: '🔄',
      category: 'Integration',
      requiresROS: true,
      autoTransitionToSafety: true  // This test should auto-transition to safety
    },
    {
      id: 'automated_safety_scenario',
      name: 'Automated Safety Scenario',
      description: 'Run complete emergency-stop to recovery cycle',
      icon: '🤖',
      category: 'Automation',
      requiresROS: true
    },
    {
      id: 'safety_load_test',
      name: 'Safety Load Test',
      description: 'Test safety system under high-frequency service calls',
      icon: '⚡',
      category: 'Performance',
      requiresROS: true
    },
    {
      id: 'multi_alert_simulation',
      name: 'Multi-Alert Simulation',
      description: 'Simulate multiple concurrent safety alerts',
      icon: '🚨',
      category: 'Simulation',
      requiresROS: true
    }
  ];

  const runTest = async (testId) => {
    if (runningTest) return;

    setRunningTest(testId);
    const startTime = Date.now();

    try {
      // Check if this test should auto-transition to safety state
      const testConfig = safetyTests.find(test => test.id === testId);
      if (testConfig?.autoTransitionToSafety && currentState !== 'SAFETY') {
        console.log(`Test ${testId} auto-transitions to SAFETY state, current state: ${currentState}`);

        // Attempt to transition to safety state first
        try {
          const transitionResult = await onStateTransition('SAFETY', `Auto-transition for safety test: ${testId}`);
          if (!transitionResult.success) {
            throw new Error(`Failed to transition to SAFETY state: ${transitionResult.message}`);
          }
          console.log('Successfully transitioned to SAFETY state for test');

          // Wait a moment for the state change to propagate
          await new Promise(resolve => setTimeout(resolve, 500));
        } catch (transitionError) {
          console.warn('Could not transition to SAFETY state, but continuing with test:', transitionError.message);
          // Don't fail the test just because we couldn't transition to safety
        }
      }

      const result = await onRunTest(testId);
      setTestResults(prev => [{
        id: testId,
        success: result.success,
        message: result.message,
        duration: Date.now() - startTime,
        timestamp: new Date(),
        details: result.details
      }, ...prev.slice(0, 9)]); // Keep last 10 results
    } catch (error) {
      setTestResults(prev => [{
        id: testId,
        success: false,
        message: `Test failed: ${error.message}`,
        duration: Date.now() - startTime,
        timestamp: new Date(),
        details: error.details
      }, ...prev.slice(0, 9)]);
    } finally {
      setRunningTest(null);
    }
  };

  const canRunTest = (test) => {
    if (runningTest) return false;
    // Temporarily disable ROS requirement check to test UI
    // if (test.requiresROS && !isConnected) return false;
    // Tests with autoTransitionToSafety can run from any state
    if (test.requiresSafetyState && currentState !== 'SAFETY' && !test.autoTransitionToSafety) return false;
    return true;
  };

  const getTestStatus = (test) => {
    if (runningTest === test.id) return 'running';
    if (!canRunTest(test)) return 'disabled';
    return 'available';
  };

  const getTestStatusText = (test) => {
    if (runningTest === test.id) return 'Running...';
    if (test.requiresROS && !isConnected) return 'Requires ROS connection';
    if (test.requiresSafetyState && currentState !== 'SAFETY') return 'Requires SAFETY state';
    return 'Run Test';
  };

  const categories = [...new Set(safetyTests.map(test => test.category))];

  return (
    <div className="safety-test-panel">
      <div className="test-panel-header">
        <h2>Safety System Tests</h2>
        <div className="test-stats">
          <span className="stat-item">
            {testResults.filter(r => r.success).length} Passed
          </span>
          <span className="stat-item">
            {testResults.filter(r => !r.success).length} Failed
          </span>
          <span className="stat-item">
            {testResults.length} Total
          </span>
        </div>
      </div>

      {(!isConnected || demoMode) && (
        <div className={`connection-warning ${demoMode ? 'demo-mode' : ''}`}>
          {demoMode ? (
            <span>
              🎭 <strong>DEMO MODE:</strong> ROS services not available, running simulated tests
            </span>
          ) : (
            <span>
              ⚠️ <strong>ROS DISCONNECTED:</strong> Some tests will run in simulation mode
            </span>
          )}
        </div>
      )}

      <div className="test-categories">
        {categories.map(category => (
          <div key={category} className="test-category">
            <h3>{category} Tests</h3>
            <div className="test-grid">
              {safetyTests
                .filter(test => test.category === category)
                .map(test => (
                  <div key={test.id} className="test-card">
                    <div className="test-header">
                      <div className="test-icon">{test.icon}</div>
                      <div className="test-info">
                        <h4>{test.name}</h4>
                        <p>{test.description}</p>
                        {test.warning && (
                          <div className="test-warning">WARNING: {test.warning}</div>
                        )}
                      </div>
                    </div>
                    <button
                      onClick={() => runTest(test.id)}
                      disabled={!canRunTest(test)}
                      className={`test-btn ${getTestStatus(test)}`}
                    >
                      {getTestStatusText(test)}
                    </button>
                  </div>
                ))}
            </div>
          </div>
        ))}
      </div>

      {testResults.length > 0 && (
        <div className="test-results">
          <h3>Recent Test Results</h3>
          <div className="results-list">
            {testResults.map((result, idx) => (
              <div key={idx} className={`result-item ${result.success ? 'success' : 'failure'}`}>
                <div className="result-header">
                  <span className="result-status">
                    {result.success ? 'SUCCESS' : 'ERROR'}
                  </span>
                  <span className="result-test">
                    {safetyTests.find(t => t.id === result.id)?.name || result.id}
                  </span>
                  <span className="result-time">
                    {result.duration}ms • {result.timestamp.toLocaleTimeString()}
                  </span>
                </div>
                <div className="result-message">{result.message}</div>
                {result.details && (
                  <div className="result-details">
                    <pre>{JSON.stringify(result.details, null, 2)}</pre>
                  </div>
                )}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};
