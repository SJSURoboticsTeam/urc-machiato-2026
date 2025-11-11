import { useState, useEffect, useRef } from 'react';
import { createSubscriber, createServiceClient, callService } from '../utils/rosbridge';
import { STATE_TOPICS, MESSAGE_TYPES, SERVICE_TYPES } from '../config/rosTopics';
import {
  SystemState,
  AutonomousSubstate,
  EquipmentServicingSubstate,
  CalibrationSubstate
} from '../config/stateDefinitions';

/**
 * Custom hook for subscribing to state machine ROS topics
 *
 * Provides real-time state machine data from ROS backend
 *
 * @param {Object} ros - ROS connection instance from useROS hook
 * @returns {Object} Current state machine status
 */
export const useStateMachine = (ros) => {
  const [currentState, setCurrentState] = useState(SystemState.BOOT);
  const [currentSubstate, setCurrentSubstate] = useState(AutonomousSubstate.NONE);
  const [currentSubSubstate, setCurrentSubSubstate] = useState(EquipmentServicingSubstate.NONE);
  const [currentCalibrationSubstate, setCurrentCalibrationSubstate] = useState(CalibrationSubstate.NONE);
  const [stateMetadata, setStateMetadata] = useState({});
  const [transitionHistory, setTransitionHistory] = useState([]);
  const [isTransitioning, setIsTransitioning] = useState(false);
  const [lastTransition, setLastTransition] = useState(null);

  // ROS subscribers
  const subscribersRef = useRef({});

  // Subscribe to state topics when ROS is connected
  useEffect(() => {
    if (!ros) return;

    console.log('Setting up state machine topic subscriptions...');

    // Current state subscriber - handle JSON strings from standalone system
    subscribersRef.current.currentState = createSubscriber(
      ros,
      '/state_machine/current_state',
      'std_msgs/String',
      (message) => {
        try {
          const data = JSON.parse(message.data);
          console.log('Received current state:', data);
          setCurrentState(data.state || SystemState.BOOT);
          setCurrentSubstate(data.substate || 'initializing');
          setCurrentSubSubstate('none');
          setCurrentCalibrationSubstate('none');
          setStateMetadata(data);
          setIsTransitioning(false); // Standalone system doesn't track transitioning
        } catch (e) {
          console.warn('Failed to parse state message:', e);
        }
      }
    );

    // State transition subscriber
    subscribersRef.current.stateTransition = createSubscriber(
      ros,
      '/state_machine/state_transition',
      'std_msgs/String',
      (message) => {
        try {
          const data = JSON.parse(message.data);
          console.log('Received state transition:', data);
          setLastTransition({
            fromState: data.from_state,
            toState: data.to_state,
            reason: data.reason,
            initiatedBy: 'standalone_system',
            timestamp: data.timestamp,
            success: data.success
          });

        // Update transition history
        setTransitionHistory(prev => {
          const newHistory = [{
            fromState: data.from_state,
            toState: data.to_state,
            reason: data.reason,
            timestamp: new Date(data.timestamp * 1000).toISOString(),
            success: data.success
          }, ...prev].slice(0, 50); // Keep last 50 transitions
          return newHistory;
        });

        // Update state if transition was successful
        if (data.success) {
          setCurrentState(data.to_state);
          setIsTransitioning(false);
        }
        } catch (e) {
          console.warn('Failed to parse transition message:', e);
        }
      }
    );

    // Substate subscriber
    subscribersRef.current.substate = createSubscriber(
      ros,
      STATE_TOPICS.SUBSTATE,
      MESSAGE_TYPES.STRING,
      (message) => {
        console.log('Received substate update:', message);
        // Parse substate data (assuming format: "AUTONOMOUS:SCIENCE" or "CALIBRATION:SETUP")
        const parts = message.data.split(':');
        if (parts.length === 2) {
          const [type, substate] = parts;
          if (type === 'AUTONOMOUS') {
            setCurrentSubstate(substate);
          } else if (type === 'EQUIPMENT_SERVICING') {
            setCurrentSubSubstate(substate);
          } else if (type === 'CALIBRATION') {
            setCurrentCalibrationSubstate(substate);
          }
        }
      }
    );

    // Cleanup function
    return () => {
      console.log('Cleaning up state machine subscriptions...');
      Object.values(subscribersRef.current).forEach(subscriber => {
        if (subscriber && subscriber.unsubscribe) {
          subscriber.unsubscribe();
        }
      });
      subscribersRef.current = {};
    };
  }, [ros]);

  // Request current state (useful for initialization)
  const requestCurrentState = () => {
    if (!ros) return;

    // Call the get_current_state service
    ros.callService(
      STATE_TOPICS.GET_CURRENT_STATE_SERVICE,
      MESSAGE_TYPES.GET_SYSTEM_STATE,
      {},
      (result) => {
        console.log('Current state response:', result);
        if (result.current_state) {
          setCurrentState(result.current_state);
          setCurrentSubstate(result.autonomous_substate || AutonomousSubstate.NONE);
          setCurrentSubSubstate(result.equipment_servicing_substate || EquipmentServicingSubstate.NONE);
          setCurrentCalibrationSubstate(result.calibration_substate || CalibrationSubstate.NONE);
        }
      },
      (error) => {
        console.error('Failed to get current state:', error);
      }
    );
  };

  // Demo transition helper function
  const runDemoTransition = async (targetState, reason = 'frontend_request') => {
    console.log('Running demo transition to:', targetState);

    // Simulate network delay
    await new Promise(resolve => setTimeout(resolve, 500));

    // Store current state before transition
    const previousState = currentState;

    // Simulate successful transition for demo purposes
    const mockResult = {
      success: true,
      message: `Demo mode: ${previousState} → ${targetState} transition would succeed`,
      new_state: targetState,
      previous_state: previousState,
      timestamp: new Date().toISOString()
    };

    // Update local state to simulate the transition
    setCurrentState(targetState);
    setIsTransitioning(false); // Demo mode doesn't have real transitioning state

    // Simulate a state transition message
    const mockTransition = {
      from_state: previousState,
      to_state: targetState,
      reason: reason,
      initiated_by: 'frontend_demo',
      timestamp: new Date().toISOString(),
      success: true
    };

    // Update transition history
    setTransitionHistory(prev => {
      const newHistory = [{
        fromState: mockTransition.from_state,
        toState: mockTransition.to_state,
        reason: mockTransition.reason,
        timestamp: new Date().toISOString(),
        success: true
      }, ...prev].slice(0, 50); // Keep last 50 transitions
      return newHistory;
    });

    console.log('Demo transition result:', mockResult);
    return mockResult;
  };

  // Request state transition
  const requestStateTransition = async (targetState, reason = 'frontend_request', force = false) => {
    // Try ROS service first, fall back to demo mode
    if (!ros || !ros.isConnected) {
      console.log('ROS not connected - running in demo mode');
      return await runDemoTransition(targetState, reason);
    }

    try {
      // Call ROS service with comprehensive state change request
      const result = await callService(ros, '/state_machine/change_state', 'autonomy_interfaces/srv/ChangeState', {
        desired_state: targetState,
        desired_substate: '',
        desired_calibration_substate: '',
        reason: reason,
        operator_id: 'frontend',
        force: false,
        metadata: [`timestamp=${new Date().toISOString()}`, `source=frontend`]
      });
      console.log('ROS state change result:', result);

      if (result.success) {
        // ROS service succeeded - return the actual result
        return {
          success: true,
          message: result.message,
          newState: result.actual_state || targetState,
          timestamp: new Date().toISOString(),
          transitionTime: result.transition_time || 0,
          warnings: result.warnings || [],
          preconditionsMet: result.preconditions_met
        };
      } else {
        // ROS service failed with invalid transition - show the error
        const errorDetails = result.failed_preconditions && result.failed_preconditions.length > 0
          ? `Failed preconditions: ${result.failed_preconditions.join(', ')}`
          : result.message;
        return {
          success: false,
          message: errorDetails,
          newState: result.actual_state || currentState,
          timestamp: new Date().toISOString(),
          failedPreconditions: result.failed_preconditions || []
        };
      }
    } catch (error) {
      console.warn('ROS service error, falling back to demo mode:', error);
      return await runDemoTransition(targetState, reason);
    }
  };

  return {
    // Current state data
    currentState,
    currentSubstate,
    currentSubSubstate,
    currentCalibrationSubstate,
    stateMetadata,
    isTransitioning,
    lastTransition,

    // History
    transitionHistory,

    // Actions
    requestCurrentState,
    requestStateTransition,

    // Status
    isSubscribed: Object.keys(subscribersRef.current).length > 0
  };
};
