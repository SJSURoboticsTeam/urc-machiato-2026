import React, { createContext, useContext, useCallback } from 'react';
import { useROSContext } from './ROSContext';
import { useStateMachine } from '../hooks/useStateMachine';
import type { StateMachineContextValue, StateBadgeConfig } from '../types/context';

const StateMachineContext = createContext<StateMachineContextValue | null>(null);

const STATE_BADGE_CONFIG: Record<string, StateBadgeConfig> = {
  BOOT: { label: 'BOOT', color: 'blue', emoji: '🔵' },
  IDLE: { label: 'IDLE', color: 'green', emoji: '🟢' },
  AUTONOMOUS: { label: 'AUTONOMOUS', color: 'cyan', emoji: '🔵' },
  TELEOPERATION: { label: 'TELEOP', color: 'yellow', emoji: '🟡' },
  SAFESTOP: { label: 'SAFE STOP', color: 'orange', emoji: '🟠' },
  SAFETY: { label: 'SAFETY', color: 'red', emoji: '🔴' },
  SHUTDOWN: { label: 'SHUTDOWN', color: 'gray', emoji: '⚫' }
};

export function useStateMachineContext(): StateMachineContextValue {
  const context = useContext(StateMachineContext);
  if (!context) {
    throw new Error('useStateMachineContext must be used within StateMachineContextProvider');
  }
  return context;
}

interface StateMachineContextProviderProps {
  children: React.ReactNode;
}

export function StateMachineContextProvider({ children }: StateMachineContextProviderProps) {
  const { ros } = useROSContext();
  const {
    currentState,
    currentSubstate,
    requestStateTransition,
    isTransitioning
  } = useStateMachine(ros);

  const getStateBadge = useCallback((): StateBadgeConfig => {
    return STATE_BADGE_CONFIG[currentState] ?? { label: currentState, color: 'gray', emoji: '⚪' };
  }, [currentState]);

  const handleEmergencyStop = useCallback(async () => {
    try {
      await requestStateTransition('SAFETY', 'Emergency stop activated from UI');
    } catch (error) {
      console.error('Emergency stop failed:', error);
    }
  }, [requestStateTransition]);

  const value: StateMachineContextValue = {
    currentState,
    currentSubstate,
    requestStateTransition,
    isTransitioning,
    getStateBadge,
    handleEmergencyStop
  };

  return (
    <StateMachineContext.Provider value={value}>
      {children}
    </StateMachineContext.Provider>
  );
}
