import React, { createContext, useContext } from 'react';
import { useROS } from '../hooks/useROS';
import type { ROSContextValue } from '../types/context';

const ROSContext = createContext<ROSContextValue | null>(null);

export function useROSContext(): ROSContextValue {
  const context = useContext(ROSContext);
  if (!context) {
    throw new Error('useROSContext must be used within ROSContextProvider');
  }
  return context;
}

interface ROSContextProviderProps {
  children: React.ReactNode;
}

export function ROSContextProvider({ children }: ROSContextProviderProps) {
  const rosApi = useROS();
  const value: ROSContextValue = {
    ros: rosApi.ros,
    isConnected: rosApi.isConnected,
    connectionStatus: rosApi.connectionStatus,
    lastError: rosApi.lastError,
    reconnectAttempts: rosApi.reconnectAttempts,
    resetReconnection: rosApi.resetReconnection,
    connect: rosApi.connect
  };
  return (
    <ROSContext.Provider value={value}>
      {children}
    </ROSContext.Provider>
  );
}
