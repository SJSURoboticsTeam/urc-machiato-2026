import React from 'react';
import { ROSContextProvider } from './ROSContext';
import { StateMachineContextProvider } from './StateMachineContext';
import { TelemetryContextProvider } from './TelemetryContext';
import { UIContextProvider } from './UIContext';

interface AppProvidersProps {
  children: React.ReactNode;
}

/**
 * Composes the four Phase 1 context providers in dependency order:
 * ROS -> StateMachine -> Telemetry -> UI.
 */
export function AppProviders({ children }: AppProvidersProps) {
  return (
    <ROSContextProvider>
      <StateMachineContextProvider>
        <TelemetryContextProvider>
          <UIContextProvider>
            {children}
          </UIContextProvider>
        </TelemetryContextProvider>
      </StateMachineContextProvider>
    </ROSContextProvider>
  );
}
