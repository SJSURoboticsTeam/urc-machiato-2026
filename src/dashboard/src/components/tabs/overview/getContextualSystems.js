import { SystemState } from '../../../config/stateDefinitions';

/**
 * Derives context-aware system status for Overview based on current state.
 * Used by OverviewIdleView, OverviewAutonomousView, OverviewSafetyView.
 */
export function getContextualSystems(currentState, systemStatus) {
  const baseSystems = {
    safety: systemStatus?.safety ?? 'ready',
    navigation: systemStatus?.navigation ?? 'ok',
    vision: systemStatus?.vision ?? 'ready'
  };

  if (currentState === SystemState.AUTONOMOUS) {
    return {
      ...baseSystems,
      mission: 'active',
      slam: 'active'
    };
  }

  if (currentState === SystemState.SAFETY) {
    return {
      ...baseSystems,
      safety: 'active'
    };
  }

  return baseSystems;
}
