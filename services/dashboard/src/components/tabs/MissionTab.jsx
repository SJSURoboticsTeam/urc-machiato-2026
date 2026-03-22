import React, { useState } from 'react';
import { useROSContext } from '../../context/ROSContext';
import { useStateMachineContext } from '../../context/StateMachineContext';
import { useTelemetryContext } from '../../context/TelemetryContext';
import { useUIContext } from '../../context/UIContext';
import { SystemState } from '../../config/stateDefinitions';
import {
  MISSION_TEMPLATES,
  useMissionRos,
  MissionExecutionView,
  MissionPlannerView
} from './mission';

/**
 * Mission tab: planner when IDLE, execution view when AUTONOMOUS with active mission.
 * ROS logic in useMissionRos; views in mission/ subcomponents.
 */
export const MissionTab = () => {
  const { ros } = useROSContext();
  const { currentState, requestStateTransition } = useStateMachineContext();
  const { telemetry, dataSource } = useTelemetryContext();
  const { activeMission, setActiveMission } = useUIContext();

  const [missionType, setMissionType] = useState('science');
  const [priority, setPriority] = useState('normal');

  const { handleStartMission } = useMissionRos(
    ros,
    setActiveMission,
    requestStateTransition,
    dataSource
  );

  if (activeMission && currentState === SystemState.AUTONOMOUS) {
    return (
      <MissionExecutionView activeMission={activeMission} telemetry={telemetry} />
    );
  }

  return (
    <MissionPlannerView
      missionType={missionType}
      setMissionType={setMissionType}
      priority={priority}
      setPriority={setPriority}
      missionTemplates={MISSION_TEMPLATES}
      onStartMission={handleStartMission}
    />
  );
};
