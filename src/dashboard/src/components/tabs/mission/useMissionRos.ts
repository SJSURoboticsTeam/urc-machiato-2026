import { useEffect, useRef, useCallback } from 'react';
import { SystemState } from '../../../config/stateDefinitions';
import { createSubscriber, createPublisher } from '../../../utils/rosbridge';
import {
  parseAndValidate,
  missionProgressSchema,
  missionStatusSchema
} from '../../../utils/validationSchemas';
import type { ActiveMission } from '../../../types/stateMachine';
import type { MissionTemplate } from '../../../types/stateMachine';

export interface UseMissionRosParams {
  ros: unknown;
  setActiveMission: (mission: ActiveMission | null) => void;
  requestStateTransition: (state: string, reason: string) => void;
}

export interface UseMissionRosReturn {
  handleStartMission: (
    missionType: string,
    priority: string,
    missionTemplates: MissionTemplate[]
  ) => void;
}

export function useMissionRos(
  ros: UseMissionRosParams['ros'],
  setActiveMission: UseMissionRosParams['setActiveMission'],
  requestStateTransition: UseMissionRosParams['requestStateTransition']
): UseMissionRosReturn {
  const subscribersRef = useRef<Record<string, { unsubscribe?: () => void }>>({});
  const commandPublisherRef = useRef<{ publish: (msg: { data: string }) => void } | null>(null);

  useEffect(() => {
    if (!ros) return;

    subscribersRef.current.status = createSubscriber(
      ros,
      '/mission/status',
      'std_msgs/String',
      (message: { data?: string }) => {
        parseAndValidate(message?.data, missionStatusSchema);
      }
    );

    subscribersRef.current.progress = createSubscriber(
      ros,
      '/mission/progress',
      'std_msgs/String',
      (message: { data?: string }) => {
        const data = parseAndValidate(message?.data, missionProgressSchema) as {
          mission_id?: string;
          name?: string;
          progress?: number;
          current_task?: string;
          next_task?: string;
          eta?: string;
          waypoints?: number | string;
          samples?: number | string;
          analysis?: string;
        } | null;
        if (data?.name != null && data?.progress !== undefined) {
          setActiveMission({
            id: data.mission_id,
            name: data.name,
            progress: data.progress,
            currentTask: data.current_task,
            nextTask: data.next_task,
            eta: data.eta,
            waypoints: data.waypoints,
            samples: data.samples,
            analysis: data.analysis
          });
        }
      }
    );

    subscribersRef.current.telemetry = createSubscriber(
      ros,
      '/mission/telemetry',
      'std_msgs/String',
      (message: { data?: string }) => {
        parseAndValidate(message?.data, missionStatusSchema);
      }
    );

    commandPublisherRef.current = createPublisher(ros, '/mission/commands', 'std_msgs/String');

    return () => {
      Object.values(subscribersRef.current).forEach((sub) => {
        if (sub && typeof sub.unsubscribe === 'function') {
          sub.unsubscribe();
        }
      });
    };
  }, [ros, setActiveMission]);

  const handleStartMission = useCallback(
    (
      missionType: string,
      priority: string,
      missionTemplates: MissionTemplate[]
    ) => {
      if (!commandPublisherRef.current) {
        console.warn('ROS2 command publisher not available');
        return;
      }

      const name = missionTemplates.find((t) => t.id === missionType)?.name ?? 'Mission';
      const missionConfig = {
        command: 'start',
        config: {
          name,
          type: missionType,
          priority,
          timestamp: Date.now()
        }
      };

      commandPublisherRef.current.publish({
        data: JSON.stringify(missionConfig)
      });

      requestStateTransition(SystemState.AUTONOMOUS, 'Mission started from UI');
    },
    [requestStateTransition]
  );

  return { handleStartMission };
}
