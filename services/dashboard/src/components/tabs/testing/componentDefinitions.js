import { Shield, Navigation, Eye, Zap, Radio } from 'lucide-react';

/**
 * Default component definitions for LegacyTestingView.
 * Status is derived from systemStatus when the view renders.
 */
export function getComponentDefinitions(systemStatus) {
  return [
    {
      id: 'safety',
      name: 'Safety',
      icon: Shield,
      status: systemStatus?.safety === 'ready' ? 'operational' : systemStatus?.safety ?? 'ready',
      tests: [
        { id: 'estop', name: 'E-Stop', status: 'passed' },
        { id: 'recovery', name: 'Recovery', status: 'passed' },
        { id: 'watchdog', name: 'Watchdog', status: 'running' },
        { id: 'thresholds', name: 'Thresholds', status: 'pending' }
      ],
      total: 6,
      passed: 4,
      failed: 0,
      running: 2
    },
    {
      id: 'navigation',
      name: 'Navigation',
      icon: Navigation,
      status: 'operational',
      tests: [
        { id: 'waypoint', name: 'Waypoint', status: 'passed' },
        { id: 'gps', name: 'GPS', status: 'passed' },
        { id: 'obstacles', name: 'Obstacles', status: 'passed' },
        { id: 'path_planning', name: 'Path Planning', status: 'passed' }
      ],
      total: 6,
      passed: 6,
      failed: 0,
      running: 0
    },
    {
      id: 'vision',
      name: 'Vision',
      icon: Eye,
      status: systemStatus?.vision ?? 'ready',
      tests: [
        { id: 'aruco', name: 'ArUco Detection', status: 'passed' },
        { id: 'object', name: 'Object Recognition', status: 'pending' }
      ],
      total: 4,
      passed: 2,
      failed: 0,
      running: 0
    },
    {
      id: 'can',
      name: 'CAN Bus',
      icon: Zap,
      status: 'mock',
      isMock: true,
      tests: [],
      total: 0,
      passed: 0,
      failed: 0,
      running: 0
    },
    {
      id: 'websocket',
      name: 'WebSocket',
      icon: Radio,
      status: systemStatus?.websocket ?? 'disconnected',
      tests: [
        { id: 'connection', name: 'Connection', status: 'passed' },
        { id: 'latency', name: 'Latency', status: 'passed' }
      ],
      total: 2,
      passed: 2,
      failed: 0,
      running: 0
    }
  ];
}
