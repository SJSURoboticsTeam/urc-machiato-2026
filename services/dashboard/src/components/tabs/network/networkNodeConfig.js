/**
 * Default network topology node config. Isolated for testing and Phase 3 typing.
 * @type {Record<string, { id: string, type: string, label: string, position: { x: number, y: number }, data: unknown, connections: string[] }>}
 */
export const INITIAL_NETWORK_NODES = {
  imu_sensor: {
    id: 'imu_sensor',
    type: 'generator',
    label: 'IMU Sensor',
    position: { x: 120, y: 120 },
    data: null,
    connections: ['imu_processor', 'state_estimator']
  },
  gps_sensor: {
    id: 'gps_sensor',
    type: 'generator',
    label: 'GPS Sensor',
    position: { x: 120, y: 240 },
    data: null,
    connections: ['navigation_controller', 'state_estimator']
  },
  battery_sensor: {
    id: 'battery_sensor',
    type: 'generator',
    label: 'Battery Monitor',
    position: { x: 120, y: 360 },
    data: null,
    connections: ['power_controller']
  },
  can_imu: {
    id: 'can_imu',
    type: 'generator',
    label: 'CAN IMU',
    position: { x: 120, y: 480 },
    data: null,
    connections: ['imu_processor', 'can_bus']
  },
  can_gps: {
    id: 'can_gps',
    type: 'generator',
    label: 'CAN GPS',
    position: { x: 120, y: 600 },
    data: null,
    connections: ['navigation_controller', 'can_bus']
  },
  can_motor_left: {
    id: 'can_motor_left',
    type: 'generator',
    label: 'CAN Left Motor',
    position: { x: 120, y: 720 },
    data: null,
    connections: ['left_motor', 'can_bus']
  },
  can_motor_right: {
    id: 'can_motor_right',
    type: 'generator',
    label: 'CAN Right Motor',
    position: { x: 120, y: 840 },
    data: null,
    connections: ['right_motor', 'can_bus']
  },
  can_bus: {
    id: 'can_bus',
    type: 'bus',
    label: 'CAN Bus',
    position: { x: 320, y: 600 },
    data: { status: 'operational', messages_per_sec: 0 },
    connections: ['mission_control', 'state_machine']
  },
  imu_processor: {
    id: 'imu_processor',
    type: 'processor',
    label: 'IMU Processor',
    position: { x: 320, y: 120 },
    data: null,
    connections: ['state_estimator', 'motion_controller']
  },
  state_estimator: {
    id: 'state_estimator',
    type: 'processor',
    label: 'State Estimator',
    position: { x: 320, y: 240 },
    data: null,
    connections: ['navigation_controller', 'motion_controller']
  },
  navigation_controller: {
    id: 'navigation_controller',
    type: 'controller',
    label: 'Navigation',
    position: { x: 520, y: 120 },
    data: null,
    connections: ['motion_controller']
  },
  motion_controller: {
    id: 'motion_controller',
    type: 'controller',
    label: 'Motion Control',
    position: { x: 520, y: 240 },
    data: null,
    connections: ['left_motor', 'right_motor']
  },
  power_controller: {
    id: 'power_controller',
    type: 'controller',
    label: 'Power Control',
    position: { x: 520, y: 360 },
    data: null,
    connections: []
  },
  left_motor: {
    id: 'left_motor',
    type: 'actuator',
    label: 'Left Motor',
    position: { x: 720, y: 180 },
    data: null,
    connections: []
  },
  right_motor: {
    id: 'right_motor',
    type: 'actuator',
    label: 'Right Motor',
    position: { x: 720, y: 300 },
    data: null,
    connections: []
  }
};
