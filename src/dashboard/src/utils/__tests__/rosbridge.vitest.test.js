/**
 * rosbridge utilities: createMessage, connectionStatus, getStatusText.
 * Pure helpers tested without a real ROS connection.
 */
import { describe, it, expect } from 'vitest';
import { createMessage, connectionStatus, getStatusText } from '../rosbridge';

describe('rosbridge createMessage', () => {
  it('string returns { data }', () => {
    expect(createMessage.string('hello')).toEqual({ data: 'hello' });
  });

  it('point returns x,y,z', () => {
    expect(createMessage.point(1, 2, 3)).toEqual({ x: 1, y: 2, z: 3 });
  });

  it('quaternion defaults w=1', () => {
    expect(createMessage.quaternion()).toEqual({ x: 0, y: 0, z: 0, w: 1 });
    expect(createMessage.quaternion(0.1, 0.2, 0.3, 0.9)).toEqual({ x: 0.1, y: 0.2, z: 0.3, w: 0.9 });
  });

  it('twist returns linear and angular', () => {
    expect(createMessage.twist(1.0, 0.5)).toEqual({
      linear: { x: 1.0, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0.5 }
    });
  });

  it('poseStamped includes header and pose', () => {
    const pose = { position: { x: 1, y: 2, z: 3 } };
    const out = createMessage.poseStamped(pose, 'odom');
    expect(out.header.frame_id).toBe('odom');
    expect(out.pose).toEqual(pose);
    expect(out.header.stamp).toBeDefined();
  });
});

describe('rosbridge connectionStatus and getStatusText', () => {
  it('connectionStatus has expected keys', () => {
    expect(connectionStatus.CONNECTED).toBe('connected');
    expect(connectionStatus.CONNECTING).toBe('connecting');
    expect(connectionStatus.DISCONNECTED).toBe('disconnected');
    expect(connectionStatus.ERROR).toBe('error');
    expect(connectionStatus.FAILED).toBe('failed');
  });

  it('getStatusText returns correct labels', () => {
    expect(getStatusText(connectionStatus.CONNECTED)).toBe('Connected to ROS');
    expect(getStatusText(connectionStatus.CONNECTING)).toBe('Connecting to ROS...');
    expect(getStatusText(connectionStatus.DISCONNECTED)).toBe('Disconnected from ROS');
    expect(getStatusText(connectionStatus.ERROR)).toBe('Connection error');
    expect(getStatusText(connectionStatus.FAILED)).toBe('Failed to connect');
    expect(getStatusText('other')).toBe('Unknown status');
  });
});

