import '@testing-library/jest-dom'
import { vi } from 'vitest'

// Mock ROSLIB globally
global.ROSLIB = {
  Ros: vi.fn().mockImplementation(() => ({
    connect: vi.fn(),
    close: vi.fn(),
    on: vi.fn(),
    isConnected: true,
  })),
  Topic: vi.fn().mockImplementation(() => ({
    subscribe: vi.fn(),
    unsubscribe: vi.fn(),
    publish: vi.fn(),
    advertise: vi.fn(),
    unadvertise: vi.fn(),
  })),
  Service: vi.fn().mockImplementation(() => ({
    callService: vi.fn(),
  })),
  Message: vi.fn(),
};
