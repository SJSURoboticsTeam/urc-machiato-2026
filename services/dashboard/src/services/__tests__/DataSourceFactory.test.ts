import { describe, it, expect } from 'vitest';
import { DataSourceFactory } from '../DataSourceFactory';
import { MockDataSource } from '../MockDataSource';
import { ROSDataSource } from '../ROSDataSource';

describe('DataSourceFactory', () => {
  it('create with mode mock returns MockDataSource', () => {
    const source = DataSourceFactory.create({ mode: 'mock' });
    expect(source).toBeInstanceOf(MockDataSource);
    expect(source.getSourceType()).toBe('mock');
  });

  it('create with mode mock uses default scenario when no URL param', () => {
    const source = DataSourceFactory.create({ mode: 'mock' });
    expect(source).toBeInstanceOf(MockDataSource);
  });

  it('create with mode mock uses config mockScenario', () => {
    const source = DataSourceFactory.create({
      mode: 'mock',
      mockScenario: 'sensor_degraded'
    });
    expect(source).toBeInstanceOf(MockDataSource);
  });

  it('create with mode ros returns ROSDataSource', () => {
    const source = DataSourceFactory.create({ mode: 'ros' });
    expect(source).toBeInstanceOf(ROSDataSource);
    expect(source.getSourceType()).toBe('ros');
  });

  it('create with mode ros uses config rosUrl', () => {
    const source = DataSourceFactory.create({
      mode: 'ros',
      rosUrl: 'ws://custom:9090'
    });
    expect(source).toBeInstanceOf(ROSDataSource);
  });

  it('create with mode auto returns ros or mock depending on VITE_USE_MOCK', () => {
    const source = DataSourceFactory.create({ mode: 'auto' });
    expect(['ros', 'mock']).toContain(source.getSourceType());
  });

  it('create throws for unknown mode', () => {
    expect(() =>
      DataSourceFactory.create({ mode: 'invalid' as 'auto' })
    ).toThrow('Unknown data source mode');
  });
});
