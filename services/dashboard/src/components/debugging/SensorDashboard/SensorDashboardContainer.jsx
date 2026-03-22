import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from '../../../utils/rosbridge';
import { useROS } from '../../../hooks/useROS';
import { parseAndValidate, sensorAnalyticsSchema } from '../../../utils/validationSchemas';
import { DEBUGGING_TOPICS } from '../../../config/rosTopics';
import { SensorHealthGrid, SensorTimeSeries, SensorAnalytics, SensorAlerts } from './index';
import { SENSOR_IDS, TIME_WINDOWS } from '../constants';

/** Build mock sensor data for demo when ROS topic is unavailable. */
function useMockSensorData() {
  const [sensorData, setSensorData] = useState({});
  const [timeSeriesData, setTimeSeriesData] = useState([]);
  const [noiseMetrics, setNoiseMetrics] = useState({});
  const [fusionContribution, setFusionContribution] = useState({});
  const [correlationMatrix, setCorrelationMatrix] = useState({});
  const [performanceMetrics, setPerformanceMetrics] = useState({});
  const nowMs = Date.now();

  useEffect(() => {
    const t0 = nowMs - 60000;
    const interval = 2000;
    const update = () => {
      const t = Date.now();
      const base = 0.7 + Math.random() * 0.25;
      const next = {};
      SENSOR_IDS.forEach((id, i) => {
        const jitter = (Math.random() - 0.5) * 0.1;
        next[id] = {
          confidence: Math.max(0, Math.min(1, base + jitter)),
          healthScore: Math.max(0, Math.min(1, base + jitter + 0.05)),
          lastUpdateMs: t - Math.random() * 1500
        };
      });
      setSensorData(next);

      setTimeSeriesData((prev) => {
        const slice = SENSOR_IDS.reduce((acc, id) => {
          acc[id] = next[id].confidence;
          return acc;
        }, { time: new Date(t).toISOString().slice(11, 19) });
        return [...prev.slice(-120), slice].slice(-120);
      });

      const noise = {};
      const fusion = {};
      const perf = {};
      SENSOR_IDS.forEach((id) => {
        noise[id] = {
          stdDev: 0.02 + Math.random() * 0.05,
          outlierCount: Math.floor(Math.random() * 3),
          signalQuality: 0.85 + Math.random() * 0.15
        };
        fusion[id] = 0.05 + Math.random() * 0.15;
        perf[id] = {
          hz: 5 + Math.random() * 15,
          latencyMs: 5 + Math.random() * 20,
          successRate: 0.92 + Math.random() * 0.08
        };
      });
      let sumFusion = Object.values(fusion).reduce((a, b) => a + b, 0);
      if (sumFusion > 0) {
        SENSOR_IDS.forEach((id) => { fusion[id] = fusion[id] / sumFusion; });
      }
      setNoiseMetrics(noise);
      setFusionContribution(fusion);
      setPerformanceMetrics(perf);

      const corr = {};
      SENSOR_IDS.forEach((id1) => {
        corr[id1] = {};
        SENSOR_IDS.forEach((id2) => {
          corr[id1][id2] = id1 === id2 ? 1 : (Math.random() * 0.6 - 0.2);
        });
      });
      setCorrelationMatrix(corr);
    };

    update();
    const id = setInterval(update, interval);
    return () => clearInterval(id);
  }, []);

  return {
    sensorData,
    timeSeriesData,
    noiseMetrics,
    fusionContribution,
    correlationMatrix,
    performanceMetrics
  };
}

/**
 * Full sensor debugging panel: grid, time-series, analytics, alerts.
 * Uses ROS /debugging/sensor_analytics when connected; falls back to mock data.
 * onSensorDataChange: optional callback to push sensor data for glanceable strip.
 */
export function SensorDashboardContainer({ onSensorDataChange }) {
  const [timeWindowSec, setTimeWindowSec] = useState(60);
  const { isConnected, ros } = useROS();
  const rosDataRef = useRef({ sensorData: {}, timeSeriesData: [] });

  const mock = useMockSensorData();

  useEffect(() => {
    if (!isConnected || !ros) return;
    const topicName = DEBUGGING_TOPICS.SENSOR_ANALYTICS;
    const topic = new ROSLIB.Topic({
      ros,
      name: topicName,
      messageType: 'std_msgs/msg/String'
    });
    topic.subscribe((msg) => {
      const raw = typeof msg?.data === 'string' ? msg.data : (msg && typeof msg === 'object' ? JSON.stringify(msg) : null);
      const data = raw ? parseAndValidate(raw, sensorAnalyticsSchema) : null;
      if (data?.sensors) {
        const now = Date.now();
        const next = {};
        Object.entries(data.sensors).forEach(([id, s]) => {
          next[id] = {
            confidence: s.confidence ?? 0,
            healthScore: s.health_score ?? s.confidence ?? 0,
            lastUpdateMs: s.last_update_ns ? s.last_update_ns / 1e6 : now
          };
        });
        rosDataRef.current.sensorData = next;
        rosDataRef.current.timeSeriesData = [
          ...(rosDataRef.current.timeSeriesData || []).slice(-119),
          { time: new Date().toISOString().slice(11, 19), ...Object.fromEntries(Object.entries(next).map(([k, v]) => [k, v.confidence])) }
        ];
      }
    });
    return () => topic.unsubscribe();
  }, [isConnected, ros]);

  const useLive = isConnected && Object.keys(rosDataRef.current.sensorData).length > 0;
  const sensorData = useLive ? rosDataRef.current.sensorData : mock.sensorData;
  const timeSeriesData = useLive ? (rosDataRef.current.timeSeriesData || []) : mock.timeSeriesData;

  React.useEffect(() => {
    if (onSensorDataChange) onSensorDataChange(sensorData);
  }, [sensorData, onSensorDataChange]);

  return (
    <div className="space-y-6 p-4">
      <div className="flex items-center justify-between">
        <h2 className="text-lg font-semibold text-zinc-200">Sensor Health Dashboard</h2>
        <span className={`text-xs ${isConnected ? 'text-emerald-400' : 'text-zinc-500'}`}>
          {isConnected ? 'Live' : 'Mock data'}
        </span>
      </div>

      <SensorAlerts sensorData={sensorData} />

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
        <h3 className="mb-3 text-sm font-medium text-zinc-300">Status grid</h3>
        <SensorHealthGrid sensorData={sensorData} stalenessSec={2} />
      </div>

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
        <h3 className="mb-3 text-sm font-medium text-zinc-300">Time series</h3>
        <SensorTimeSeries
          timeSeriesData={timeSeriesData}
          timeWindowSec={timeWindowSec}
          onTimeWindowChange={setTimeWindowSec}
        />
      </div>

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
        <h3 className="mb-3 text-sm font-medium text-zinc-300">Analytics</h3>
        <SensorAnalytics
          noiseMetrics={mock.noiseMetrics}
          fusionContribution={mock.fusionContribution}
          correlationMatrix={mock.correlationMatrix}
          performanceMetrics={mock.performanceMetrics}
        />
      </div>
    </div>
  );
}
