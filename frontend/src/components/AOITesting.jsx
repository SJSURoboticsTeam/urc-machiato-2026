import React, { useState, useEffect, useRef } from 'react';

/**
 * Minimal AOI Testing Interface
 *
 * Real-time AOI monitoring with simulation for testing network conditions.
 * Designed for serial RGBD camera and CAN bus IMU/motor sensors.
 */
export const AOITesting = ({ isConnected, ros }) => {
  const [aoiData, setAoiData] = useState({});
  const [simulating, setSimulating] = useState(false);
  const [simulationConfig, setSimulationConfig] = useState({
    networkStress: false,
    serialDelay: 30, // ms
    canDelay: 10     // ms
  });
  const simulationRef = useRef(null);

  // AOI subscription - use the same pattern as other hooks
  useEffect(() => {
    if (!isConnected || !ros) return;

    const subscriptions = [];

    try {
      // AOI Status subscription
      const aoiStatusSub = ros.subscribe(
        '/system/aoi_status',
        (msg) => {
          try {
            const data = JSON.parse(msg.data);
        setAoiData(prev => ({
          ...prev,
          sensors: {
            ...prev.sensors,
            [data.sensor_name]: {
              name: data.sensor_name,
              aoi: data.current_aoi,
              status: data.freshness_status,
              quality: data.quality_score,
              transport: data.transport_type || getTransportType(data.sensor_name),
              networkLatency: data.network_latency || 0,
              transportLatency: data.transport_latency || 0,
              congestionDetected: data.congestion_detected || false,
              congestionFactor: data.congestion_factor || 1.0,
              predictedAoi: data.predicted_aoi || data.current_aoi,
              aoiTrend: data.aoi_trend || 0
            }
          }
        }));
          } catch (e) {
            console.warn('Failed to parse AOI status:', e);
          }
        },
        10
      );
      subscriptions.push(aoiStatusSub);

      // AOI Metrics subscription
      const aoiMetricsSub = ros.subscribe(
        '/system/aoi_metrics',
        (msg) => {
          try {
        const data = JSON.parse(msg.data);
        setAoiData(prev => ({
          ...prev,
          metrics: {
            systemAoi: data.system_average_aoi,
            freshSensors: data.fresh_sensors,
            totalSensors: data.total_sensors,
            health: data.health_status,
            // Network health metrics
            serialSensors: data.serial_sensors || 0,
            canSensors: data.can_sensors || 0,
            ethernetSensors: data.ethernet_sensors || 0,
            localSensors: data.local_sensors || 0,
            avgNetworkLatency: data.avg_network_latency || 0,
            maxNetworkLatency: data.max_network_latency || 0,
            congestedLinks: data.congested_links || 0,
            networkHealthScore: data.network_health_score || 1.0,
            networkRecommendations: data.network_recommendations || []
          }
        }));
          } catch (e) {
            console.warn('Failed to parse AOI metrics:', e);
          }
        },
        10
      );
      subscriptions.push(aoiMetricsSub);

    } catch (error) {
      console.error('Error setting up AOI subscriptions:', error);
    }

    // Cleanup subscriptions
    return () => {
      subscriptions.forEach(sub => {
        try {
          sub.unsubscribe();
        } catch (error) {
          console.error('Error unsubscribing from AOI topic:', error);
        }
      });
    };
  }, [isConnected, ros]);

  // Simulation engine
  const startSimulation = () => {
    setSimulating(true);
    simulationRef.current = setInterval(() => {
      const simulatedSensors = generateSimulatedSensors(simulationConfig);
      setAoiData({
        sensors: simulatedSensors,
        metrics: calculateSimulatedMetrics(simulatedSensors),
        simulated: true
      });
    }, 100); // 10Hz updates
  };

  const stopSimulation = () => {
    setSimulating(false);
    if (simulationRef.current) {
      clearInterval(simulationRef.current);
      simulationRef.current = null;
    }
    setAoiData({});
  };

  const generateSimulatedSensors = (config) => {
    const sensors = {
      imu: { transport: 'CAN', baseDelay: config.canDelay },
      camera_rgb: { transport: 'SERIAL', baseDelay: config.serialDelay },
      camera_depth: { transport: 'SERIAL', baseDelay: config.serialDelay + 10 },
      gps: { transport: 'LOCAL', baseDelay: 2 },
      slam_pose: { transport: 'LOCAL', baseDelay: 5 }
    };

    return Object.entries(sensors).reduce((acc, [name, params]) => {
      const networkDelay = params.baseDelay + (config.networkStress ? Math.random() * 50 : 0);
      const jitter = Math.random() * 5;
      const totalAoi = networkDelay + jitter;

      acc[name] = {
        name,
        aoi: totalAoi / 1000, // Convert to seconds
        status: getAoiStatus(totalAoi / 1000),
        quality: Math.max(0, 1 - (totalAoi / 1000 / 0.5)), // 0-500ms quality scale
        transport: params.transport
      };
      return acc;
    }, {});
  };

  const calculateSimulatedMetrics = (sensors) => {
    const aois = Object.values(sensors).map(s => s.aoi);
    const freshCount = aois.filter(a => a < 1.0).length;

    return {
      systemAoi: aois.reduce((a, b) => a + b, 0) / aois.length,
      freshSensors: freshCount,
      totalSensors: aois.length,
      health: freshCount > aois.length * 0.7 ? 'HEALTHY' : 'WARNING'
    };
  };

  const getTransportType = (sensorName) => {
    if (sensorName.includes('camera')) return 'SERIAL';
    if (sensorName.includes('imu') || sensorName.includes('motor')) return 'CAN';
    return 'LOCAL';
  };

  const getAoiStatus = (aoi) => {
    if (aoi < 0.1) return 'FRESH';
    if (aoi < 1.0) return 'ACCEPTABLE';
    if (aoi < 5.0) return 'STALE';
    return 'CRITICAL';
  };

  const getAoiColor = (aoi) => {
    if (aoi < 0.1) return '#10b981'; // green
    if (aoi < 1.0) return '#f59e0b'; // amber
    if (aoi < 5.0) return '#f97316'; // orange
    return '#ef4444'; // red
  };

  const getTransportIcon = (transport) => {
    switch (transport) {
      case 'SERIAL': return 'Serial';
      case 'CAN': return 'CAN';
      default: return 'Local';
    }
  };

  return (
    <div className="aoi-testing">
      <div className="testing-header">
        <h2>AOI Testing</h2>
        <div className="testing-controls">
          {!simulating ? (
            <button
              onClick={startSimulation}
              className="btn btn-primary"
            >
              ▶️ Start Simulation
            </button>
          ) : (
            <button
              onClick={stopSimulation}
              className="btn btn-destructive"
            >
              ⏹️ Stop Simulation
            </button>
          )}

          {simulating && (
            <label className="stress-toggle">
              <input
                type="checkbox"
                checked={simulationConfig.networkStress}
                onChange={(e) => setSimulationConfig(prev => ({
                  ...prev,
                  networkStress: e.target.checked
                }))}
              />
              Network Stress
            </label>
          )}
        </div>
      </div>

      <div className="testing-status">
        <span className={simulating ? 'status-simulating' : 'status-live'}>
          {simulating ? 'SIMULATION' : isConnected ? 'LIVE' : 'DISCONNECTED'}
        </span>
        {simulating && simulationConfig.networkStress && (
          <span className="status-stress">STRESS TEST</span>
        )}
      </div>

      {/* System Metrics */}
      <div className="metrics-grid">
          <div className="metric-card">
            <div className="metric-value" style={{ color: getAoiColor(aoiData.metrics?.systemAoi) }}>
              {aoiData.metrics?.systemAoi?.toFixed(3) || '--'}s
            </div>
            <div className="metric-label">System AOI</div>
          </div>

          <div className="metric-card">
            <div className="metric-value">
              {aoiData.metrics?.freshSensors || 0}/{aoiData.metrics?.totalSensors || 0}
            </div>
            <div className="metric-label">Fresh Sensors</div>
          </div>

          <div className="metric-card">
            <div className="metric-value" style={{
              color: (aoiData.metrics?.networkHealthScore || 1.0) > 0.7 ? '#10b981' : '#f59e0b'
            }}>
              {((aoiData.metrics?.networkHealthScore || 1.0) * 100).toFixed(0)}%
            </div>
            <div className="metric-label">Network Health</div>
          </div>

          <div className="metric-card">
            <div className="metric-value" style={{
              color: (aoiData.metrics?.congestedLinks || 0) > 0 ? '#f97316' : '#10b981'
            }}>
              {aoiData.metrics?.congestedLinks || 0}
            </div>
            <div className="metric-label">Congested Links</div>
          </div>
      </div>

      {/* Sensor AOI List */}
      <div className="sensor-list">
        <h3>Sensor AOI</h3>
        <div className="sensor-grid">
          {Object.values(aoiData.sensors || {}).map(sensor => (
            <div key={sensor.name} className="sensor-item">
              <div className="sensor-info">
                <span className="sensor-name">
                  {getTransportIcon(sensor.transport)} {sensor.name}
                </span>
                <span className="sensor-transport">
                  {sensor.transport}
                  {sensor.congestionDetected && <span className="congestion-indicator">WARNING</span>}
                </span>
              </div>

              <div className="sensor-aoi">
                <div className="aoi-bar">
                  <div
                    className="aoi-fill"
                    style={{
                      width: `${Math.min(sensor.aoi * 200, 100)}%`, // Scale for visibility
                      backgroundColor: getAoiColor(sensor.aoi)
                    }}
                  />
                </div>
                <div className="aoi-value">
                  {sensor.aoi.toFixed(3)}s
                </div>
                <div className="aoi-details">
                  <small>Network: {(sensor.networkLatency * 1000).toFixed(1)}ms</small>
                  <small>Transport: {(sensor.transportLatency * 1000).toFixed(1)}ms</small>
                </div>
              </div>

              <div className="sensor-status">
                <span className="status-text">{sensor.status}</span>
                <span className="quality-text">
                  {(sensor.quality * 100).toFixed(0)}%
                </span>
                <div className="sensor-predictive">
                  <small className={`trend-${sensor.aoiTrend === 1 ? 'up' : sensor.aoiTrend === -1 ? 'down' : 'stable'}`}>
                    {sensor.aoiTrend === 1 ? 'UP' : sensor.aoiTrend === -1 ? 'DOWN' : 'STABLE'} {(sensor.predictedAoi * 1000).toFixed(0)}ms
                  </small>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>

      {/* Network Topology */}
      <div className="network-topology">
        <h3>Network Topology</h3>
        <div className="topology-grid">
          <div className="transport-card">
            <div className="transport-header">
              <span className="transport-icon">Serial</span>
              <span className="transport-name">Serial</span>
            </div>
            <div className="transport-count">
              {aoiData.metrics?.serialSensors || 0} sensors
            </div>
            <div className="transport-bandwidth">1 Mbps</div>
          </div>

          <div className="transport-card">
            <div className="transport-header">
              <span className="transport-icon">CAN</span>
              <span className="transport-name">CAN</span>
            </div>
            <div className="transport-count">
              {aoiData.metrics?.canSensors || 0} sensors
            </div>
            <div className="transport-bandwidth">1 Mbps</div>
          </div>

          <div className="transport-card">
            <div className="transport-header">
              <span className="transport-icon">Ethernet</span>
              <span className="transport-name">Ethernet</span>
            </div>
            <div className="transport-count">
              {aoiData.metrics?.ethernetSensors || 0} sensors
            </div>
            <div className="transport-bandwidth">100 Mbps</div>
          </div>

          <div className="transport-card">
            <div className="transport-header">
              <span className="transport-icon">Local</span>
              <span className="transport-name">Local</span>
            </div>
            <div className="transport-count">
              {aoiData.metrics?.localSensors || 0} sensors
            </div>
            <div className="transport-bandwidth">1 Gbps+</div>
          </div>
        </div>

        {/* Network Recommendations */}
        {aoiData.metrics?.networkRecommendations?.length > 0 && (
          <div className="network-recommendations">
            <h4>💡 Network Optimization Recommendations</h4>
            <ul>
              {aoiData.metrics.networkRecommendations.map((rec, idx) => (
                <li key={idx}>{rec}</li>
              ))}
            </ul>
          </div>
        )}
      </div>

      {!isConnected && !simulating && (
        <div className="no-data-message">
          Connect to ROS or start simulation to view AOI data
        </div>
      )}
    </div>
  );
};
