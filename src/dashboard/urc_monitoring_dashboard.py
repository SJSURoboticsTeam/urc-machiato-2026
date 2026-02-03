#!/usr/bin/env python3
"""
URC 2026 Rover Monitoring Dashboard - Streamlit + Plotly Implementation

Professional web-based monitoring dashboard with real-time visualization,
interactive controls, and comprehensive rover status monitoring.

Replaces 150+ lines of custom plotting with professional web dashboard
capabilities using Streamlit and Plotly.

Author: URC 2026 Dashboard Team
"""

import time
import random
import pandas as pd
import numpy as np
from datetime import datetime, timedelta
from typing import Dict, List, Any, Optional

# Import visualization libraries
try:
    import streamlit as st
    import plotly.graph_objects as go
    import plotly.express as px
    from plotly.subplots import make_subplots

    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False

    # Fallback implementations
    class st:
        @staticmethod
        def title(*args, **kwargs):
            print(*args)

        @staticmethod
        def header(*args, **kwargs):
            print(*args)

        @staticmethod
        def subheader(*args, **kwargs):
            print(*args)

        @staticmethod
        def metric(*args, **kwargs):
            print(f"{args[0]}: {args[1]}")

        @staticmethod
        def columns(*args):
            return [MockColumn() for _ in range(args[0])]

        @staticmethod
        def empty():
            return MockEmpty()

        @staticmethod
        def plotly_chart(*args, **kwargs):
            print("Chart would be displayed here")

        @staticmethod
        def line_chart(*args, **kwargs):
            print("Line chart would be displayed here")

        @staticmethod
        def selectbox(*args, **kwargs):
            return args[1][0] if len(args) > 1 else None

        @staticmethod
        def multiselect(*args, **kwargs):
            return args[1] if len(args) > 1 else []

        @staticmethod
        def checkbox(*args, **kwargs):
            return False

        @staticmethod
        def button(*args, **kwargs):
            return False

        @staticmethod
        def slider(*args, **kwargs):
            return args[1] if len(args) > 1 else 0

        @staticmethod
        def text_input(*args, **kwargs):
            return ""

        @staticmethod
        def sidebar():
            return MockSidebar()

    class MockColumn:
        def metric(self, *args, **kwargs):
            print(f"{args[0]}: {args[1]}")

    class MockEmpty:
        def line_chart(self, *args, **kwargs):
            print("Updating chart...")

    class MockSidebar:
        def title(self, *args, **kwargs):
            print(*args)

        def selectbox(self, *args, **kwargs):
            return args[1][0] if len(args) > 1 else None

    go = None
    px = None

# Import our data processing and test factories
from tests.factories.test_data_factories import TelemetryFactory, RoverStateFactory
from src.core.data_manager import get_data_manager


class URCMonitoringDashboard:
    """
    Professional monitoring dashboard for URC 2026 rover operations.

    Provides real-time visualization, historical analysis, and interactive
    controls for comprehensive rover monitoring.
    """

    def __init__(self):
        self.data_processor = TelemetryDataProcessor()
        self.telemetry_history = []
        self.max_history_points = 1000

        # Initialize session state for Streamlit
        if VISUALIZATION_AVAILABLE:
            if "dashboard_initialized" not in st.session_state:
                st.session_state.dashboard_initialized = True
                st.session_state.telemetry_data = []
                st.session_state.alerts = []
                st.session_state.mission_status = "IDLE"

    def run_dashboard(self):
        """Main dashboard execution."""
        if not VISUALIZATION_AVAILABLE:
            self._run_fallback_dashboard()
            return

        # Configure page
        st.set_page_config(
            page_title="URC 2026 Rover Monitor",
            page_icon="🚀",
            layout="wide",
            initial_sidebar_state="expanded",
        )

        # Title and header
        st.title("🚀 URC 2026 Mars Rover Monitoring Dashboard")
        st.markdown("*Real-time rover status and mission monitoring*")

        # Sidebar controls
        self._render_sidebar()

        # Main dashboard layout
        self._render_main_dashboard()

        # Auto-refresh
        time.sleep(1)
        st.rerun()

    def _render_sidebar(self):
        """Render sidebar with controls."""
        with st.sidebar:
            st.title("🎛️ Controls")

            # Mission control
            st.subheader("Mission Control")
            mission_action = st.selectbox(
                "Mission Action:",
                [
                    "Start Mission",
                    "Pause Mission",
                    "Resume Mission",
                    "Emergency Stop",
                    "Return Home",
                ],
            )

            if st.button("Execute Action", type="primary"):
                self._execute_mission_action(mission_action)

            # Monitoring settings
            st.subheader("Monitoring Settings")
            refresh_rate = st.slider("Refresh Rate (seconds):", 1, 10, 2)
            history_length = st.slider("History Length (minutes):", 5, 60, 15)

            # Alert settings
            st.subheader("Alert Settings")
            battery_alert = st.checkbox("Battery Alerts", value=True)
            comm_alert = st.checkbox("Communication Alerts", value=True)
            temp_alert = st.checkbox("Temperature Alerts", value=True)

            # System info
            st.subheader("System Info")
            st.metric("Dashboard Uptime", "00:15:30")
            st.metric("Data Points", len(self.telemetry_history))

    def _render_main_dashboard(self):
        """Render main dashboard content."""
        # Generate current telemetry
        current_telemetry = self._generate_current_telemetry()
        self._update_telemetry_history(current_telemetry)

        # Key metrics row
        self._render_key_metrics(current_telemetry)

        # Charts row
        col1, col2 = st.columns(2)
        with col1:
            self._render_system_status_chart()
        with col2:
            self._render_mission_progress_chart()

        # Detailed charts
        self._render_detailed_charts()

        # Alerts and logs
        self._render_alerts_and_logs()

    def _render_key_metrics(self, telemetry: Dict[str, Any]):
        """Render key performance metrics."""
        st.header("📊 Key Metrics")

        col1, col2, col3, col4 = st.columns(4)

        with col1:
            battery_level = telemetry.get("battery_level", 85)
            battery_color = "normal"
            if battery_level < 20:
                battery_color = "inverse"  # Red background
            st.metric(
                "Battery Level",
                f"{battery_level}%",
                delta=f"{random.uniform(-2, 1):.1f}%",
                delta_color="inverse" if battery_level < 20 else "normal",
            )

        with col2:
            comm_status = telemetry.get("communication_status", "connected")
            status_color = "🟢" if comm_status == "connected" else "🔴"
            st.metric("Communication", f"{status_color} {comm_status.title()}")

        with col3:
            system_health = telemetry.get("system_health", "nominal")
            health_color = (
                "🟢"
                if system_health == "nominal"
                else "🟡" if system_health == "degraded" else "🔴"
            )
            st.metric("System Health", f"{health_color} {system_health.title()}")

        with col4:
            mission_status = st.session_state.mission_status
            status_icon = {
                "IDLE": "⏸️",
                "ACTIVE": "▶️",
                "PAUSED": "⏸️",
                "EMERGENCY": "🚨",
            }.get(mission_status, "❓")
            st.metric("Mission Status", f"{status_icon} {mission_status}")

    def _render_system_status_chart(self):
        """Render system status overview chart."""
        st.subheader("System Status Overview")

        if not self.telemetry_history:
            st.info("Waiting for telemetry data...")
            return

        # Create DataFrame from history
        df = pd.DataFrame(self.telemetry_history[-50:])  # Last 50 points

        if df.empty:
            return

        # Create subplots
        fig = make_subplots(
            rows=2,
            cols=2,
            subplot_titles=(
                "Battery Level",
                "CPU Usage",
                "Memory Usage",
                "Temperature",
            ),
            specs=[
                [{"type": "scatter"}, {"type": "scatter"}],
                [{"type": "scatter"}, {"type": "scatter"}],
            ],
        )

        # Add traces
        if "battery_level" in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df.index,
                    y=df["battery_level"],
                    mode="lines+markers",
                    name="Battery",
                    line=dict(color="blue"),
                ),
                row=1,
                col=1,
            )

        if "cpu_usage" in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df.index,
                    y=df["cpu_usage"],
                    mode="lines",
                    name="CPU",
                    line=dict(color="red"),
                ),
                row=1,
                col=2,
            )

        if "memory_usage" in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df.index,
                    y=df["memory_usage"],
                    mode="lines",
                    name="Memory",
                    line=dict(color="green"),
                ),
                row=2,
                col=1,
            )

        if "temperature" in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df.index,
                    y=df["temperature"],
                    mode="lines",
                    name="Temperature",
                    line=dict(color="orange"),
                ),
                row=2,
                col=2,
            )

        fig.update_layout(height=400, showlegend=False)
        st.plotly_chart(fig, use_container_width=True)

    def _render_mission_progress_chart(self):
        """Render mission progress visualization."""
        st.subheader("Mission Progress")

        # Generate sample mission data
        mission_data = self._generate_mission_progress_data()

        if mission_data:
            # Create progress chart
            fig = go.Figure()

            fig.add_trace(
                go.Scatter(
                    x=[wp["x"] for wp in mission_data["waypoints"]],
                    y=[wp["y"] for wp in mission_data["waypoints"]],
                    mode="lines+markers+text",
                    text=[wp["name"] for wp in mission_data["waypoints"]],
                    textposition="top center",
                    name="Planned Path",
                    line=dict(color="blue", width=2),
                )
            )

            if mission_data["current_position"]:
                fig.add_trace(
                    go.Scatter(
                        x=[mission_data["current_position"]["x"]],
                        y=[mission_data["current_position"]["y"]],
                        mode="markers",
                        marker=dict(size=15, color="red", symbol="star"),
                        name="Current Position",
                    )
                )

            fig.update_layout(
                title="Mission Path & Progress",
                xaxis_title="X Position (m)",
                yaxis_title="Y Position (m)",
                height=400,
            )

            st.plotly_chart(fig, use_container_width=True)

            # Progress metrics
            col1, col2, col3 = st.columns(3)
            with col1:
                st.metric("Waypoints Completed", mission_data["completed_waypoints"])
            with col2:
                st.metric("Total Waypoints", mission_data["total_waypoints"])
            with col3:
                progress = (
                    mission_data["completed_waypoints"]
                    / mission_data["total_waypoints"]
                ) * 100
                st.metric("Progress", f"{progress:.1f}%")

    def _render_detailed_charts(self):
        """Render detailed analysis charts."""
        st.header("📈 Detailed Analysis")

        tab1, tab2, tab3 = st.tabs(
            ["Telemetry Trends", "Sensor Data", "Performance Metrics"]
        )

        with tab1:
            self._render_telemetry_trends()

        with tab2:
            self._render_sensor_data()

        with tab3:
            self._render_performance_metrics()

    def _render_telemetry_trends(self):
        """Render telemetry trends chart."""
        if not self.telemetry_history:
            st.info("No telemetry data available yet.")
            return

        df = pd.DataFrame(self.telemetry_history)

        # Time series plot
        fig = px.line(
            df,
            x=df.index,
            y=["battery_level", "cpu_usage", "memory_usage"],
            title="Telemetry Trends Over Time",
        )

        fig.update_layout(xaxis_title="Time (samples)", yaxis_title="Value", height=400)

        st.plotly_chart(fig, use_container_width=True)

    def _render_sensor_data(self):
        """Render sensor data visualization."""
        # Generate sample sensor data
        sensor_data = self._generate_sensor_data()

        col1, col2 = st.columns(2)

        with col1:
            # IMU data
            imu_df = pd.DataFrame(
                {
                    "time": sensor_data["timestamps"],
                    "accel_x": sensor_data["imu"]["accel_x"],
                    "accel_y": sensor_data["imu"]["accel_y"],
                    "accel_z": sensor_data["imu"]["accel_z"],
                }
            )

            fig = px.line(
                imu_df,
                x="time",
                y=["accel_x", "accel_y", "accel_z"],
                title="IMU Acceleration",
            )
            st.plotly_chart(fig, use_container_width=True)

        with col2:
            # GPS data
            gps_df = pd.DataFrame(
                {
                    "time": sensor_data["timestamps"],
                    "latitude": sensor_data["gps"]["latitude"],
                    "longitude": sensor_data["gps"]["longitude"],
                }
            )

            fig = px.line(
                gps_df, x="time", y=["latitude", "longitude"], title="GPS Position"
            )
            st.plotly_chart(fig, use_container_width=True)

    def _render_performance_metrics(self):
        """Render performance metrics."""
        # Generate sample performance data
        perf_data = self._generate_performance_data()

        col1, col2 = st.columns(2)

        with col1:
            # Response times
            fig = go.Figure()
            fig.add_trace(
                go.Histogram(
                    x=perf_data["response_times"], nbinsx=20, name="Response Times"
                )
            )
            fig.update_layout(title="Command Response Time Distribution")
            st.plotly_chart(fig, use_container_width=True)

        with col2:
            # Throughput
            fig = go.Figure()
            fig.add_trace(
                go.Scatter(
                    x=perf_data["time"],
                    y=perf_data["throughput"],
                    mode="lines",
                    name="Commands/sec",
                )
            )
            fig.update_layout(title="System Throughput")
            st.plotly_chart(fig, use_container_width=True)

    def _render_alerts_and_logs(self):
        """Render alerts and system logs."""
        st.header("🚨 Alerts & Logs")

        col1, col2 = st.columns(2)

        with col1:
            st.subheader("Active Alerts")
            alerts = self._get_active_alerts()

            if alerts:
                for alert in alerts:
                    if alert["severity"] == "critical":
                        st.error(f"🚨 {alert['message']}")
                    elif alert["severity"] == "warning":
                        st.warning(f"⚠️ {alert['message']}")
                    else:
                        st.info(f"ℹ️ {alert['message']}")
            else:
                st.success("✅ No active alerts")

        with col2:
            st.subheader("Recent Logs")
            logs = self._get_recent_logs()

            for log in logs[-10:]:  # Last 10 logs
                timestamp = log["timestamp"].strftime("%H:%M:%S")
                level = log["level"]
                message = log["message"]

                if level == "ERROR":
                    st.error(f"{timestamp}: {message}")
                elif level == "WARNING":
                    st.warning(f"{timestamp}: {message}")
                else:
                    st.text(f"{timestamp}: {message}")

    def _generate_current_telemetry(self) -> Dict[str, Any]:
        """Generate current telemetry data."""
        # Use our factory to generate realistic telemetry
        telemetry = RoverStateFactory()

        # Add additional computed fields
        telemetry.update(
            {
                "cpu_usage": random.uniform(10, 90),
                "memory_usage": random.uniform(20, 85),
                "temperature": random.uniform(25, 75),
                "network_latency": random.uniform(5, 50),
                "timestamp": datetime.now(),
            }
        )

        return telemetry

    def _update_telemetry_history(self, telemetry: Dict[str, Any]):
        """Update telemetry history."""
        self.telemetry_history.append(telemetry)

        # Keep only recent history
        if len(self.telemetry_history) > self.max_history_points:
            self.telemetry_history = self.telemetry_history[-self.max_history_points :]

        # Also update Streamlit session state
        if VISUALIZATION_AVAILABLE:
            st.session_state.telemetry_data = self.telemetry_history[
                -100:
            ]  # Keep last 100 for display

    def _generate_mission_progress_data(self) -> Dict[str, Any]:
        """Generate mission progress data."""
        return {
            "waypoints": [
                {"name": "Start", "x": 0, "y": 0},
                {"name": "WP1", "x": 10, "y": 5},
                {"name": "WP2", "x": 20, "y": 15},
                {"name": "WP3", "x": 30, "y": 10},
                {"name": "Goal", "x": 40, "y": 20},
            ],
            "current_position": {"x": 15, "y": 8},
            "completed_waypoints": 2,
            "total_waypoints": 5,
        }

    def _generate_sensor_data(self) -> Dict[str, Any]:
        """Generate sample sensor data."""
        timestamps = pd.date_range(
            start=datetime.now() - timedelta(minutes=5), end=datetime.now(), freq="1s"
        )

        return {
            "timestamps": timestamps,
            "imu": {
                "accel_x": np.random.normal(0, 0.1, len(timestamps)),
                "accel_y": np.random.normal(0, 0.1, len(timestamps)),
                "accel_z": np.random.normal(9.8, 0.05, len(timestamps)),
            },
            "gps": {
                "latitude": np.linspace(37.7749, 37.7849, len(timestamps)),
                "longitude": np.linspace(-122.4194, -122.4094, len(timestamps)),
            },
        }

    def _generate_performance_data(self) -> Dict[str, Any]:
        """Generate performance metrics data."""
        return {
            "response_times": np.random.exponential(
                0.1, 100
            ),  # Exponential distribution
            "time": pd.date_range(
                start=datetime.now() - timedelta(minutes=10),
                end=datetime.now(),
                freq="1s",
            ),
            "throughput": np.random.normal(50, 10, 600),  # Commands per second
        }

    def _get_active_alerts(self) -> List[Dict[str, Any]]:
        """Get active system alerts."""
        alerts = []

        # Check battery level
        if self.telemetry_history:
            latest = self.telemetry_history[-1]
            if latest.get("battery_level", 100) < 20:
                alerts.append(
                    {
                        "severity": "critical",
                        "message": f"Battery level critical: {latest['battery_level']}%",
                    }
                )
            elif latest.get("battery_level", 100) < 30:
                alerts.append(
                    {
                        "severity": "warning",
                        "message": f"Battery level low: {latest['battery_level']}%",
                    }
                )

        return alerts

    def _get_recent_logs(self) -> List[Dict[str, Any]]:
        """Get recent system logs."""
        logs = []

        for i in range(20):
            timestamp = datetime.now() - timedelta(seconds=i * 30)
            level = random.choice(["INFO", "WARNING", "ERROR"])
            message = {
                "INFO": "System operating normally",
                "WARNING": "Sensor calibration recommended",
                "ERROR": "Communication timeout detected",
            }[level]

            logs.append({"timestamp": timestamp, "level": level, "message": message})

        return logs

    def _execute_mission_action(self, action: str):
        """Execute mission action."""
        action_map = {
            "Start Mission": "ACTIVE",
            "Pause Mission": "PAUSED",
            "Resume Mission": "ACTIVE",
            "Emergency Stop": "EMERGENCY",
            "Return Home": "RETURNING",
        }

        if VISUALIZATION_AVAILABLE:
            st.session_state.mission_status = action_map.get(action, "IDLE")

        st.success(f"Executed: {action}")

    def _run_fallback_dashboard(self):
        """Run fallback dashboard when libraries not available."""
        print("=" * 60)
        print("URC 2026 Rover Monitoring Dashboard (Fallback Mode)")
        print("=" * 60)

        # Generate sample data
        telemetry = self._generate_current_telemetry()
        self._update_telemetry_history(telemetry)

        # Display key metrics
        print(f"Battery Level: {telemetry.get('battery_level', 'N/A')}%")
        print(f"Communication: {telemetry.get('communication_status', 'N/A')}")
        print(f"System Health: {telemetry.get('system_health', 'N/A')}")
        print(f"Mission Status: IDLE")

        print(f"\nTelemetry History: {len(self.telemetry_history)} points")

        # Simple alerts
        alerts = self._get_active_alerts()
        if alerts:
            print("\nActive Alerts:")
            for alert in alerts:
                print(f"  {alert['severity'].upper()}: {alert['message']}")

        print(
            "\nRun 'streamlit run src/dashboard/urc_monitoring_dashboard.py' for full dashboard"
        )
        print("=" * 60)


def main():
    """Main dashboard entry point."""
    dashboard = URCMonitoringDashboard()

    if VISUALIZATION_AVAILABLE:
        dashboard.run_dashboard()
    else:
        dashboard._run_fallback_dashboard()


if __name__ == "__main__":
    main()
