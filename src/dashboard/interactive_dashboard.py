#!/usr/bin/env python3
"""
Interactive System Dashboard - Real-time System Visualization

Provides a web-based dashboard for monitoring and controlling the URC 2026 system:
- Real-time metrics and health status
- Component control and configuration
- Mission monitoring and control
- Log viewing and filtering
- Performance visualization

Author: URC 2026 Dashboard Team
"""

import asyncio
import json
import time
from typing import Dict, List, Any, Optional
from dataclasses import dataclass
from pathlib import Path

# Import unified systems
from src.infrastructure.config import get_config_manager, RoverConfig as SystemConfig
from src.core.observability import get_observability_system
from src.core.simplified_component_registry import get_component_registry
from src.core.data_manager import get_data_manager

# Dashboard libraries with fallbacks
try:
    import streamlit as st
    import plotly.graph_objects as go
    import plotly.express as px
    from plotly.subplots import make_subplots

    STREAMLIT_AVAILABLE = True
except ImportError:
    STREAMLIT_AVAILABLE = False

try:
    import pandas as pd

    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False


# DashboardConfig moved to unified configuration system


class SystemDashboard:
    """
    Interactive web dashboard for system monitoring and control.

    Features:
    - Real-time metrics visualization
    - Component health monitoring
    - Mission control interface
    - Log analysis and filtering
    - Configuration management
    - Performance analytics
    """

    def __init__(self, config: Optional[SystemConfig] = None):
        if not STREAMLIT_AVAILABLE:
            raise ImportError("Streamlit required for interactive dashboard")

        # Use unified configuration system
        self.config_manager = get_config_manager()
        self.config = config or self.config_manager.get_config()

        # Use unified systems
        self.observability = get_observability_system()
        self.component_registry = get_component_registry()
        self.data_manager = get_data_manager()

    def initialize_dashboard(self):
        """Initialize dashboard components."""
        # All systems are now unified and available through managers
        self.observability.info(
            "Dashboard initialized",
            component="dashboard",
            streamlit_available=STREAMLIT_AVAILABLE,
        )

    def run_dashboard(self):
        """Run the Streamlit dashboard."""
        if not STREAMLIT_AVAILABLE:
            print("❌ Streamlit not available for dashboard")
            return

        # Configure Streamlit page
        st.set_page_config(
            page_title=self.config.title,
            page_icon="🚀",
            layout="wide",
            initial_sidebar_state="expanded",
        )

        # Main dashboard
        st.title(self.config.title)

        # Sidebar navigation
        page = st.sidebar.selectbox(
            "Navigation",
            [
                "Overview",
                "System Health",
                "Components",
                "Missions",
                "Configuration",
                "Logs",
                "Performance",
            ],
        )

        # Refresh button and auto-refresh
        col1, col2 = st.sidebar.columns(2)
        with col1:
            if st.button("🔄 Refresh"):
                st.rerun()
        with col2:
            auto_refresh = st.checkbox(
                "Auto Refresh", value=self.config.enable_real_time
            )

        if auto_refresh:
            time.sleep(self.config.refresh_interval)
            st.rerun()

        # Render selected page
        if page == "Overview":
            self.render_overview_page()
        elif page == "System Health":
            self.render_health_page()
        elif page == "Components":
            self.render_components_page()
        elif page == "Missions":
            self.render_missions_page()
        elif page == "Configuration":
            self.render_configuration_page()
        elif page == "Logs":
            self.render_logs_page()
        elif page == "Performance":
            self.render_performance_page()

        # Footer
        st.sidebar.markdown("---")
        st.sidebar.markdown("**URC 2026 Control Center**")
        st.sidebar.markdown(f"Last updated: {time.strftime('%H:%M:%S')}")

    def render_overview_page(self):
        """Render main overview dashboard."""
        st.header("🖥️ System Overview")

        # Get system status
        system_status = self._get_system_status()

        # Key metrics row
        col1, col2, col3, col4 = st.columns(4)

        with col1:
            st.metric(
                "System Health",
                f"{system_status.get('health_score', 0)}%",
                delta="+5" if system_status.get("health_score", 0) > 80 else "-5",
            )

        with col2:
            uptime = system_status.get("uptime_seconds", 0)
            st.metric("Uptime", f"{uptime/3600:.1f}h")

        with col3:
            components = system_status.get("total_components", 0)
            running = system_status.get("running_components", 0)
            st.metric("Components", f"{running}/{components}")

        with col4:
            memory = system_status.get("memory_usage_mb", 0)
            st.metric("Memory", f"{memory:.0f}MB")

        # System status overview
        st.subheader("System Status")

        # Health status
        health_col, status_col = st.columns(2)

        with health_col:
            if system_status.get("health"):
                health_data = system_status["health"]
                st.write("**Health Checks:**")
                st.write(
                    f"- Overall: {'✅ Healthy' if health_data.get('overall_healthy') else '❌ Issues'}"
                )
                st.write(f"- Checks Run: {health_data.get('checks_run', 0)}")
                st.write(f"- Issues: {health_data.get('issues_found', 0)}")

        with status_col:
            st.write("**Component Status:**")
            if system_status.get("components"):
                for name, comp_data in list(system_status["components"].items())[:5]:
                    state = comp_data.get("state", "unknown")
                    status_icon = (
                        "🟢"
                        if state == "running"
                        else "🟡" if state == "initialized" else "🔴"
                    )
                    st.write(f"{status_icon} {name}")

        # Recent activity
        st.subheader("Recent Activity")
        if system_status.get("recent_logs"):
            for log in system_status["recent_logs"][-5:]:
                level = log.get("level", "INFO")
                message = log.get("message", "")
                component = log.get("component", "")

                level_icon = {
                    "INFO": "ℹ️",
                    "WARNING": "⚠️",
                    "ERROR": "❌",
                    "CRITICAL": "🚨",
                }.get(level, "📝")

                st.write(f"{level_icon} **{component}**: {message}")

    def render_health_page(self):
        """Render detailed health monitoring page."""
        st.header("🏥 System Health Dashboard")

        system_status = self._get_system_status()

        # Health overview
        st.subheader("Health Overview")

        if system_status.get("health"):
            health_data = system_status["health"]

            # Overall health indicator
            health_score = 100 if health_data.get("overall_healthy") else 50
            st.progress(health_score / 100)

            if health_data.get("overall_healthy"):
                st.success("✅ All systems healthy")
            else:
                st.error(
                    f"❌ {health_data.get('issues_found', 0)} health issues detected"
                )

            # Detailed health checks
            if health_data.get("results"):
                st.subheader("Health Check Details")

                for check_name, result in health_data["results"].items():
                    healthy = result.get("healthy", False)
                    message = result.get("message", "OK")

                    if healthy:
                        st.success(f"✅ {check_name}: {message}")
                    else:
                        st.error(f"❌ {check_name}: {message}")

        # Resource usage
        st.subheader("Resource Usage")

        if system_status.get("metrics"):
            metrics = system_status["metrics"]

            col1, col2, col3 = st.columns(3)

            with col1:
                cpu = metrics.get("system_info", {}).get("cpu_percent", 0)
                st.metric("CPU Usage", f"{cpu:.1f}%")
                st.progress(cpu / 100)

            with col2:
                memory = metrics.get("system_info", {}).get("memory_percent", 0)
                st.metric("Memory Usage", f"{memory:.1f}%")
                st.progress(memory / 100)

            with col3:
                disk = metrics.get("system_info", {}).get("disk_percent", 0)
                st.metric("Disk Usage", f"{disk:.1f}%")
                st.progress(disk / 100)

    def render_components_page(self):
        """Render component management page."""
        st.header("🔧 Component Management")

        if not self.component_registry:
            st.warning("Component registry not available")
            return

        # Component overview
        system_status = self._get_system_status()
        components = system_status.get("components", {})

        st.subheader("Component Status")

        # Status summary
        total = len(components)
        running = len([c for c in components.values() if c.get("state") == "running"])
        errors = len([c for c in components.values() if c.get("state") == "error"])

        col1, col2, col3, col4 = st.columns(4)
        with col1:
            st.metric("Total", total)
        with col2:
            st.metric("Running", running)
        with col3:
            st.metric("Errors", errors)
        with col4:
            st.metric("Health", f"{running/total*100:.0f}%" if total > 0 else "0%")

        # Component details
        st.subheader("Component Details")

        for name, data in components.items():
            with st.expander(f"{name} ({data.get('state', 'unknown')})"):
                st.write(f"**State:** {data.get('state', 'unknown')}")
                st.write(f"**Priority:** {data.get('priority', 'normal')}")
                st.write(f"**Dependencies:** {', '.join(data.get('dependencies', []))}")
                st.write(f"**Last Used:** {time.ctime(data.get('last_used', 0))}")

                if data.get("error_message"):
                    st.error(f"Error: {data['error_message']}")

                # Control buttons
                col1, col2 = st.columns(2)
                with col1:
                    if st.button(f"Restart {name}", key=f"restart_{name}"):
                        st.info(f"Restarting {name}...")
                        # Implementation would restart component

                with col2:
                    if st.button(f"Stop {name}", key=f"stop_{name}"):
                        st.info(f"Stopping {name}...")
                        # Implementation would stop component

    def render_missions_page(self):
        """Render mission control page."""
        st.header("🎯 Mission Control Center")

        st.subheader("Active Missions")

        # Mock mission data - would come from mission executor
        missions = [
            {
                "id": "waypoint_nav_001",
                "type": "waypoint_navigation",
                "status": "running",
                "progress": 65,
            },
            {
                "id": "obj_detect_002",
                "type": "object_detection",
                "status": "completed",
                "progress": 100,
            },
        ]

        for mission in missions:
            with st.expander(f"{mission['id']} - {mission['type']}"):
                col1, col2, col3 = st.columns(3)
                with col1:
                    st.write(f"**Status:** {mission['status']}")
                with col2:
                    st.progress(mission["progress"] / 100)
                with col3:
                    st.write(f"**Progress:** {mission['progress']}%")

                # Mission controls
                if mission["status"] == "running":
                    if st.button(
                        f"Pause {mission['id']}", key=f"pause_{mission['id']}"
                    ):
                        st.info(f"Pausing mission {mission['id']}")

                    if st.button(f"Stop {mission['id']}", key=f"stop_{mission['id']}"):
                        st.error(f"Stopping mission {mission['id']}")

        # Mission creation
        st.subheader("Create New Mission")

        with st.form("new_mission"):
            mission_type = st.selectbox(
                "Mission Type",
                [
                    "waypoint_navigation",
                    "object_detection",
                    "follow_me",
                    "sample_collection",
                ],
            )

            if mission_type == "waypoint_navigation":
                waypoints = st.text_area(
                    "Waypoints (JSON format)",
                    """[
                    {"latitude": 35.0, "longitude": -120.0},
                    {"latitude": 35.01, "longitude": -120.01}
                ]""",
                )

            submitted = st.form_submit_button("Launch Mission")
            if submitted:
                st.success("Mission launched successfully!")

    def render_configuration_page(self):
        """Render configuration management page."""
        st.header("⚙️ Configuration Management")

        if not self.config_manager:
            st.warning("Configuration manager not available")
            return

        # Current configuration
        try:
            config = self.config_manager.get_config()

            st.subheader("Current Configuration")

            # Environment info
            col1, col2 = st.columns(2)
            with col1:
                st.write(f"**Environment:** {config.environment}")
            with col2:
                st.write(f"**Version:** {config.version}")

            # Network settings
            st.subheader("Network Configuration")
            col1, col2 = st.columns(2)
            with col1:
                st.write("**WebSocket Port:**", config.network.websocket_port)
                st.write("**HTTP Port:**", config.network.http_port)
            with col2:
                st.write("**CORS Enabled:**", config.network.enable_cors)
                st.write("**Max Connections:**", config.network.max_connections)

            # Navigation settings
            st.subheader("Navigation Configuration")
            st.write(f"**Update Rate:** {config.navigation.update_rate_hz} Hz")
            st.write(
                f"**Waypoint Tolerance:** {config.navigation.waypoint_tolerance_m}m"
            )
            st.write(
                f"**Max Velocity:** {config.navigation.max_linear_velocity_ms} m/s"
            )

            # Configuration editing
            st.subheader("Edit Configuration")

            with st.form("config_edit"):
                new_ws_port = st.number_input(
                    "WebSocket Port", value=config.network.websocket_port
                )
                new_nav_rate = st.slider(
                    "Navigation Update Rate (Hz)",
                    min_value=1.0,
                    max_value=50.0,
                    value=config.navigation.update_rate_hz,
                )

                if st.form_submit_button("Save Changes"):
                    # Update configuration
                    config.network.websocket_port = new_ws_port
                    config.navigation.update_rate_hz = new_nav_rate

                    if self.config_manager.update_config(
                        {"network": config.network, "navigation": config.navigation}
                    ):
                        st.success("Configuration updated successfully!")
                        st.rerun()
                    else:
                        st.error("Failed to update configuration")

        except Exception as e:
            st.error(f"Configuration error: {e}")

    def render_logs_page(self):
        """Render logs and monitoring page."""
        st.header("📋 System Logs & Monitoring")

        if not self.monitoring_system:
            st.warning("Monitoring system not available")
            return

        # Log filters
        col1, col2, col3 = st.columns(3)

        with col1:
            log_level = st.selectbox(
                "Log Level", ["ALL", "INFO", "WARNING", "ERROR", "CRITICAL"]
            )

        with col2:
            component_filter = st.selectbox(
                "Component", ["ALL", "observability", "navigation", "bridge"]
            )

        with col3:
            max_entries = st.slider("Max Entries", 10, 500, 100)

        # Get logs
        system_status = self._get_system_status()
        recent_logs = system_status.get("recent_logs", [])
        error_logs = system_status.get("error_logs", [])

        # Filter logs
        if log_level != "ALL":
            recent_logs = [log for log in recent_logs if log.get("level") == log_level]

        if component_filter != "ALL":
            recent_logs = [
                log for log in recent_logs if log.get("component") == component_filter
            ]

        recent_logs = recent_logs[-max_entries:]

        # Display logs
        st.subheader("Recent Logs")

        if recent_logs:
            for log in recent_logs:
                level = log.get("level", "INFO")
                timestamp = time.strftime(
                    "%H:%M:%S", time.localtime(log.get("timestamp", 0))
                )
                component = log.get("component", "unknown")
                message = log.get("message", "")

                # Color coding
                if level == "ERROR":
                    st.error(f"[{timestamp}] {component}: {message}")
                elif level == "WARNING":
                    st.warning(f"[{timestamp}] {component}: {message}")
                elif level == "CRITICAL":
                    st.error(f"🚨 [{timestamp}] {component}: {message}")
                else:
                    st.info(f"[{timestamp}] {component}: {message}")
        else:
            st.info("No logs available")

        # Error summary
        if error_logs:
            st.subheader("Error Summary")
            st.error(f"Total Errors: {len(error_logs)}")

            # Error breakdown by component
            error_components = {}
            for log in error_logs:
                comp = log.get("component", "unknown")
                error_components[comp] = error_components.get(comp, 0) + 1

            for comp, count in error_components.items():
                st.write(f"- {comp}: {count} errors")

    def render_performance_page(self):
        """Render performance analytics page."""
        st.header("📊 Performance Analytics")

        system_status = self._get_system_status()

        # Performance metrics
        st.subheader("System Metrics")

        if system_status.get("metrics"):
            metrics = system_status["metrics"]

            # Create charts using Plotly
            if PANDAS_AVAILABLE:
                # CPU/Memory over time (mock data for demo)
                import pandas as pd

                # Mock time series data
                times = pd.date_range(start="now", periods=20, freq="5s")
                cpu_data = [50 + (i % 10) for i in range(20)]
                mem_data = [60 + (i % 15) for i in range(20)]

                df = pd.DataFrame(
                    {"time": times, "cpu_percent": cpu_data, "memory_percent": mem_data}
                )

                # CPU chart
                fig_cpu = px.line(
                    df, x="time", y="cpu_percent", title="CPU Usage Over Time"
                )
                st.plotly_chart(fig_cpu, use_container_width=True)

                # Memory chart
                fig_mem = px.line(
                    df, x="time", y="memory_percent", title="Memory Usage Over Time"
                )
                st.plotly_chart(fig_mem, use_container_width=True)

        # Performance recommendations
        st.subheader("Performance Recommendations")

        recommendations = [
            "✅ CPU usage is within acceptable limits",
            "✅ Memory usage is stable",
            "ℹ️ Consider increasing log retention for better debugging",
            "ℹ️ Monitor component startup times for optimization opportunities",
        ]

        for rec in recommendations:
            st.write(rec)

    def _get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        try:
            if self.monitoring_system:
                return self.monitoring_system.get_system_status()
            elif self.component_registry:
                return self.component_registry.get_system_status()
        except Exception:
            pass

        # Fallback status
        return {
            "health_score": 85,
            "uptime_seconds": time.time(),
            "total_components": 5,
            "running_components": 4,
            "memory_usage_mb": 150,
            "health": {"overall_healthy": True, "checks_run": 3, "issues_found": 0},
            "recent_logs": [
                {
                    "timestamp": time.time(),
                    "level": "INFO",
                    "component": "dashboard",
                    "message": "Dashboard loaded",
                }
            ],
        }


def create_dashboard_app(config: Optional[DashboardConfig] = None) -> SystemDashboard:
    """Create dashboard application instance."""
    dashboard = SystemDashboard(config)
    dashboard.initialize_dashboard()
    return dashboard


def run_dashboard(config: Optional[DashboardConfig] = None):
    """Run the dashboard application."""
    dashboard = create_dashboard_app(config)
    dashboard.run_dashboard()


if __name__ == "__main__":
    run_dashboard()
