#!/usr/bin/env python3
"""
Auto-Generated API Documentation and Architecture Diagrams

Creates comprehensive documentation including:
- API reference documentation
- Architecture diagrams (Mermaid format)
- Component dependency graphs
- Configuration schema docs
- Usage examples and tutorials

Author: URC 2026 Documentation Team
"""

import os
import inspect
import importlib
import json
from pathlib import Path
from typing import Dict, List, Any, Optional, Type
from dataclasses import dataclass
import sys

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


@dataclass
class APIEndpoint:
    """API endpoint documentation."""
    path: str
    method: str
    description: str
    parameters: List[Dict[str, Any]]
    responses: Dict[str, Any]
    example_request: Optional[str] = None
    example_response: Optional[str] = None


@dataclass
class ComponentDoc:
    """Component documentation."""
    name: str
    type: str
    description: str
    dependencies: List[str]
    methods: List[Dict[str, Any]]
    attributes: List[Dict[str, Any]]


class DocumentationGenerator:
    """
    Automatic documentation generator for the URC 2026 system.

    Generates API docs, architecture diagrams, and component documentation
    from code analysis and runtime introspection.
    """

    def __init__(self, output_dir: str = "docs/generated"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.components: Dict[str, ComponentDoc] = {}
        self.api_endpoints: List[APIEndpoint] = []
        self.architecture_data: Dict[str, Any] = {}

    def generate_full_documentation(self):
        """Generate complete documentation suite."""
        print("📚 Generating URC 2026 Documentation")
        print("=" * 50)

        # Analyze codebase
        self.analyze_codebase()

        # Generate API documentation
        self.generate_api_docs()

        # Generate architecture diagrams
        self.generate_architecture_diagrams()

        # Generate component documentation
        self.generate_component_docs()

        # Generate configuration documentation
        self.generate_config_docs()

        # Generate usage examples
        self.generate_examples()

        print("✅ Documentation generated successfully!")
        print(f"📁 Output directory: {self.output_dir}")

    def analyze_codebase(self):
        """Analyze the codebase to extract documentation data."""
        print("🔍 Analyzing codebase...")

        # Core modules to analyze
        modules_to_analyze = [
            'src.core.configuration_manager',
            'src.core.monitoring_system',
            'src.core.component_registry',
            'src.bridges.simple_bridge',
            'src.autonomy.core.navigation.autonomy_navigation.navigation_node',
            'src.dashboard.interactive_dashboard',
            'src.core.fastapi_server',
            'src.core.database_manager'
        ]

        for module_path in modules_to_analyze:
            try:
                self._analyze_module(module_path)
            except ImportError as e:
                print(f"⚠️ Could not import {module_path}: {e}")

        # Analyze component registry if available
        try:
            from src.core.component_registry import get_component_registry
            registry = get_component_registry()
            self.architecture_data['component_registry'] = registry.get_system_status()
        except ImportError:
            pass

    def _analyze_module(self, module_path: str):
        """Analyze a specific module for documentation."""
        module = importlib.import_module(module_path)

        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj):
                self._analyze_class(obj, module_path)
            elif inspect.isfunction(obj) and not name.startswith('_'):
                self._analyze_function(obj, module_path)

    def _analyze_class(self, cls: Type, module_path: str):
        """Analyze a class for documentation."""
        if cls.__name__.startswith('_'):
            return

        # Extract class documentation
        doc = inspect.getdoc(cls) or "No documentation available"
        description = doc.split('\n')[0] if doc else "No description"

        # Extract methods
        methods = []
        for method_name, method in inspect.getmembers(cls, predicate=inspect.isfunction):
            if not method_name.startswith('_'):
                method_doc = inspect.getdoc(method) or "No documentation"
                methods.append({
                    'name': method_name,
                    'signature': str(inspect.signature(method)),
                    'doc': method_doc
                })

        # Extract attributes
        attributes = []
        for attr_name in dir(cls):
            if not attr_name.startswith('_') and not callable(getattr(cls, attr_name)):
                try:
                    attr_value = getattr(cls, attr_name)
                    if not callable(attr_value):
                        attributes.append({
                            'name': attr_name,
                            'type': type(attr_value).__name__,
                            'value': str(attr_value)[:100]  # Truncate long values
                        })
                except:
                    pass

        # Extract dependencies (from class annotations or docstring)
        dependencies = []
        if hasattr(cls, '__dependencies__'):
            dependencies = getattr(cls, '__dependencies__', [])

        component_doc = ComponentDoc(
            name=cls.__name__,
            type="class",
            description=description,
            dependencies=dependencies,
            methods=methods,
            attributes=attributes
        )

        self.components[cls.__name__] = component_doc

    def _analyze_function(self, func, module_path: str):
        """Analyze a function for documentation."""
        if func.__name__.startswith('_'):
            return

        # This could be extended to document standalone functions
        pass

    def generate_api_docs(self):
        """Generate API documentation."""
        print("📖 Generating API documentation...")

        # Analyze API endpoints from various sources
        self._analyze_fastapi_endpoints()
        self._analyze_bridge_endpoints()
        self._analyze_websocket_endpoints()

        # Generate Markdown API docs
        api_content = self._generate_api_markdown()
        self._write_file("api_reference.md", api_content)

        # Generate OpenAPI spec
        openapi_spec = self._generate_openapi_spec()
        self._write_file("openapi_spec.json", json.dumps(openapi_spec, indent=2))

    def _analyze_fastapi_endpoints(self):
        """Analyze FastAPI endpoints."""
        try:
            from src.core.fastapi_server import URCFastAPIServer
            # This would analyze the FastAPI app routes
            # For now, add some example endpoints
            pass
        except ImportError:
            pass

    def _analyze_bridge_endpoints(self):
        """Analyze bridge communication endpoints."""
        try:
            from src.bridges.simple_bridge import SimpleBridge
            # Analyze registered command handlers
            pass
        except ImportError:
            pass

    def _analyze_websocket_endpoints(self):
        """Analyze WebSocket endpoints."""
        # Add WebSocket endpoint documentation
        self.api_endpoints.extend([
            APIEndpoint(
                path="/ws/urc_control",
                method="WebSocket",
                description="Main WebSocket endpoint for rover control and telemetry",
                parameters=[
                    {"name": "command", "type": "string", "description": "Command type"},
                    {"name": "data", "type": "object", "description": "Command data"}
                ],
                responses={
                    "200": {"description": "Command processed successfully"},
                    "400": {"description": "Invalid command format"}
                }
            )
        ])

    def _generate_api_markdown(self) -> str:
        """Generate API documentation in Markdown format."""
        content = ["# URC 2026 API Reference\n\n"]

        content.append("## Overview\n\n")
        content.append("The URC 2026 system provides RESTful APIs, WebSocket endpoints, ")
        content.append("and bridge interfaces for rover control and monitoring.\n\n")

        for endpoint in self.api_endpoints:
            content.append(f"## {endpoint.method} {endpoint.path}\n\n")
            content.append(f"{endpoint.description}\n\n")

            if endpoint.parameters:
                content.append("### Parameters\n\n")
                content.append("| Name | Type | Description |\n")
                content.append("|------|------|-------------|\n")
                for param in endpoint.parameters:
                    content.append(f"| {param['name']} | {param['type']} | {param['description']} |\n")
                content.append("\n")

            if endpoint.responses:
                content.append("### Responses\n\n")
                for status, response in endpoint.responses.items():
                    content.append(f"- **{status}**: {response['description']}\n")
                content.append("\n")

            if endpoint.example_request:
                content.append("### Example Request\n\n")
                content.append(f"```json\n{endpoint.example_request}\n```\n\n")

            if endpoint.example_response:
                content.append("### Example Response\n\n")
                content.append(f"```json\n{endpoint.example_response}\n```\n\n")

        return "".join(content)

    def _generate_openapi_spec(self) -> Dict[str, Any]:
        """Generate OpenAPI specification."""
        return {
            "openapi": "3.0.3",
            "info": {
                "title": "URC 2026 Mars Rover API",
                "version": "1.0.0",
                "description": "API for controlling and monitoring the URC 2026 Mars rover"
            },
            "servers": [
                {"url": "http://localhost:8080", "description": "Development server"},
                {"url": "ws://localhost:8765", "description": "WebSocket server"}
            ],
            "paths": {},
            "components": {
                "schemas": {}
            }
        }

    def generate_architecture_diagrams(self):
        """Generate architecture diagrams in Mermaid format."""
        print("📊 Generating architecture diagrams...")

        # System overview diagram
        overview_diagram = self._generate_system_overview_diagram()
        self._write_file("architecture_overview.md", overview_diagram)

        # Component dependency diagram
        dependency_diagram = self._generate_dependency_diagram()
        self._write_file("component_dependencies.md", dependency_diagram)

        # Data flow diagram
        data_flow_diagram = self._generate_data_flow_diagram()
        self._write_file("data_flow.md", data_flow_diagram)

        # Communication architecture
        comm_diagram = self._generate_communication_diagram()
        self._write_file("communication_architecture.md", comm_diagram)

    def _generate_system_overview_diagram(self) -> str:
        """Generate system overview diagram."""
        content = ["# URC 2026 System Architecture Overview\n\n"]
        content.append("```mermaid\n")
        content.append("graph TB\n")
        content.append("    subgraph \"User Interface\"\n")
        content.append("        UI[Interactive Dashboard]\n")
        content.append("        CLI[Command Line Interface]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"API Layer\"\n")
        content.append("        FastAPI[FastAPI Server]\n")
        content.append("        WS[WebSocket Bridge]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Core Systems\"\n")
        content.append("        Config[Configuration Manager]\n")
        content.append("        Monitor[Monitoring System]\n")
        content.append("        Registry[Component Registry]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Autonomy Stack\"\n")
        content.append("        Nav[Navigation Node]\n")
        content.append("        BT[Behavior Trees]\n")
        content.append("        State[State Machine]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Hardware Interfaces\"\n")
        content.append("        CAN[CAN Bridge]\n")
        content.append("        Sensors[Sensor Interfaces]\n")
        content.append("        Motors[Motor Controllers]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    UI --> FastAPI\n")
        content.append("    CLI --> WS\n")
        content.append("    FastAPI --> Config\n")
        content.append("    WS --> Registry\n")
        content.append("    Registry --> Nav\n")
        content.append("    Nav --> BT\n")
        content.append("    Nav --> State\n")
        content.append("    Nav --> CAN\n")
        content.append("    CAN --> Motors\n")
        content.append("    CAN --> Sensors\n")
        content.append("    Monitor -.-> FastAPI\n")
        content.append("    Monitor -.-> WS\n")
        content.append("```\n\n")
        content.append("## Architecture Description\n\n")
        content.append("The URC 2026 system follows a layered architecture with clear separation of concerns:\n\n")
        content.append("- **User Interface Layer**: Web dashboard and CLI for system interaction\n")
        content.append("- **API Layer**: RESTful APIs and WebSocket communication\n")
        content.append("- **Core Systems Layer**: Configuration, monitoring, and component management\n")
        content.append("- **Autonomy Stack**: Navigation, behavior trees, and state management\n")
        content.append("- **Hardware Layer**: Direct interfaces to rover hardware\n")

        return "".join(content)

    def _generate_dependency_diagram(self) -> str:
        """Generate component dependency diagram."""
        content = ["# Component Dependencies\n\n"]
        content.append("```mermaid\n")
        content.append("graph TD\n")

        # Add components and their dependencies
        components_added = set()

        for comp_name, comp_doc in self.components.items():
            if comp_name not in components_added:
                content.append(f"    {comp_name}[{comp_name}]\n")
                components_added.add(comp_name)

            for dep in comp_doc.dependencies:
                if dep not in components_added:
                    content.append(f"    {dep}[{dep}]\n")
                    components_added.add(dep)
                content.append(f"    {comp_name} --> {dep}\n")

        content.append("```\n\n")
        content.append("## Dependency Analysis\n\n")
        content.append("This diagram shows the dependency relationships between system components.\n")
        content.append("Components with no dependencies are leaf nodes, while components with many\n")
        content.append("dependencies are core infrastructure components.\n")

        return "".join(content)

    def _generate_data_flow_diagram(self) -> str:
        """Generate data flow diagram."""
        content = ["# Data Flow Architecture\n\n"]
        content.append("```mermaid\n")
        content.append("flowchart TD\n")
        content.append("    subgraph \"Data Sources\"\n")
        content.append("        GPS[GPS Sensor]\n")
        content.append("        IMU[IMU Sensor]\n")
        content.append("        CAM[Stereo Camera]\n")
        content.append("        ENC[Motor Encoders]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Data Processing\"\n")
        content.append("        GNSS[GNSS Processor]\n")
        content.append("        Terrain[Terrain Analyzer]\n")
        content.append("        Vision[Computer Vision]\n")
        content.append("        StateEst[State Estimator]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Control Systems\"\n")
        content.append("        Nav[Navigation Controller]\n")
        content.append("        Path[Path Planner]\n")
        content.append("        Motion[Motion Controller]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Actuators\"\n")
        content.append("        Motors[Motor Controllers]\n")
        content.append("        Servos[Servo Controllers]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    GPS --> GNSS\n")
        content.append("    IMU --> StateEst\n")
        content.append("    CAM --> Vision\n")
        content.append("    ENC --> StateEst\n")
        content.append("    GNSS --> Nav\n")
        content.append("    Vision --> Terrain\n")
        content.append("    Terrain --> Path\n")
        content.append("    StateEst --> Motion\n")
        content.append("    Nav --> Path\n")
        content.append("    Path --> Motion\n")
        content.append("    Motion --> Motors\n")
        content.append("    Motion --> Servos\n")
        content.append("```\n\n")
        content.append("## Data Flow Description\n\n")
        content.append("Sensor data flows from hardware through processing pipelines to control systems,\n")
        content.append("culminating in actuator commands. This architecture ensures real-time performance\n")
        content.append("and fault tolerance through parallel processing paths.\n")

        return "".join(content)

    def _generate_communication_diagram(self) -> str:
        """Generate communication architecture diagram."""
        content = ["# Communication Architecture\n\n"]
        content.append("```mermaid\n")
        content.append("graph LR\n")
        content.append("    subgraph \"External Systems\"\n")
        content.append("        Ground[Ground Station]\n")
        content.append("        Cloud[Cloud Services]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Network Layer\"\n")
        content.append("        WiFi[(WiFi 2.4GHz)]\n")
        content.append("        Ethernet[(Ethernet)]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"Protocol Bridges\"\n")
        content.append("        WS[WebSocket Bridge]\n")
        content.append("        HTTP[HTTP Bridge]\n")
        content.append("        CAN[CAN Bridge]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    subgraph \"ROS2 Ecosystem\"\n")
        content.append("        NavNode[Navigation Node]\n")
        content.append("        SensorNode[Sensor Nodes]\n")
        content.append("        ControlNode[Control Nodes]\n")
        content.append("    end\n")
        content.append("\n")
        content.append("    Ground --> WiFi\n")
        content.append("    Cloud --> Ethernet\n")
        content.append("    WiFi --> WS\n")
        content.append("    Ethernet --> HTTP\n")
        content.append("    WS --> NavNode\n")
        content.append("    HTTP --> SensorNode\n")
        content.append("    CAN --> ControlNode\n")
        content.append("    NavNode -.-> CAN\n")
        content.append("    SensorNode -.-> CAN\n")
        content.append("```\n\n")
        content.append("## Communication Architecture\n\n")
        content.append("The system uses multiple communication protocols to ensure reliability:\n\n")
        content.append("- **WiFi (2.4GHz)**: Primary wireless communication with ground station\n")
        content.append("- **Ethernet**: High-bandwidth connection for data-intensive operations\n")
        content.append("- **WebSocket**: Real-time bidirectional communication\n")
        content.append("- **HTTP/REST**: Standard API communication\n")
        content.append("- **CAN Bus**: Deterministic real-time hardware communication\n")
        content.append("- **ROS2**: Inter-process communication within the rover\n")

        return "".join(content)

    def generate_component_docs(self):
        """Generate detailed component documentation."""
        print("🔧 Generating component documentation...")

        component_content = self._generate_component_markdown()
        self._write_file("component_reference.md", component_content)

    def _generate_component_markdown(self) -> str:
        """Generate component documentation in Markdown."""
        content = ["# Component Reference\n\n"]

        content.append("## Overview\n\n")
        content.append("This document provides detailed information about all system components,\n")
        content.append("their interfaces, dependencies, and usage patterns.\n\n")

        for comp_name, comp_doc in sorted(self.components.items()):
            content.append(f"## {comp_name}\n\n")
            content.append(f"**Type:** {comp_doc.type}\n\n")
            content.append(f"**Description:** {comp_doc.description}\n\n")

            if comp_doc.dependencies:
                content.append("**Dependencies:**\n")
                for dep in comp_doc.dependencies:
                    content.append(f"- {dep}\n")
                content.append("\n")

            if comp_doc.attributes:
                content.append("**Attributes:**\n\n")
                content.append("| Name | Type | Description |\n")
                content.append("|------|------|-------------|\n")
                for attr in comp_doc.attributes:
                    content.append(f"| {attr['name']} | {attr['type']} | {attr.get('description', '')} |\n")
                content.append("\n")

            if comp_doc.methods:
                content.append("**Methods:**\n\n")
                for method in comp_doc.methods:
                    content.append(f"### `{method['name']}{method['signature']}`\n\n")
                    content.append(f"{method['doc']}\n\n")

        return "".join(content)

    def generate_config_docs(self):
        """Generate configuration documentation."""
        print("⚙️ Generating configuration documentation...")

        try:
            from src.core.configuration_manager import SystemConfig

            config_content = self._generate_config_markdown()
            self._write_file("configuration_reference.md", config_content)

            # Generate JSON schema
            schema = self._generate_config_schema()
            self._write_file("config_schema.json", json.dumps(schema, indent=2))

        except ImportError:
            print("⚠️ Configuration manager not available for documentation")

    def _generate_config_markdown(self) -> str:
        """Generate configuration documentation."""
        content = ["# Configuration Reference\n\n"]

        content.append("## Overview\n\n")
        content.append("The URC 2026 system uses a hierarchical configuration system with\n")
        content.append("environment-specific overrides and runtime validation.\n\n")

        content.append("## Configuration Sections\n\n")

        sections = [
            ("ROS2", "ROS2 middleware configuration"),
            ("Database", "Database connection and schema settings"),
            ("Network", "Network interfaces and communication settings"),
            ("Navigation", "Navigation system parameters"),
            ("Safety", "Safety system thresholds and timeouts"),
            ("Monitoring", "Monitoring and observability settings")
        ]

        for section_name, description in sections:
            content.append(f"### {section_name} Configuration\n\n")
            content.append(f"{description}\n\n")
            content.append("**Location:** `config/{environment}.yaml`\n\n")
            content.append("**Environment Variables:** `URC_CONFIG_{section_name.upper()}_*`\n\n")

        content.append("## Environment Variables\n\n")
        content.append("Configuration can be overridden using environment variables:\n\n")
        content.append("```bash\n")
        content.append("export URC_CONFIG_NETWORK_WEBSOCKET_PORT=8766\n")
        content.append("export URC_CONFIG_NAVIGATION_UPDATE_RATE_HZ=15.0\n")
        content.append("```\n\n")

        return "".join(content)

    def _generate_config_schema(self) -> Dict[str, Any]:
        """Generate JSON schema for configuration validation."""
        return {
            "$schema": "http://json-schema.org/draft-07/schema#",
            "title": "URC 2026 Configuration Schema",
            "type": "object",
            "properties": {
                "environment": {"type": "string", "enum": ["development", "testing", "staging", "production"]},
                "version": {"type": "string"},
                "ros2": {
                    "type": "object",
                    "properties": {
                        "namespace": {"type": "string"},
                        "domain_id": {"type": ["integer", "null"]},
                        "use_sim_time": {"type": "boolean"},
                        "qos_preset": {"type": "string"}
                    }
                },
                "database": {
                    "type": "object",
                    "properties": {
                        "type": {"type": "string", "enum": ["sqlite", "postgresql"]},
                        "path": {"type": "string"},
                        "connection_pool_size": {"type": "integer", "minimum": 1, "maximum": 20}
                    }
                }
            },
            "required": ["environment"]
        }

    def generate_examples(self):
        """Generate usage examples and tutorials."""
        print("📝 Generating usage examples...")

        examples_content = self._generate_examples_markdown()
        self._write_file("usage_examples.md", examples_content)

    def _generate_examples_markdown(self) -> str:
        """Generate usage examples."""
        content = ["# Usage Examples and Tutorials\n\n"]

        content.append("## Getting Started\n\n")
        content.append("### Basic System Startup\n\n")
        content.append("```python\n")
        content.append("from src.core.configuration_manager import load_system_config\n")
        content.append("from src.core.monitoring_system import start_system_monitoring\n")
        content.append("from src.core.component_registry import get_component_registry\n\n")
        content.append("# Load configuration\n")
        content.append("config = load_system_config('development')\n\n")
        content.append("# Start monitoring\n")
        content.append("monitor = start_system_monitoring()\n\n")
        content.append("# Get component registry\n")
        content.append("registry = get_component_registry()\n")
        content.append("```\n\n")

        content.append("### Mission Execution\n\n")
        content.append("```python\n")
        content.append("from missions.waypoint_navigation_mission import WaypointNavigationMission\n\n")
        content.append("# Create mission\n")
        content.append("waypoints = [\n")
        content.append("    {'latitude': 35.0, 'longitude': -120.0},\n")
        content.append("    {'latitude': 35.01, 'longitude': -120.01}\n")
        content.append("]\n\n")
        content.append("mission = WaypointNavigationMission(waypoints)\n\n")
        content.append("# Execute mission\n")
        content.append("result = await mission.execute()\n")
        content.append("```\n\n")

        content.append("### Dashboard Usage\n\n")
        content.append("```python\n")
        content.append("import streamlit as st\n")
        content.append("from src.dashboard.interactive_dashboard import create_dashboard_app\n\n")
        content.append("# Create and run dashboard\n")
        content.append("dashboard = create_dashboard_app()\n")
        content.append("dashboard.run_dashboard()\n")
        content.append("```\n\n")

        content.append("### Configuration Management\n\n")
        content.append("```python\n")
        content.append("from src.core.configuration_manager import get_config_manager\n\n")
        content.append("# Get configuration manager\n")
        content.append("config_mgr = get_config_manager()\n\n")
        content.append("# Update navigation settings\n")
        content.append("config_mgr.update_config({\n")
        content.append("    'navigation': {\n")
        content.append("        'update_rate_hz': 15.0,\n")
        content.append("        'waypoint_tolerance_m': 1.5\n")
        content.append("    }\n")
        content.append("})\n")
        content.append("```\n\n")

        return "".join(content)

    def _write_file(self, filename: str, content: str):
        """Write content to file."""
        file_path = self.output_dir / filename
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✅ Generated {filename}")


def main():
    """Generate documentation."""
    generator = DocumentationGenerator()
    generator.generate_full_documentation()


if __name__ == "__main__":
    main()




