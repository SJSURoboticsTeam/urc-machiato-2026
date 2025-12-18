#!/usr/bin/env python3
"""
WebSocket Manager - URC Competition WebSocket Connection Management

Handles WebSocket server setup, client connections, message routing,
and command processing for the URC competition bridge.
"""

import asyncio
import json
import threading
import time
from typing import Any, Awaitable, Callable, Dict, Optional, Set

try:
    import websockets
    from websockets.server import WebSocketServerProtocol

    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False
    WebSocketServerProtocol = Any

from .constants import (
    DEFAULT_WEBSOCKET_PORT,
    WEBSOCKET_CLOSE_TIMEOUT,
    WEBSOCKET_PING_INTERVAL,
)


class WebSocketManager:
    """
    Manages WebSocket connections and messaging for competition telemetry.

    Handles:
    - WebSocket server lifecycle
    - Client connection management
    - Message routing and command processing
    - Connection health monitoring
    """

    def __init__(
        self,
        logger,
        telemetry_data: Dict[str, Any],
        port: int = 8080,
        max_clients: int = 10,
        redundancy_manager=None
    ):
        """
        Initialize the WebSocket Manager.

        Args:
            logger: Logger instance for WebSocket operations
            telemetry_data: Reference to telemetry data dictionary
            port: WebSocket server port
            max_clients: Maximum number of concurrent clients
            redundancy_manager: Optional redundancy manager for failover
        """
        self.logger = logger
        self.telemetry_data = telemetry_data
        self.port = port
        self.max_clients = max_clients
        self.redundancy_manager = redundancy_manager

        # WebSocket server components
        self.websocket_server: Optional[Any] = None
        self.websocket_clients: Set[Any] = set()
        self.websocket_thread: Optional[threading.Thread] = None
        self.server_running = False

        # Configuration
        self.port = DEFAULT_WEBSOCKET_PORT
        self.endpoint_name = "competition_bridge"

        # Command handlers
        self.command_handlers: Dict[str, Callable] = {}

    def register_command_handler(self, command: str, handler: Callable) -> None:
        """
        Register a command handler.

        Args:
            command: Command name to handle
            handler: Function to call for this command
        """
        self.command_handlers[command] = handler

    def set_port(self, port: int) -> None:
        """Set the WebSocket server port."""
        self.port = port

    def start_server(self) -> bool:
        """
        Start the WebSocket server.

        Returns:
            True if server started successfully, False otherwise
        """
        if not WEBSOCKET_AVAILABLE:
            self.logger.error(
                "websockets library not available - WebSocket server disabled"
            )
            return False

        try:
            self.websocket_thread = threading.Thread(target=self._run_async_server)
            self.websocket_thread.daemon = True
            self.websocket_thread.start()
            self.logger.info(f"WebSocket server thread started on port {self.port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to start WebSocket server: {e}")
            return False

    def _run_async_server(self) -> None:
        """Run the async WebSocket server in a new event loop."""
        try:
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            # Run the server
            loop.run_until_complete(self.websocket_server_loop())

        except Exception as e:
            self.logger.error(f"WebSocket server thread error: {e}")
        finally:
            try:
                loop.close()
            except:
                pass

    def stop_server(self) -> None:
        """Stop the WebSocket server."""
        self.server_running = False
        if self.websocket_server:
            try:
                # Note: In a real implementation, we'd properly close the server
                self.logger.info("WebSocket server stopped")
            except Exception as e:
                self.logger.error(f"Error stopping WebSocket server: {e}")

    async def websocket_server_loop(self) -> None:
        """WebSocket server main loop."""
        try:
            self.server_running = True

            async def handler(websocket: WebSocketServerProtocol, path: str) -> None:
                await self._handle_client_connection(websocket, path)

            server = await websockets.serve(
                handler,
                "0.0.0.0",
                self.port,
                max_size=1048576,  # 1MB max message size
                max_queue=128,  # Increased queue for high throughput
                ping_interval=WEBSOCKET_PING_INTERVAL,  # Optimized ping frequency
                close_timeout=WEBSOCKET_CLOSE_TIMEOUT,  # Faster connection cleanup
                compression=None,  # Disable compression for speed
            )

            self.websocket_server = server
            self.logger.info(f"WebSocket server started on ws://0.0.0.0:{self.port}")
            await server.wait_closed()

        except Exception as e:
            self.logger.error(f"WebSocket server loop error: {e}")
        finally:
            self.server_running = False

    async def _handle_client_connection(
        self, websocket: WebSocketServerProtocol, path: str
    ) -> None:
        """
        Handle a new WebSocket client connection.

        Args:
            websocket: WebSocket connection
            path: Connection path
        """
        client_id = f"competition_{int(time.time() * 1000)}_{hash(websocket)}"

        # Handle through redundancy manager if enabled
        if self.redundancy_manager:
            await self.redundancy_manager.handle_client_connection(
                websocket, self.endpoint_name, client_id
            )
        else:
            # Legacy handling without redundancy
            self.websocket_clients.add(websocket)
            self.logger.info(
                f"WebSocket client connected ({len(self.websocket_clients)} total)"
            )

            try:
                # Send initial telemetry
                await self._send_message(
                    websocket, self.telemetry_data, "initial telemetry"
                )

                # Handle incoming messages
                async for message in websocket:
                    await self._handle_websocket_message(websocket, message)

            except websockets.exceptions.ConnectionClosed:
                self.logger.info(f"WebSocket client {client_id} disconnected")
            except Exception as e:
                self.logger.error(f"WebSocket client {client_id} error: {e}")
            finally:
                self.websocket_clients.discard(websocket)

    async def _handle_websocket_message(
        self, websocket: WebSocketServerProtocol, message: str
    ) -> None:
        """
        Handle incoming WebSocket message.

        Args:
            websocket: WebSocket connection
            message: Received message
        """
        try:
            data = json.loads(message)
            command_type = data.get("type", "")

            # Route to appropriate handler
            if command_type in self.command_handlers:
                await self.command_handlers[command_type](websocket, data)
            else:
                await self._handle_standard_command(websocket, command_type, data)

        except json.JSONDecodeError:
            self.logger.warning("Received invalid JSON message")
        except Exception as e:
            self.logger.error(f"WebSocket message handling error: {e}")

    async def _handle_standard_command(
        self,
        websocket: WebSocketServerProtocol,
        command_type: str,
        data: Dict[str, Any],
    ) -> None:
        """
        Handle standard WebSocket commands.

        Args:
            websocket: WebSocket connection
            command_type: Type of command
            data: Command data
        """
        try:
            if command_type == "start_mission":
                mission_type = data.get("mission", "autonomous")
                if "start_mission" in self.command_handlers:
                    await self.command_handlers["start_mission"](mission_type)

            elif command_type == "stop_mission":
                if "stop_mission" in self.command_handlers:
                    await self.command_handlers["stop_mission"]()

            elif command_type == "emergency_stop":
                if "emergency_stop" in self.command_handlers:
                    await self.command_handlers["emergency_stop"]()

            elif command_type == "teleoperation_enable":
                if "teleoperation_enable" in self.command_handlers:
                    await self.command_handlers["teleoperation_enable"]()

            elif command_type == "teleoperation_disable":
                if "teleoperation_disable" in self.command_handlers:
                    await self.command_handlers["teleoperation_disable"]()

            elif command_type == "request_telemetry":
                # Send current telemetry
                await self._send_message(
                    websocket, self.telemetry_data, "current telemetry"
                )

            elif command_type == "system_command":
                # Handle system-level commands
                cmd = data.get("command", "")
                if "system_command" in self.command_handlers:
                    await self.command_handlers["system_command"](cmd, data)

            elif command_type == "start_autonomous_navigation":
                # Start autonomous navigation mission
                targets = data.get("targets", {})
                if "start_autonomous_navigation" in self.command_handlers:
                    await self.command_handlers["start_autonomous_navigation"](targets)
                self.logger.info("Autonomous navigation started via WebSocket")

            elif command_type == "target_reached":
                # Mark target as reached
                target_type = data.get("target_type", "")
                target_index = data.get("target_index", 0)
                if "target_reached" in self.command_handlers:
                    await self.command_handlers["target_reached"](
                        target_type, target_index
                    )

            elif command_type == "abort_to_previous":
                # Abort and return to previous target
                if "abort_to_previous" in self.command_handlers:
                    previous_target = await self.command_handlers["abort_to_previous"]()
                    if previous_target:
                        await self._send_message(
                            websocket,
                            {
                                "type": "abort_acknowledged",
                                "previous_target": previous_target,
                            },
                            "abort acknowledgment",
                        )

            elif command_type == "highlight_object":
                # Highlight object on C2 display
                object_name = data.get("object_name", "")
                confidence = data.get("confidence", 0.0)
                bounding_box = data.get("bounding_box", [])
                if "highlight_object" in self.command_handlers:
                    await self.command_handlers["highlight_object"](
                        object_name, confidence, bounding_box
                    )

            elif command_type == "equipment_servicing_command":
                # Handle equipment servicing commands
                subcommand = data.get("subcommand", "")
                if subcommand == "complete_task":
                    task_name = data.get("task_name", "")
                    if "complete_equipment_task" in self.command_handlers:
                        await self.command_handlers["complete_equipment_task"](
                            task_name
                        )
                elif subcommand == "set_launch_key":
                    launch_key = data.get("launch_key", "")
                    if "set_launch_key" in self.command_handlers:
                        await self.command_handlers["set_launch_key"](launch_key)

            elif command_type == "get_compliance_status":
                # Send compliance status to judges
                if "get_compliance_status" in self.command_handlers:
                    status = await self.command_handlers["get_compliance_status"]()
                    await self._send_message(
                        websocket,
                        {"type": "compliance_status", "data": status},
                        "compliance status",
                    )

        except Exception as e:
            self.logger.error(f"Error handling command {command_type}: {e}")

    async def _send_message(
        self,
        websocket: WebSocketServerProtocol,
        message: Dict[str, Any],
        description: str = "",
    ) -> None:
        """
        Send a message via WebSocket with error handling.

        Args:
            websocket: WebSocket connection to send to
            message: Dictionary message to send
            description: Optional description for logging
        """
        try:
            await websocket.send(json.dumps(message))
            if description:
                self.logger.debug(f"Sent WebSocket message: {description}")
        except Exception as e:
            self.logger.warning(
                f"Failed to send WebSocket message ({description}): {e}"
            )

    def broadcast_telemetry(self, telemetry_data: Dict[str, Any]) -> None:
        """
        Broadcast telemetry data to all connected clients.

        Args:
            telemetry_data: Telemetry data to broadcast
        """
        if self.redundancy_manager:
            # Use redundancy manager for broadcasting
            self.redundancy_manager.broadcast_to_clients(telemetry_data)
        else:
            # Legacy broadcasting without redundancy
            if self.websocket_clients:
                # In a real implementation, we'd create tasks for async broadcasting
                self.logger.debug(
                    f"Would broadcast telemetry to {len(self.websocket_clients)} clients"
                )

    def get_connection_status(self) -> Dict[str, Any]:
        """Get current WebSocket connection status."""
        return {
            "server_running": self.server_running,
            "port": self.port,
            "clients_connected": len(self.websocket_clients)
            if not self.redundancy_manager
            else 0,
            "redundancy_enabled": self.redundancy_manager is not None,
        }
