"""Structured logging configuration for simulation framework.

Provides comprehensive logging setup with structured output,
performance monitoring, and RL training support.

Author: URC 2026 Autonomy Team
"""

import logging
import sys
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import structlog
    STRUCTLOG_AVAILABLE = True
except ImportError:
    STRUCTLOG_AVAILABLE = False
    print("Warning: structlog not available, using basic logging")


def setup_simulation_logging(
    log_level: str = "INFO",
    log_file: str = "simulation.log",
    enable_structured: bool = True,
    enable_json: bool = False
) -> None:
    """Setup comprehensive simulation logging.

    Args:
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR)
        log_file: Path to log file
        enable_structured: Enable structured logging with context
        enable_json: Output logs in JSON format
    """
    # Ensure log directory exists
    log_path = Path(log_file)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    # Configure Python logging
    numeric_level = getattr(logging, log_level.upper(), logging.INFO)

    # Create formatters
    if enable_json and STRUCTLOG_AVAILABLE:
        # JSON formatter for structured logs
        formatter = None  # structlog handles formatting
    else:
        # Human-readable formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

    # Setup handlers
    handlers = []

    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    if formatter:
        console_handler.setFormatter(formatter)
    console_handler.setLevel(numeric_level)
    handlers.append(console_handler)

    # File handler
    file_handler = logging.FileHandler(log_file)
    if formatter:
        file_handler.setFormatter(formatter)
    file_handler.setLevel(numeric_level)
    handlers.append(file_handler)

    # Configure root logger
    logging.basicConfig(
        level=numeric_level,
        handlers=handlers,
        force=True  # Override any existing configuration
    )

    # Configure structlog if available
    if STRUCTLOG_AVAILABLE and enable_structured:
        _configure_structlog(enable_json)
    else:
        # Fallback structured logging without structlog
        logging.info("Using basic logging (install structlog for enhanced structured logging)")


def _configure_structlog(enable_json: bool = False):
    """Configure structlog for structured logging."""
    import structlog

    shared_processors = [
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.processors.UnicodeDecoder(),
    ]

    if enable_json:
        # JSON output for log aggregation systems
        shared_processors.append(structlog.processors.JSONRenderer())
    else:
        # Human-readable output with colors
        try:
            import colorama
            colorama.init()
            shared_processors.append(
                structlog.dev.ConsoleRenderer(colors=True)
            )
        except ImportError:
            shared_processors.append(
                structlog.dev.ConsoleRenderer()
            )

    structlog.configure(
        processors=shared_processors,
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )


class SimulationLogger:
    """Enhanced logger for simulation components."""

    def __init__(self, name: str, component_type: str = "simulation"):
        """Initialize simulation logger.

        Args:
            name: Logger name (usually __name__)
            component_type: Type of component (simulation, sensor, network, etc.)
        """
        if STRUCTLOG_AVAILABLE:
            self.logger = structlog.get_logger(name)
        else:
            self.logger = logging.getLogger(name)

        self.component_type = component_type
        self._context = {
            "component": component_type,
            "logger_name": name
        }

    def _add_context(self, **kwargs) -> Dict[str, Any]:
        """Add context to log message."""
        context = self._context.copy()
        context.update(kwargs)
        return context

    def debug(self, message: str, **kwargs):
        """Log debug message with context."""
        self.logger.debug(message, **self._add_context(**kwargs))

    def info(self, message: str, **kwargs):
        """Log info message with context."""
        self.logger.info(message, **self._add_context(**kwargs))

    def warning(self, message: str, **kwargs):
        """Log warning message with context."""
        self.logger.warning(message, **self._add_context(**kwargs))

    def error(self, message: str, **kwargs):
        """Log error message with context."""
        self.logger.error(message, **self._add_context(**kwargs))

    def critical(self, message: str, **kwargs):
        """Log critical message with context."""
        self.logger.critical(message, **self._add_context(**kwargs))

    def log_performance(self, operation: str, duration: float, **metrics):
        """Log performance metrics."""
        self.logger.info(
            f"Performance: {operation}",
            operation=operation,
            duration_ms=duration * 1000,
            **self._add_context(**metrics)
        )

    def log_simulation_step(self, step_count: int, sim_time: float, **metrics):
        """Log simulation step information."""
        self.logger.debug(
            f"Simulation step {step_count}",
            step=step_count,
            simulation_time=sim_time,
            **self._add_context(**metrics)
        )

    def log_rl_step(self, episode: int, step: int, reward: float, **rl_metrics):
        """Log RL training step."""
        self.logger.info(
            f"RL Step: Episode {episode}, Step {step}, Reward {reward:.3f}",
            episode=episode,
            step=step,
            reward=reward,
            **self._add_context(**rl_metrics)
        )

    def bind_context(self, **context):
        """Bind additional context to this logger instance."""
        self._context.update(context)
        return self


def get_simulation_logger(name: str, component_type: str = "simulation") -> SimulationLogger:
    """Get a simulation logger instance.

    Args:
        name: Logger name
        component_type: Component type for context

    Returns:
        SimulationLogger instance
    """
    return SimulationLogger(name, component_type)
