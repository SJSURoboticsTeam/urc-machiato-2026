#!/usr/bin/env python3
"""
Standardized Logging System for URC 2026

Provides consistent, structured logging across all components.
Replaces inconsistent logging practices with unified format.

Features:
- Structured JSON logging for production
- Human-readable format for development
- Performance-aware logging with timing
- Context tracking across components
- Automatic log rotation and archival

Usage:
    from infrastructure.logging import get_logger
    
    logger = get_logger("navigation")
    logger.info("Path planning completed", extra={
        'waypoints': 15,
        'distance': 245.6,
        'duration': 1.2
    })

Author: URC 2026 Logging Team
"""

import logging
import json
import time
import sys
import os
from datetime import datetime
from typing import Dict, Any, Optional
from pathlib import Path

import structlog


class URCFormatter(logging.Formatter):
    """Custom formatter for URC 2026 structured logging."""
    
    def __init__(self, use_json: bool = False):
        super().__init__()
        self.use_json = use_json
    
    def format(self, record: logging.LogRecord) -> str:
        """Format log record with URC standard structure."""
        # Create base log entry
        log_entry = {
            'timestamp': datetime.fromtimestamp(record.created).isoformat(),
            'level': record.levelname,
            'logger': record.name,
            'message': record.getMessage(),
            'module': record.module,
            'function': record.funcName,
            'line': record.lineno,
            'thread': record.thread,
            'process': record.process,
        }
        
        # Add extra fields if present
        if hasattr(record, '__dict__'):
            extra_fields = {}
            for key, value in record.__dict__.items():
                if key not in ['name', 'msg', 'args', 'levelname', 'levelno', 
                              'pathname', 'filename', 'module', 'lineno', 
                              'funcName', 'created', 'msecs', 'relativeCreated',
                              'thread', 'threadName', 'processName', 'process']:
                    extra_fields[key] = value
            
            log_entry.update(extra_fields)
        
        # Add performance timing if available
        if hasattr(record, 'start_time') and hasattr(record, 'end_time'):
            duration = record.end_time - record.start_time
            log_entry['duration'] = round(duration, 3)
        
        if self.use_json:
            return json.dumps(log_entry, default=str)
        else:
            # Human-readable format
            extra_info = []
            for key, value in log_entry.items():
                if key not in ['timestamp', 'level', 'logger', 'message']:
                    extra_info.append(f"{key}={value}")
            
            extra_str = f" [{', '.join(extra_info)}]" if extra_info else ""
            return f"{log_entry['timestamp']} {log_entry['level']:<8} {log_entry['logger']:<20} {log_entry['message']}{extra_str}"


class PerformanceLogger:
    """Logger with built-in performance tracking."""
    
    def __init__(self, logger: logging.Logger):
        self.logger = logger
    
    def log_operation(
        self, 
        operation: str, 
        level: str = "info",
        start_time: Optional[float] = None,
        end_time: Optional[float] = None,
        **kwargs
    ):
        """Log operation with performance data."""
        start_time = start_time or time.time()
        end_time = end_time or time.time()
        duration = end_time - start_time
        
        # Create log record with performance data
        extra_data = {
            'operation': operation,
            'start_time': start_time,
            'end_time': end_time,
            'duration': round(duration, 3),
            **kwargs
        }
        
        getattr(self.logger, level.lower())(
            f"Operation {operation} completed in {duration:.3f}s",
            extra=extra_data
        )
    
    def __enter__(self):
        """Context manager entry - start timing."""
        self.start_time = time.time()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - log performance."""
        end_time = time.time()
        duration = end_time - self.start_time
        
        if exc_type:
            self.logger.error(
                f"Operation failed after {duration:.3f}s",
                extra={'duration': duration, 'error': str(exc_val)},
                exc_info=True
            )
        else:
            self.logger.info(
                f"Operation completed in {duration:.3f}s",
                extra={'duration': duration}
            )


class ContextLogger:
    """Logger with automatic context tracking."""
    
    def __init__(self, logger: logging.Logger, context: Dict[str, Any]):
        self.logger = logger
        self.context = context
    
    def _log_with_context(self, level: str, message: str, **kwargs):
        """Log message with context."""
        extra_data = {**self.context, **kwargs}
        getattr(self.logger, level.lower())(message, extra=extra_data)
    
    def debug(self, message: str, **kwargs):
        self._log_with_context('debug', message, **kwargs)
    
    def info(self, message: str, **kwargs):
        self._log_with_context('info', message, **kwargs)
    
    def warning(self, message: str, **kwargs):
        self._log_with_context('warning', message, **kwargs)
    
    def error(self, message: str, **kwargs):
        self._log_with_context('error', message, **kwargs)
    
    def critical(self, message: str, **kwargs):
        self._log_with_context('critical', message, **kwargs)


# Global logging configuration
_loggers = {}
_configured = False


def _configure_logging():
    """Configure logging system with URC standards."""
    global _configured
    if _configured:
        return
    
    # Get configuration
    try:
        from infrastructure.config.environment import get_env_manager
        env = get_env_manager()
        log_level = getattr(logging, env.system.log_level.upper(), logging.INFO)
        use_json = env.is_production()
    except Exception:
        log_level = logging.INFO
        use_json = False
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)
    
    # Remove existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(URCFormatter(use_json=use_json))
    root_logger.addHandler(console_handler)
    
    # File handler for production
    if use_json:
        log_dir = Path('/var/log/urc')
        log_dir.mkdir(exist_ok=True)
        
        file_handler = logging.handlers.RotatingFileHandler(
            filename=log_dir / 'urc.log',
            maxBytes=50 * 1024 * 1024,  # 50MB
            backupCount=5
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(URCFormatter(use_json=True))
        root_logger.addHandler(file_handler)
    
    # Configure structlog
    structlog.configure(
        processors=[
            structlog.stdlib.filter_by_level,
            structlog.stdlib.add_logger_name,
            structlog.stdlib.add_log_level,
            structlog.stdlib.PositionalArgumentsFormatter(),
            structlog.processors.TimeStamper(fmt="iso"),
            structlog.processors.StackInfoRenderer(),
            structlog.processors.format_exc_info,
            structlog.processors.UnicodeDecoder(),
            structlog.processors.JSONRenderer() if use_json else structlog.dev.ConsoleRenderer(),
        ],
        context_class=dict,
        logger_factory=structlog.stdlib.LoggerFactory(),
        wrapper_class=structlog.stdlib.BoundLogger,
        cache_logger_on_first_use=True,
    )
    
    _configured = True


def get_logger(name: str, context: Optional[Dict[str, Any]] = None) -> logging.Logger:
    """Get configured logger with optional context."""
    _configure_logging()
    
    if name not in _loggers:
        logger = logging.getLogger(name)
        _loggers[name] = logger
    
    if context:
        return ContextLogger(_loggers[name], context)
    
    return _loggers[name]


def get_performance_logger(name: str) -> PerformanceLogger:
    """Get performance-aware logger."""
    logger = get_logger(name)
    return PerformanceLogger(logger)


def log_system_startup(component: str, version: str = "unknown"):
    """Log system startup information."""
    logger = get_logger("system")
    logger.info(
        f"URC 2026 {component} starting",
        extra={
            'component': component,
            'version': version,
            'timestamp': datetime.now().isoformat(),
            'python_version': sys.version,
            'pid': os.getpid()
        }
    )


def log_system_shutdown(component: str, reason: str = "normal"):
    """Log system shutdown information."""
    logger = get_logger("system")
    logger.info(
        f"URC 2026 {component} shutting down",
        extra={
            'component': component,
            'reason': reason,
            'timestamp': datetime.now().isoformat(),
            'uptime': time.time() - getattr(log_system_startup, '_start_time', time.time())
        }
    )


def log_error_with_context(
    error: Exception,
    context: Dict[str, Any],
    logger_name: str = "error"
):
    """Log error with full context information."""
    logger = get_logger(logger_name)
    logger.error(
        f"Error occurred: {error}",
        extra={
            'error_type': type(error).__name__,
            'error_message': str(error),
            'context': context,
            'timestamp': datetime.now().isoformat()
        },
        exc_info=True
    )


# Decorators for common logging patterns
def log_performance(operation_name: str):
    """Decorator to automatically log function performance."""
    def decorator(func):
        def wrapper(*args, **kwargs):
            logger = get_performance_logger(func.__module__)
            with logger:
                return func(*args, **kwargs)
        return wrapper
    return decorator


def log_errors(logger_name: str = None):
    """Decorator to automatically log function errors."""
    def decorator(func):
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                logger_name_final = logger_name or func.__module__
                logger = get_logger(logger_name_final)
                logger.error(
                    f"Function {func.__name__} failed",
                    extra={
                        'function': func.__name__,
                        'args_count': len(args),
                        'kwargs_keys': list(kwargs.keys()),
                        'error_type': type(e).__name__,
                        'error_message': str(e)
                    },
                    exc_info=True
                )
                raise
        return wrapper
    return decorator


if __name__ == "__main__":
    # Demo logging system
    _configure_logging()
    
    # Demo 1: Basic logging
    logger = get_logger("demo")
    logger.info("This is a demo log message")
    
    # Demo 2: Performance logging
    perf_logger = get_performance_logger("demo_perf")
    perf_logger.log_operation("demo_operation", duration=1.5, items_processed=100)
    
    # Demo 3: Context logging
    context_logger = get_logger("demo_context", {"rover_id": "test-001", "mission": "demo"})
    context_logger.info("Contextual log message")
    
    # Demo 4: Performance decorator
    @log_performance("demo_function")
    def demo_function():
        time.sleep(0.1)
        return "success"
    
    result = demo_function()
    print(f"Demo result: {result}")