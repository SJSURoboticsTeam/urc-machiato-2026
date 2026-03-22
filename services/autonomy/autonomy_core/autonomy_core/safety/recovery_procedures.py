"""
Recovery procedures for safety subsystem.

Provides a small set of recovery actions invoked by SafetyMonitor recovery
callbacks or EmergencyResponseCoordinator. Used by the automatic recovery
path in SafetyWatchdog when enable_automatic_recovery is True.
"""

import logging
from typing import Any, Optional

logger = logging.getLogger(__name__)


def clear_and_recheck(watchdog: Any) -> bool:
    """
    Clear safety stop and re-check health.

    Invoked when automatic recovery determines health has been restored.
    Call watchdog.clear_safety_stop() and optionally re-validate that
    heartbeat and sensor data are current.

    Args:
        watchdog: SafetyWatchdog instance (or object with clear_safety_stop and
                 optional _health_ok_for_recovery).

    Returns:
        True if clear was performed, False if skipped (e.g. health not ok).
    """
    if watchdog is None:
        return False
    try:
        if hasattr(watchdog, "_health_ok_for_recovery") and callable(
            getattr(watchdog, "_health_ok_for_recovery")
        ):
            if not watchdog._health_ok_for_recovery():
                logger.warning(
                    "Recovery procedure clear_and_recheck: health check failed, skipping clear"
                )
                return False
        watchdog.clear_safety_stop()
        logger.info("Recovery procedure clear_and_recheck: safety stop cleared")
        return True
    except Exception as e:
        logger.error("Recovery procedure clear_and_recheck failed: %s", e)
        return False


def restart_perception(context: Optional[Any] = None) -> bool:
    """
    Request perception subsystem restart (stub or service call).

    When perception is degraded (e.g. sensor timeout), this procedure
    can be invoked to request a restart. If a perception restart service
    exists, it can be called here; otherwise logs the request.

    Args:
        context: Optional ROS2 node or context for calling a restart service.

    Returns:
        True if restart was requested/succeeded, False otherwise.
    """
    logger.info("Recovery procedure restart_perception: restart requested (stub)")
    # TODO: Call /perception/restart or equivalent service when available
    if context is not None and hasattr(context, "get_logger"):
        context.get_logger().info("Restart perception recovery procedure invoked")
    return True


def run_recovery_procedures(
    watchdog: Any,
    procedures: list[str],
    context: Optional[Any] = None,
) -> dict[str, bool]:
    """
    Run a list of named recovery procedures.

    Args:
        watchdog: SafetyWatchdog instance for clear_and_recheck.
        procedures: List of procedure names: "clear_and_recheck", "restart_perception".
        context: Optional context for procedures that need it.

    Returns:
        Dict mapping procedure name to success (True/False).
    """
    results: dict[str, bool] = {}
    for name in procedures:
        if name == "clear_and_recheck":
            results[name] = clear_and_recheck(watchdog)
        elif name == "restart_perception":
            results[name] = restart_perception(context)
        else:
            logger.warning("Unknown recovery procedure: %s", name)
            results[name] = False
    return results
