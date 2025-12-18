#!/usr/bin/env python3
"""
Comprehensive tests for Error Handling System

Tests error boundaries, safe execution, logging, and error aggregation.
"""

import pytest
from unittest.mock import Mock, patch, call
import time

from autonomy_state_machine.error_handling import (
    handle_service_error, error_boundary, safe_execute, validate_transition_request,
    log_operation_start, log_operation_complete, create_error_summary
)
from autonomy_state_machine.error_handling import (
    StateMachineError, TransitionError, ContextError, PolicyError, MonitoringError
)
from autonomy_state_machine.states import RoverState


class TestErrorHandling:
    """Comprehensive test suite for error handling utilities."""

    @pytest.fixture
    def mock_logger(self):
        """Create a mock logger."""
        return Mock()

    def test_custom_exceptions_hierarchy(self):
        """Test custom exception hierarchy."""
        # Test base exception
        error = StateMachineError("Test error", "TestComponent", "ERR001")
        assert str(error) == "[TestComponent] Test error"
        assert error.component == "TestComponent"
        assert error.error_code == "ERR001"

        # Test specific exceptions
        transition_error = TransitionError("Invalid transition")
        assert isinstance(transition_error, StateMachineError)

        context_error = ContextError("Context evaluation failed")
        assert isinstance(context_error, StateMachineError)

        policy_error = PolicyError("Policy execution failed")
        assert isinstance(policy_error, StateMachineError)

        monitoring_error = MonitoringError("Monitoring failed")
        assert isinstance(monitoring_error, StateMachineError)

    def test_handle_service_error_basic(self, mock_logger):
        """Test basic error handling functionality."""
        test_error = ValueError("Test error")

        handle_service_error(mock_logger, test_error, "test operation", "TestComponent")

        mock_logger.error.assert_called_once()
        call_args = mock_logger.error.call_args[0][0]
        assert "[TestComponent]" in call_args
        assert "Unexpected error" in call_args
        assert "test operation" in call_args

    def test_handle_service_error_with_traceback(self, mock_logger):
        """Test error handling with traceback."""
        test_error = RuntimeError("Runtime error")

        handle_service_error(
            mock_logger, test_error, "complex operation",
            "ComplexComponent", include_traceback=True
        )

        # Should have called error and debug (for traceback)
        assert mock_logger.error.called
        assert mock_logger.debug.called

    def test_handle_service_error_custom_exception(self, mock_logger):
        """Test error handling with custom StateMachineError."""
        custom_error = TransitionError("Invalid transition", "StateMachine", "TRANS001")

        handle_service_error(mock_logger, custom_error, "state change", "StateManager")

        mock_logger.error.assert_called_once()
        call_args = mock_logger.error.call_args[0][0]
        assert "[StateManager]" in call_args
        assert "Invalid transition" in call_args

    def test_error_boundary_context_manager_success(self, mock_logger):
        """Test error boundary with successful execution."""
        execution_count = 0

        with error_boundary(mock_logger, "TestComponent", "test operation"):
            execution_count += 1
            assert execution_count == 1

        # Should not have logged any errors
        mock_logger.error.assert_not_called()

    def test_error_boundary_context_manager_failure(self, mock_logger):
        """Test error boundary with failed execution."""
        execution_count = 0

        with pytest.raises(ValueError):
            with error_boundary(mock_logger, "TestComponent", "failing operation", reraise=True):
                execution_count += 1
                raise ValueError("Test failure")

        # Should have logged the error
        mock_logger.error.assert_called_once()
        assert execution_count == 1  # Code before exception should have run

    def test_error_boundary_context_manager_no_reraise(self, mock_logger):
        """Test error boundary that doesn't re-raise exceptions."""
        execution_count = 0

        with error_boundary(mock_logger, "TestComponent", "failing operation", reraise=False):
            execution_count += 1
            raise RuntimeError("Test failure")

        # Should have logged the error but not raised
        mock_logger.error.assert_called_once()
        assert execution_count == 1

    def test_safe_execute_success(self, mock_logger):
        """Test safe_execute with successful function."""
        def successful_function():
            return "success"

        result = safe_execute(successful_function, mock_logger, "TestComponent", "test operation")

        assert result == "success"
        mock_logger.error.assert_not_called()

    def test_safe_execute_failure(self, mock_logger):
        """Test safe_execute with failing function."""
        def failing_function():
            raise ValueError("Function failed")

        result = safe_execute(
            failing_function, mock_logger, "TestComponent",
            "failing operation", default_return="default"
        )

        assert result == "default"
        mock_logger.error.assert_called_once()

    def test_safe_execute_custom_exception_type(self, mock_logger):
        """Test safe_execute with specific exception type."""
        def failing_function():
            raise KeyError("Key not found")

        # Should catch KeyError specifically
        result = safe_execute(
            failing_function, mock_logger, "TestComponent",
            "key operation", default_return="default", error_type=KeyError
        )

        assert result == "default"
        mock_logger.error.assert_called_once()

    def test_safe_execute_wrong_exception_type(self, mock_logger):
        """Test safe_execute with wrong exception type (should catch and return default)."""
        def failing_function():
            raise ValueError("Value error")

        # Should catch ValueError when expecting Exception (default)
        result = safe_execute(
            failing_function, mock_logger, "TestComponent",
            "value operation", default_return="handled", error_type=Exception
        )

        assert result == "handled"
        mock_logger.error.assert_called_once()

    def test_validate_transition_request_valid(self):
        """Test validation of valid transition requests."""
        # Should not raise
        validate_transition_request(RoverState.READY, RoverState.AUTO, "valid transition")

    def test_validate_transition_request_no_current_state(self):
        """Test validation with None current state."""
        with pytest.raises(ValueError, match="Current state cannot be None"):
            validate_transition_request(None, RoverState.AUTO, "invalid")

    def test_validate_transition_request_no_target_state(self):
        """Test validation with None target state."""
        with pytest.raises(ValueError, match="Target state cannot be None"):
            validate_transition_request(RoverState.READY, None, "invalid")

    def test_validate_transition_request_empty_reason(self):
        """Test validation with empty reason."""
        with pytest.raises(ValueError, match="Transition reason cannot be empty"):
            validate_transition_request(RoverState.READY, RoverState.AUTO, "")

    def test_validate_transition_request_whitespace_reason(self):
        """Test validation with whitespace-only reason."""
        with pytest.raises(ValueError, match="Transition reason cannot be empty"):
            validate_transition_request(RoverState.READY, RoverState.AUTO, "   ")

    def test_log_operation_start(self, mock_logger):
        """Test operation start logging."""
        log_operation_start(mock_logger, "test_operation", "TestComponent",
                          param1="value1", param2=42)

        mock_logger.info.assert_called_once()
        call_args = mock_logger.info.call_args[0][0]
        assert "[TestComponent]" in call_args
        assert "Starting test_operation" in call_args
        assert "param1=value1" in call_args
        assert "param2=42" in call_args

    def test_log_operation_complete(self, mock_logger):
        """Test operation completion logging."""
        log_operation_complete(mock_logger, "test_operation", "TestComponent",
                             duration=1.5, result="success", items_processed=100)

        mock_logger.info.assert_called_once()
        call_args = mock_logger.info.call_args[0][0]
        assert "[TestComponent]" in call_args
        assert "Completed test_operation" in call_args
        assert ".500s" in call_args or "1.500s" in call_args
        assert "result=success" in call_args
        assert "items_processed=100" in call_args

    def test_log_operation_complete_no_duration(self, mock_logger):
        """Test operation completion logging without duration."""
        log_operation_complete(mock_logger, "test_operation", "TestComponent",
                             result="success")

        mock_logger.info.assert_called_once()
        call_args = mock_logger.info.call_args[0][0]
        assert "1.50s" not in call_args  # No duration in output

    def test_create_error_summary_empty(self):
        """Test error summary creation with empty error list."""
        summary = create_error_summary([])

        assert summary["total_errors"] == 0
        assert len(summary["error_types"]) == 0
        assert len(summary["components"]) == 0
        assert len(summary["timestamps"]) == 0

    def test_create_error_summary_with_errors(self):
        """Test error summary creation with actual errors."""
        mock_errors = [
            {
                "type": "ValueError",
                "component": "TestComponent",
                "timestamp": time.time()
            },
            {
                "type": "RuntimeError",
                "component": "TestComponent",
                "timestamp": time.time()
            },
            {
                "type": "ValueError",
                "component": "OtherComponent",
                "timestamp": time.time()
            }
        ]

        summary = create_error_summary(mock_errors)

        assert summary["total_errors"] == 3
        assert summary["error_types"]["ValueError"] == 2
        assert summary["error_types"]["RuntimeError"] == 1
        assert summary["components"]["TestComponent"] == 2
        assert summary["components"]["OtherComponent"] == 1
        assert len(summary["timestamps"]) == 3

    def test_create_error_summary_with_exception_objects(self):
        """Test error summary creation with exception objects."""
        error1 = ValueError("Test error 1")
        error1.component = "TestComponent"  # Mock component attribute

        error2 = RuntimeError("Test error 2")
        error2.component = "TestComponent"

        summary = create_error_summary([error1, error2])

        assert summary["total_errors"] == 2
        assert summary["error_types"]["ValueError"] == 1
        assert summary["error_types"]["RuntimeError"] == 1
        assert summary["components"]["TestComponent"] == 2

    def test_error_boundary_nested_contexts(self, mock_logger):
        """Test nested error boundary contexts."""
        inner_executed = False
        outer_executed = False

        with error_boundary(mock_logger, "OuterComponent", "outer operation", reraise=False):
            outer_executed = True

            with error_boundary(mock_logger, "InnerComponent", "inner operation", reraise=False):
                inner_executed = True
                raise ValueError("Inner failure")

            # This should execute since inner exception was not re-raised
            assert True, "Should reach here since inner exception was handled"

        # Both should have executed
        assert outer_executed == True
        assert inner_executed == True

        # Should have logged the inner error
        mock_logger.error.assert_called_once()

    @patch('time.time')
    def test_error_boundary_with_timing(self, mock_time, mock_logger):
        """Test error boundary timing and context."""
        mock_time.return_value = 1000.0

        with pytest.raises(ConnectionError):
            with error_boundary(mock_logger, "TimedComponent", "timed operation", reraise=True):
                time.sleep(0.1)  # Simulate work
                raise ConnectionError("Connection failed")

        # Verify error was logged with proper context
        mock_logger.error.assert_called_once()
        call_args = mock_logger.error.call_args[0][0]
        assert "[TimedComponent]" in call_args
        assert "timed operation" in call_args

    def test_safe_execute_with_complex_function(self, mock_logger):
        """Test safe_execute with complex function and side effects."""
        side_effects = []

        def complex_function():
            side_effects.append("started")
            result = 42 / 2  # This works
            side_effects.append("calculated")
            return result

        result = safe_execute(complex_function, mock_logger, "ComplexComponent", "complex calc")

        assert result == 21.0
        assert side_effects == ["started", "calculated"]
        mock_logger.error.assert_not_called()

    def test_safe_execute_function_exception_in_cleanup(self, mock_logger):
        """Test safe_execute when function fails during cleanup."""
        def failing_function():
            class CustomException(Exception):
                def __str__(self):
                    raise RuntimeError("Cleanup failed")
            raise CustomException("Original failure")

        result = safe_execute(
            failing_function, mock_logger, "CleanupComponent",
            "failing operation", default_return="recovered"
        )

        # Should still return default value even if error handling has issues
        assert result == "recovered"
        mock_logger.error.assert_called()  # Should have logged something
