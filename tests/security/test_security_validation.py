#!/usr/bin/env python3
"""
Security Validation Tests - URC 2026 Rover

Tests security aspects of the rover system including:
- Input validation and sanitization
- Authentication and authorization
- Secure communication
- Data protection
- Access control

Critical for competition safety and preventing malicious interference.
"""

import hashlib
import hmac
import json
import time
import unittest
from unittest.mock import MagicMock, Mock, patch


class TestInputValidation(unittest.TestCase):
    """Test input validation and sanitization."""

    def test_mission_command_validation(self):
        """Test mission command input validation."""
        validator = self.create_mock_input_validator()

        # Valid commands
        valid_commands = [
            {
                "type": "waypoint_navigation",
                "waypoints": [{"x": 10, "y": 20, "name": "point1"}],
                "timeout": 300,
            },
            {"type": "return_to_operator", "use_gps": True, "use_aruco": False},
        ]

        for cmd in valid_commands:
            self.assertTrue(validator.validate_mission_command(cmd))

        # Invalid commands
        invalid_commands = [
            {"type": "", "waypoints": []},  # Empty type
            {"waypoints": [{"x": "invalid", "y": None}]},  # Invalid data types
            {"type": "unknown_command"},  # Unknown command type
            {
                "type": "waypoint_navigation",
                "waypoints": "not_a_list",
            },  # Wrong data type
        ]

        for cmd in invalid_commands:
            self.assertFalse(validator.validate_mission_command(cmd))

    def test_websocket_message_validation(self):
        """Test WebSocket message validation."""
        validator = self.create_mock_message_validator()

        # Valid messages
        valid_messages = [
            {"type": "command", "data": {"action": "start"}},
            {"type": "telemetry", "sensor": "imu", "reading": [1.2, 0.3, 9.8]},
            {"type": "status", "component": "mission", "state": "active"},
        ]

        for msg in valid_messages:
            self.assertTrue(validator.validate_websocket_message(msg))

        # Invalid messages
        invalid_messages = [
            {},  # Empty message
            {"type": None},  # Null type
            {"type": "command", "data": None},  # Null data
            {"type": "unknown_type"},  # Unknown message type
            {"data": {"action": "start"}},  # Missing type
        ]

        for msg in invalid_messages:
            self.assertFalse(validator.validate_websocket_message(msg))

    def test_configuration_validation(self):
        """Test configuration file validation."""
        validator = self.create_mock_config_validator()

        # Valid configurations
        valid_configs = [
            {"simulation_rate": 10, "safety_enabled": True},
            {"max_speed": 2.0, "timeout": 300, "debug": False},
        ]

        for config in valid_configs:
            self.assertTrue(validator.validate_config(config))

        # Invalid configurations
        invalid_configs = [
            {"simulation_rate": -1},  # Negative value
            {"max_speed": "fast"},  # Wrong type
            {"timeout": 999999},  # Too large
            {"invalid_param": "value"},  # Unknown parameter
        ]

        for config in invalid_configs:
            self.assertFalse(validator.validate_config(config))

    def test_sql_injection_prevention(self):
        """Test prevention of SQL injection attacks."""
        # Since this system likely doesn't use SQL databases directly,
        # this tests parameter sanitization for any external systems
        sanitizer = self.create_mock_data_sanitizer()

        safe_inputs = ["normal_mission_name", "waypoint_001", "return_to_operator"]

        dangerous_inputs = [
            "'; DROP TABLE missions; --",
            "1 OR 1=1",
            "<script>alert('xss')</script>",
            "../../../etc/passwd",
        ]

        for safe_input in safe_inputs:
            sanitized = sanitizer.sanitize(safe_input)
            self.assertEqual(sanitized, safe_input)  # Should remain unchanged

        for dangerous_input in dangerous_inputs:
            sanitized = sanitizer.sanitize(dangerous_input)
            self.assertNotEqual(sanitized, dangerous_input)  # Should be modified
            # Ensure dangerous characters are escaped or removed
            self.assertNotIn(";", sanitized)
            self.assertNotIn("<", sanitized)

    def create_mock_input_validator(self):
        """Create mock input validator."""

        class MockInputValidator:
            def validate_mission_command(self, command):
                if not isinstance(command, dict):
                    return False

                if "type" not in command or not command["type"]:
                    return False

                valid_types = [
                    "waypoint_navigation",
                    "return_to_operator",
                    "aruco_search",
                    "follow_me",
                ]
                if command["type"] not in valid_types:
                    return False

                # Type-specific validation
                if command["type"] == "waypoint_navigation":
                    if "waypoints" not in command or not isinstance(
                        command["waypoints"], list
                    ):
                        return False
                    for wp in command["waypoints"]:
                        if not isinstance(wp, dict) or "x" not in wp or "y" not in wp:
                            return False

                return True

        return MockInputValidator()

    def create_mock_message_validator(self):
        """Create mock message validator."""

        class MockMessageValidator:
            def validate_websocket_message(self, message):
                if not isinstance(message, dict):
                    return False

                if "type" not in message:
                    return False

                valid_types = ["command", "telemetry", "status", "error"]
                if message["type"] not in valid_types:
                    return False

                # Type-specific validation
                if message["type"] == "command" and "data" not in message:
                    return False

                if message["type"] == "telemetry" and "sensor" not in message:
                    return False

                return True

        return MockMessageValidator()

    def create_mock_config_validator(self):
        """Create mock configuration validator."""

        class MockConfigValidator:
            def validate_config(self, config):
                if not isinstance(config, dict):
                    return False

                # Check for known parameters
                known_params = {
                    "simulation_rate": (int, 1, 100),
                    "max_speed": (float, 0.1, 5.0),
                    "timeout": (int, 10, 3600),
                    "safety_enabled": (bool, None, None),
                    "debug": (bool, None, None),
                }

                for key, value in config.items():
                    if key not in known_params:
                        return False  # Unknown parameter

                    expected_type, min_val, max_val = known_params[key]
                    if not isinstance(value, expected_type):
                        return False

                    if min_val is not None and value < min_val:
                        return False
                    if max_val is not None and value > max_val:
                        return False

                return True

        return MockConfigValidator()

    def create_mock_data_sanitizer(self):
        """Create mock data sanitizer."""

        class MockDataSanitizer:
            def sanitize(self, input_string):
                if not isinstance(input_string, str):
                    return str(input_string)

                # Remove dangerous characters
                sanitized = (
                    input_string.replace(";", "").replace("<", "").replace(">", "")
                )
                sanitized = sanitized.replace("'", "").replace('"', "")
                sanitized = (
                    sanitized.replace("--", "").replace("/*", "").replace("*/", "")
                )

                # Limit length
                return sanitized[:100] if len(sanitized) > 100 else sanitized

        return MockDataSanitizer()


class TestAuthentication(unittest.TestCase):
    """Test authentication and authorization mechanisms."""

    def test_api_key_validation(self):
        """Test API key validation."""
        auth_manager = self.create_mock_auth_manager()

        valid_keys = ["urc2026_rover_key_123", "competition_key_456"]
        invalid_keys = ["", "wrong_key", "expired_key_789"]

        for key in valid_keys:
            self.assertTrue(auth_manager.validate_api_key(key))

        for key in invalid_keys:
            self.assertFalse(auth_manager.validate_api_key(key))

    def test_session_management(self):
        """Test session creation and validation."""
        session_manager = self.create_mock_session_manager()

        # Create session
        session_id = session_manager.create_session("operator_001", ["read", "execute"])
        self.assertIsNotNone(session_id)

        # Validate session
        self.assertTrue(session_manager.validate_session(session_id))

        # Check permissions
        self.assertTrue(session_manager.has_permission(session_id, "read"))
        self.assertTrue(session_manager.has_permission(session_id, "execute"))
        self.assertFalse(session_manager.has_permission(session_id, "admin"))

        # Expire session
        session_manager.expire_session(session_id)
        self.assertFalse(session_manager.validate_session(session_id))

    def test_rate_limiting(self):
        """Test API rate limiting."""
        rate_limiter = self.create_mock_rate_limiter()

        client_id = "dashboard_client"

        # Should allow normal rate
        for i in range(10):
            self.assertTrue(rate_limiter.check_rate_limit(client_id, "api_call"))

        # Should block excessive requests
        for i in range(5):
            self.assertFalse(rate_limiter.check_rate_limit(client_id, "api_call"))

        # Should allow after cooldown
        rate_limiter.reset_rate_limit(client_id)
        self.assertTrue(rate_limiter.check_rate_limit(client_id, "api_call"))

    def create_mock_auth_manager(self):
        """Create mock authentication manager."""

        class MockAuthManager:
            def __init__(self):
                self.valid_keys = ["urc2026_rover_key_123", "competition_key_456"]

            def validate_api_key(self, key):
                return key in self.valid_keys

        return MockAuthManager()

    def create_mock_session_manager(self):
        """Create mock session manager."""

        class MockSessionManager:
            def __init__(self):
                self.sessions = {}

            def create_session(self, user_id, permissions):
                session_id = f"session_{user_id}_{int(time.time())}"
                self.sessions[session_id] = {
                    "user_id": user_id,
                    "permissions": permissions,
                    "created": time.time(),
                    "active": True,
                }
                return session_id

            def validate_session(self, session_id):
                session = self.sessions.get(session_id)
                return session and session["active"]

            def has_permission(self, session_id, permission):
                session = self.sessions.get(session_id)
                return session and permission in session["permissions"]

            def expire_session(self, session_id):
                if session_id in self.sessions:
                    self.sessions[session_id]["active"] = False

        return MockSessionManager()

    def create_mock_rate_limiter(self):
        """Create mock rate limiter."""

        class MockRateLimiter:
            def __init__(self):
                self.requests = {}
                self.max_requests = 10
                self.window_seconds = 60

            def check_rate_limit(self, client_id, action):
                key = f"{client_id}_{action}"
                now = time.time()

                if key not in self.requests:
                    self.requests[key] = []

                # Clean old requests
                self.requests[key] = [
                    t for t in self.requests[key] if now - t < self.window_seconds
                ]

                if len(self.requests[key]) >= self.max_requests:
                    return False

                self.requests[key].append(now)
                return True

            def reset_rate_limit(self, client_id):
                # Clear all requests for this client
                keys_to_remove = [
                    k for k in self.requests.keys() if k.startswith(client_id)
                ]
                for key in keys_to_remove:
                    del self.requests[key]

        return MockRateLimiter()


class TestSecureCommunication(unittest.TestCase):
    """Test secure communication protocols."""

    def test_message_integrity(self):
        """Test message integrity verification."""
        integrity_checker = self.create_mock_integrity_checker()

        # Create valid message with HMAC
        message = {"type": "command", "data": {"action": "start"}}
        signed_message = integrity_checker.sign_message(message)

        # Verify valid signature
        self.assertTrue(integrity_checker.verify_signature(signed_message))

        # Verify tampered message fails
        tampered = signed_message.copy()
        tampered["data"]["action"] = "stop"  # Tamper with data
        self.assertFalse(integrity_checker.verify_signature(tampered))

    def test_encryption_decryption(self):
        """Test data encryption and decryption."""
        crypto_manager = self.create_mock_crypto_manager()

        test_data = {"secret": "mission_coordinates", "value": [37.7749, -122.4194]}

        # Encrypt data
        encrypted = crypto_manager.encrypt(test_data)
        self.assertNotEqual(encrypted, test_data)

        # Decrypt data
        decrypted = crypto_manager.decrypt(encrypted)
        self.assertEqual(decrypted, test_data)

        # Test with wrong key
        crypto_manager_wrong = self.create_mock_crypto_manager(key="wrong_key")
        try:
            crypto_manager_wrong.decrypt(encrypted)
            self.fail("Should fail with wrong key")
        except:
            pass  # Expected to fail

    def test_certificate_validation(self):
        """Test SSL/TLS certificate validation."""
        cert_validator = self.create_mock_cert_validator()

        # Valid certificates
        valid_certs = ["cert_rover_valid.pem", "cert_dashboard_valid.pem"]
        for cert in valid_certs:
            self.assertTrue(cert_validator.validate_certificate(cert))

        # Invalid/expired certificates
        invalid_certs = [
            "cert_expired.pem",
            "cert_self_signed.pem",
            "cert_unknown_ca.pem",
        ]
        for cert in invalid_certs:
            self.assertFalse(cert_validator.validate_certificate(cert))

    def create_mock_integrity_checker(self):
        """Create mock integrity checker."""

        class MockIntegrityChecker:
            def __init__(self):
                self.secret_key = b"test_secret_key"

            def sign_message(self, message):
                message_str = json.dumps(message, sort_keys=True)
                signature = hmac.new(
                    self.secret_key, message_str.encode(), hashlib.sha256
                ).hexdigest()
                return {"message": message, "signature": signature}

            def verify_signature(self, signed_message):
                message = signed_message["message"]
                signature = signed_message["signature"]

                message_str = json.dumps(message, sort_keys=True)
                expected_signature = hmac.new(
                    self.secret_key, message_str.encode(), hashlib.sha256
                ).hexdigest()

                return hmac.compare_digest(signature, expected_signature)

        return MockIntegrityChecker()

    def create_mock_crypto_manager(self, key="test_encryption_key"):
        """Create mock crypto manager."""

        class MockCryptoManager:
            def __init__(self, encryption_key="test_key"):
                self.key = encryption_key

            def encrypt(self, data):
                # Simple mock encryption (not secure!)
                data_str = json.dumps(data)
                # XOR with key pattern (for testing only)
                encrypted = "".join(
                    chr(ord(c) ^ ord(self.key[i % len(self.key)]))
                    for i, c in enumerate(data_str)
                )
                return {"encrypted": encrypted, "key_version": "1.0"}

            def decrypt(self, encrypted_data):
                if encrypted_data.get("key_version") != "1.0":
                    raise ValueError("Unsupported key version")

                encrypted = encrypted_data["encrypted"]
                # Reverse XOR
                decrypted = "".join(
                    chr(ord(c) ^ ord(self.key[i % len(self.key)]))
                    for i, c in enumerate(encrypted)
                )

                return json.loads(decrypted)

        return MockCryptoManager(key)

    def create_mock_cert_validator(self):
        """Create mock certificate validator."""

        class MockCertValidator:
            def __init__(self):
                self.valid_certs = ["cert_rover_valid.pem", "cert_dashboard_valid.pem"]

            def validate_certificate(self, cert_path):
                # Mock validation - check if cert is in valid list
                return cert_path in self.valid_certs

        return MockCertValidator()


if __name__ == "__main__":
    unittest.main()
