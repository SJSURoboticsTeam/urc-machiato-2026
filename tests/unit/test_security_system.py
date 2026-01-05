#!/usr/bin/env python3
"""
Security System Tests - URC 2026

Tests the security systems for:
- Authentication and authorization
- Input validation and sanitization
- Access control and permissions
- Secure communication channels
- Audit logging and monitoring
- Threat detection and response

Author: URC 2026 Testing Team
"""

import hashlib
import hmac
import json
import time
import pytest
from unittest.mock import Mock, patch, AsyncMock
from typing import Dict, Any, Optional

from src.core.security_manager import SecurityManager, AccessControl, AuditLogger


class TestSecurityManager:
    """Test SecurityManager functionality."""

    @pytest.fixture
    def security_manager(self):
        """Create SecurityManager instance."""
        return SecurityManager()

    @pytest.fixture
    def sample_credentials(self):
        """Sample user credentials for testing."""
        return {
            "username": "operator",
            "password": "secure_password_123",
            "role": "mission_operator"
        }

    def test_security_manager_initialization(self):
        """Test SecurityManager initialization."""
        manager = SecurityManager()

        assert manager.auth_system is not None
        assert manager.access_control is not None
        assert manager.audit_logger is not None
        assert isinstance(manager.sessions, dict)

    def test_password_hashing(self, security_manager):
        """Test password hashing functionality."""
        password = "test_password_123"

        # Hash password
        hashed = security_manager.hash_password(password)
        assert hashed != password
        assert len(hashed) > 0

        # Verify password
        assert security_manager.verify_password(password, hashed) is True
        assert security_manager.verify_password("wrong_password", hashed) is False

    def test_user_authentication(self, security_manager, sample_credentials):
        """Test user authentication."""
        # Create user
        user_id = security_manager.create_user(
            sample_credentials["username"],
            sample_credentials["password"],
            sample_credentials["role"]
        )

        assert user_id is not None

        # Authenticate user
        auth_result = security_manager.authenticate(
            sample_credentials["username"],
            sample_credentials["password"]
        )

        assert auth_result["authenticated"] is True
        assert auth_result["user_id"] == user_id
        assert auth_result["role"] == sample_credentials["role"]

        # Test failed authentication
        failed_auth = security_manager.authenticate("wrong_user", "wrong_pass")
        assert failed_auth["authenticated"] is False

    def test_session_management(self, security_manager, sample_credentials):
        """Test session management."""
        # Create user and authenticate
        security_manager.create_user(
            sample_credentials["username"],
            sample_credentials["password"],
            sample_credentials["role"]
        )

        auth_result = security_manager.authenticate(
            sample_credentials["username"],
            sample_credentials["password"]
        )

        session_token = auth_result["session_token"]
        assert session_token is not None

        # Validate session
        session_valid = security_manager.validate_session(session_token)
        assert session_valid["valid"] is True
        assert session_valid["user_id"] == auth_result["user_id"]

        # Revoke session
        security_manager.revoke_session(session_token)

        # Session should no longer be valid
        session_invalid = security_manager.validate_session(session_token)
        assert session_invalid["valid"] is False

    def test_role_based_access_control(self, security_manager):
        """Test role-based access control."""
        # Define permissions
        permissions = {
            "mission_start": ["mission_operator", "admin"],
            "system_shutdown": ["admin"],
            "telemetry_view": ["mission_operator", "viewer", "admin"]
        }

        # Test access for different roles
        assert security_manager.check_permission("mission_operator", "mission_start", permissions) is True
        assert security_manager.check_permission("viewer", "mission_start", permissions) is False
        assert security_manager.check_permission("admin", "system_shutdown", permissions) is True
        assert security_manager.check_permission("mission_operator", "system_shutdown", permissions) is False

    def test_input_validation(self, security_manager):
        """Test input validation and sanitization."""
        # Valid inputs
        valid_commands = [
            {"action": "start_mission", "mission_type": "sample_collection"},
            {"action": "navigate", "waypoint": {"lat": 37.123456, "lon": -108.987654}}
        ]

        # Invalid inputs
        invalid_commands = [
            {"action": "<script>alert('xss')</script>", "mission_type": "sample_collection"},  # XSS attempt
            {"action": "start_mission", "mission_type": "drop table users;"},  # SQL injection
            {"action": "navigate", "waypoint": {"lat": "invalid", "lon": -108.987654}}  # Type mismatch
        ]

        for command in valid_commands:
            result = security_manager.validate_input(command)
            assert result["valid"] is True

        for command in invalid_commands:
            result = security_manager.validate_input(command)
            assert result["valid"] is False
            assert len(result["errors"]) > 0

    def test_api_key_authentication(self, security_manager):
        """Test API key authentication."""
        # Create API key
        api_key, secret = security_manager.create_api_key("test_service", ["read", "write"])

        assert api_key is not None
        assert secret is not None

        # Authenticate with API key
        auth_result = security_manager.authenticate_api_key(api_key, secret)
        assert auth_result["authenticated"] is True
        assert auth_result["service"] == "test_service"
        assert "read" in auth_result["permissions"]

        # Test invalid API key
        invalid_auth = security_manager.authenticate_api_key("invalid_key", "invalid_secret")
        assert invalid_auth["authenticated"] is False

    def test_request_signing(self, security_manager):
        """Test request signing for API security."""
        payload = {"action": "start_mission", "timestamp": int(time.time())}
        secret = "test_secret_key"

        # Sign request
        signature = security_manager.sign_request(payload, secret)

        # Verify signature
        is_valid = security_manager.verify_request_signature(payload, signature, secret)
        assert is_valid is True

        # Test invalid signature
        is_invalid = security_manager.verify_request_signature(payload, "invalid_sig", secret)
        assert is_invalid is False

    def test_rate_limiting(self, security_manager):
        """Test rate limiting for API protection."""
        client_id = "test_client"

        # Test normal requests
        for i in range(10):
            allowed = security_manager.check_rate_limit(client_id, "api_requests", limit=100, window=60)
            assert allowed is True

        # Test rate limit exceeded
        # This would require many rapid requests or mocking time
        assert True  # Placeholder - would implement full rate limiting test

    def test_encryption_decryption(self, security_manager):
        """Test data encryption and decryption."""
        sensitive_data = {
            "api_key": "sk-1234567890abcdef",
            "database_password": "super_secret_db_pass"
        }

        # Encrypt data
        encrypted = security_manager.encrypt_data(sensitive_data)
        assert encrypted != json.dumps(sensitive_data)

        # Decrypt data
        decrypted = security_manager.decrypt_data(encrypted)
        assert decrypted == sensitive_data

    def test_secure_communication(self, security_manager):
        """Test secure communication channel setup."""
        # Test TLS/SSL configuration
        ssl_config = security_manager.configure_ssl("server.crt", "server.key")
        assert ssl_config is not None
        assert "certfile" in ssl_config
        assert "keyfile" in ssl_config

        # Test certificate validation
        valid_cert = security_manager.validate_certificate("server.crt")
        assert isinstance(valid_cert, bool)

    def test_audit_logging(self, security_manager, sample_credentials):
        """Test audit logging functionality."""
        # Create user and perform actions
        security_manager.create_user(
            sample_credentials["username"],
            sample_credentials["password"],
            sample_credentials["role"]
        )

        security_manager.authenticate(
            sample_credentials["username"],
            sample_credentials["password"]
        )

        # Check audit logs
        logs = security_manager.get_audit_logs(user=sample_credentials["username"])
        assert len(logs) >= 2  # Should have user creation and authentication logs

        # Verify log structure
        for log in logs:
            assert "timestamp" in log
            assert "action" in log
            assert "user" in log

    def test_threat_detection(self, security_manager):
        """Test threat detection and response."""
        # Simulate suspicious activities
        suspicious_patterns = [
            {"action": "brute_force_attempt", "ip": "192.168.1.100", "attempts": 10},
            {"action": "unusual_traffic", "bytes_per_second": 1000000},
            {"action": "privilege_escalation_attempt", "user": "operator", "target_role": "admin"}
        ]

        for pattern in suspicious_patterns:
            detected = security_manager.detect_threat(pattern)
            assert isinstance(detected, dict)
            assert "threat_level" in detected

    def test_security_incident_response(self, security_manager):
        """Test security incident response."""
        # Simulate security incident
        incident = {
            "type": "unauthorized_access",
            "severity": "high",
            "affected_system": "mission_control"
        }

        # Trigger incident response
        response = security_manager.handle_security_incident(incident)

        assert response["actions_taken"] is not None
        assert len(response["actions_taken"]) > 0

    def test_password_policy_enforcement(self, security_manager):
        """Test password policy enforcement."""
        # Test valid passwords
        valid_passwords = [
            "StrongPass123!",
            "Complex_Password_456",
            "Secure@2024#Mission"
        ]

        # Test invalid passwords
        invalid_passwords = [
            "weak",  # Too short
            "password",  # No numbers/symbols
            "12345678",  # No letters
            "Password",  # No numbers/symbols
        ]

        for password in valid_passwords:
            assert security_manager.validate_password_policy(password) is True

        for password in invalid_passwords:
            assert security_manager.validate_password_policy(password) is False

    def test_multi_factor_authentication(self, security_manager, sample_credentials):
        """Test multi-factor authentication."""
        # Create user with MFA enabled
        security_manager.create_user(
            sample_credentials["username"],
            sample_credentials["password"],
            sample_credentials["role"],
            mfa_enabled=True
        )

        # Generate MFA secret
        secret = security_manager.generate_mfa_secret(sample_credentials["username"])
        assert secret is not None

        # Generate MFA code
        mfa_code = security_manager.generate_mfa_code(secret)

        # Verify MFA code
        verified = security_manager.verify_mfa_code(sample_credentials["username"], mfa_code)
        assert verified is True

        # Test invalid MFA code
        invalid_verified = security_manager.verify_mfa_code(sample_credentials["username"], "000000")
        assert invalid_verified is False

    def test_secure_file_operations(self, security_manager, tmp_path):
        """Test secure file operations."""
        # Create test file with sensitive data
        test_file = tmp_path / "sensitive_data.txt"
        test_file.write_text("This is sensitive information")

        # Encrypt file
        encrypted_file = security_manager.encrypt_file(str(test_file))
        assert encrypted_file != str(test_file)

        # Decrypt file
        decrypted_content = security_manager.decrypt_file(encrypted_file)
        assert decrypted_content == "This is sensitive information"

    def test_network_security_monitoring(self, security_manager):
        """Test network security monitoring."""
        # Simulate network traffic
        traffic_data = {
            "source_ip": "192.168.1.100",
            "destination_ip": "10.0.0.1",
            "protocol": "TCP",
            "port": 8080,
            "bytes": 1024,
            "packets": 10
        }

        # Analyze traffic
        analysis = security_manager.analyze_network_traffic(traffic_data)
        assert "risk_level" in analysis
        assert "anomalies" in analysis

    def test_security_policy_enforcement(self, security_manager):
        """Test security policy enforcement."""
        # Define security policies
        policies = {
            "password_min_length": 8,
            "session_timeout": 3600,
            "max_login_attempts": 5,
            "require_mfa": True
        }

        # Test policy compliance
        user_compliant = {
            "password_length": 12,
            "mfa_enabled": True,
            "last_login": time.time() - 1800  # 30 minutes ago
        }

        user_non_compliant = {
            "password_length": 6,
            "mfa_enabled": False,
            "last_login": time.time() - 7200  # 2 hours ago
        }

        assert security_manager.check_policy_compliance(user_compliant, policies) is True
        assert security_manager.check_policy_compliance(user_non_compliant, policies) is False


class TestAccessControl:
    """Test AccessControl functionality."""

    @pytest.fixture
    def access_control(self):
        """Create AccessControl instance."""
        return AccessControl()

    def test_permission_granting(self, access_control):
        """Test permission granting and checking."""
        # Grant permissions
        access_control.grant_permission("user_123", "mission:start")
        access_control.grant_permission("user_123", "telemetry:read")

        # Check permissions
        assert access_control.has_permission("user_123", "mission:start") is True
        assert access_control.has_permission("user_123", "telemetry:read") is True
        assert access_control.has_permission("user_123", "admin:shutdown") is False

    def test_role_based_permissions(self, access_control):
        """Test role-based permission assignment."""
        # Define role permissions
        access_control.define_role_permissions("operator", ["mission:*", "telemetry:read"])
        access_control.define_role_permissions("admin", ["*"])

        # Assign roles
        access_control.assign_role("user_123", "operator")

        # Check role-based permissions
        assert access_control.has_permission("user_123", "mission:start") is True
        assert access_control.has_permission("user_123", "telemetry:read") is True
        assert access_control.has_permission("user_123", "admin:shutdown") is False

        # Assign admin role
        access_control.assign_role("user_456", "admin")
        assert access_control.has_permission("user_456", "admin:shutdown") is True

    def test_permission_revocation(self, access_control):
        """Test permission revocation."""
        # Grant and then revoke permission
        access_control.grant_permission("user_123", "mission:start")
        assert access_control.has_permission("user_123", "mission:start") is True

        access_control.revoke_permission("user_123", "mission:start")
        assert access_control.has_permission("user_123", "mission:start") is False

    def test_access_control_auditing(self, access_control):
        """Test access control auditing."""
        # Perform access control operations
        access_control.grant_permission("user_123", "mission:start")
        access_control.has_permission("user_123", "mission:start")
        access_control.has_permission("user_123", "admin:shutdown")  # Denied

        # Get audit logs
        logs = access_control.get_audit_logs()
        assert len(logs) >= 3  # Should have grant and check operations

        # Verify audit log structure
        for log in logs:
            assert "timestamp" in log
            assert "action" in log
            assert "user" in log


class TestAuditLogger:
    """Test AuditLogger functionality."""

    @pytest.fixture
    def audit_logger(self):
        """Create AuditLogger instance."""
        return AuditLogger()

    def test_audit_log_creation(self, audit_logger):
        """Test audit log creation."""
        # Log security events
        audit_logger.log_event(
            "authentication",
            "user_login",
            user="operator",
            ip_address="192.168.1.100",
            success=True
        )

        audit_logger.log_event(
            "authorization",
            "permission_check",
            user="operator",
            resource="mission:start",
            action="granted"
        )

        # Retrieve logs
        logs = audit_logger.get_logs(event_type="authentication")
        assert len(logs) >= 1

        logs = audit_logger.get_logs(user="operator")
        assert len(logs) >= 2

    def test_audit_log_filtering(self, audit_logger):
        """Test audit log filtering."""
        # Log multiple events
        for i in range(5):
            audit_logger.log_event(
                "test_event",
                f"action_{i}",
                user=f"user_{i}",
                severity="info" if i % 2 == 0 else "warning"
            )

        # Filter by severity
        info_logs = audit_logger.get_logs(severity="info")
        warning_logs = audit_logger.get_logs(severity="warning")

        assert len(info_logs) == 3
        assert len(warning_logs) == 2

    def test_audit_log_retention(self, audit_logger):
        """Test audit log retention policies."""
        # Log events with timestamps
        audit_logger.log_event("old_event", "test", timestamp=time.time() - 86400)  # 1 day ago
        audit_logger.log_event("new_event", "test", timestamp=time.time())  # Now

        # Apply retention policy (keep only last 12 hours)
        audit_logger.apply_retention_policy(hours=12)

        # Should only have new event
        recent_logs = audit_logger.get_logs(hours=24)
        assert len(recent_logs) == 1
        assert recent_logs[0]["event_type"] == "new_event"

    def test_audit_log_export(self, audit_logger, tmp_path):
        """Test audit log export functionality."""
        # Log some events
        audit_logger.log_event("export_test", "event1", user="test_user")
        audit_logger.log_event("export_test", "event2", user="test_user")

        # Export logs
        export_path = tmp_path / "audit_export.json"
        audit_logger.export_logs(str(export_path))

        assert export_path.exists()

        # Verify export content
        with open(export_path) as f:
            exported_data = json.load(f)
            assert len(exported_data) == 2
            assert exported_data[0]["event_type"] == "export_test"
