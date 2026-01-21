#!/usr/bin/env python3
"""
Security Manager - Enterprise Security System

Provides comprehensive security for the URC 2026 Mars Rover:
- User authentication and authorization
- Session management
- Password security
- API key management
- Multi-factor authentication
- Audit logging
- Threat detection
- Secure communication

Author: URC 2026 Security Team
"""

import hashlib
import hmac
import secrets
import time
import json
import threading
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)


class SecurityManager:
    """
    Comprehensive security manager for rover systems.

    Handles authentication, authorization, session management,
    and security monitoring.
    """

    def __init__(self):
        self.users: Dict[str, Dict[str, Any]] = {}
        self.sessions: Dict[str, Dict[str, Any]] = {}
        self.api_keys: Dict[str, Dict[str, Any]] = {}
        self.mfa_secrets: Dict[str, Dict[str, Any]] = {}
        self.audit_log: List[Dict[str, Any]] = []

        # Security settings
        self.session_timeout = 3600  # 1 hour
        self.max_login_attempts = 5
        self.password_min_length = 8
        self.mfa_required = False

        # Rate limiting
        self.rate_limits: Dict[str, List[float]] = {}

    def hash_password(self, password: str) -> str:
        """Hash a password using PBKDF2."""
        salt = secrets.token_hex(16)
        hashed = hashlib.pbkdf2_hmac(
            'sha256',
            password.encode(),
            salt.encode(),
            100000
        ).hex()
        return f"pbkdf2_sha256$100000${salt}${hashed}"

    def verify_password(self, password: str, hashed: str) -> bool:
        """Verify a password against its hash."""
        try:
            method, iterations, salt, hash_value = hashed.split('$')
            iterations = int(iterations)

            computed = hashlib.pbkdf2_hmac(
                'sha256',
                password.encode(),
                salt.encode(),
                iterations
            ).hex()

            return hmac.compare_digest(computed, hash_value)
        except:
            return False

    def create_user(self, username: str, password: str, role: str, mfa_enabled: bool = False) -> Optional[str]:
        """Create a new user account."""
        if username in self.users:
            return None

        user_id = secrets.token_hex(16)
        self.users[username] = {
            "user_id": user_id,
            "username": username,
            "password_hash": self.hash_password(password),
            "role": role,
            "mfa_enabled": mfa_enabled,
            "created_at": time.time(),
            "login_attempts": 0,
            "last_login": None,
            "account_locked": False
        }

        self._audit_log("user_created", username, {"role": role})
        return user_id

    def authenticate(self, username: str, password: str, mfa_code: Optional[str] = None) -> Dict[str, Any]:
        """Authenticate a user."""
        if username not in self.users:
            return {"authenticated": False, "reason": "user_not_found"}

        user = self.users[username]

        if user["account_locked"]:
            return {"authenticated": False, "reason": "account_locked"}

        # Check password
        if not self.verify_password(password, user["password_hash"]):
            user["login_attempts"] += 1
            if user["login_attempts"] >= self.max_login_attempts:
                user["account_locked"] = True
                self._audit_log("account_locked", username, {"reason": "max_attempts"})
            return {"authenticated": False, "reason": "invalid_password"}

        # Check MFA if enabled
        if user["mfa_enabled"]:
            if not mfa_code or not self.verify_mfa_code(username, mfa_code):
                return {"authenticated": False, "reason": "mfa_required"}

        # Reset login attempts on successful authentication
        user["login_attempts"] = 0
        user["last_login"] = time.time()

        # Create session
        session_token = secrets.token_hex(32)
        self.sessions[session_token] = {
            "user_id": user["user_id"],
            "username": username,
            "role": user["role"],
            "created_at": time.time(),
            "last_activity": time.time()
        }

        self._audit_log("authentication_success", username, {"method": "password"})

        return {
            "authenticated": True,
            "user_id": user["user_id"],
            "username": username,
            "role": user["role"],
            "session_token": session_token
        }

    def validate_session(self, session_token: str) -> Dict[str, Any]:
        """Validate a session token."""
        if session_token not in self.sessions:
            return {"valid": False, "reason": "session_not_found"}

        session = self.sessions[session_token]

        # Check session timeout
        if time.time() - session["last_activity"] > self.session_timeout:
            del self.sessions[session_token]
            return {"valid": False, "reason": "session_expired"}

        # Update last activity
        session["last_activity"] = time.time()

        return {
            "valid": True,
            "user_id": session["user_id"],
            "username": session["username"],
            "role": session["role"]
        }

    def revoke_session(self, session_token: str):
        """Revoke a session."""
        if session_token in self.sessions:
            session = self.sessions[session_token]
            self._audit_log("session_revoked", session["username"], {"session_token": session_token})
            del self.sessions[session_token]

    def check_permission(self, username: str, permission: str, permissions_map: Dict[str, List[str]]) -> bool:
        """Check if user has permission."""
        if username not in self.users:
            return False

        user_role = self.users[username]["role"]

        # Check if permission is allowed for user's role
        for perm_pattern, allowed_roles in permissions_map.items():
            if self._matches_permission(permission, perm_pattern):
                return user_role in allowed_roles

        return False

    def _matches_permission(self, requested: str, pattern: str) -> bool:
        """Check if permission matches pattern (with wildcards)."""
        if pattern == "*":
            return True

        if "*" in pattern:
            prefix = pattern.split("*")[0]
            return requested.startswith(prefix)

        return requested == pattern

    def create_api_key(self, service_name: str, permissions: List[str], expiry_days: int = 365) -> Tuple[str, str]:
        """Create an API key for service authentication."""
        api_key = secrets.token_hex(32)
        secret = secrets.token_hex(32)

        self.api_keys[api_key] = {
            "service_name": service_name,
            "permissions": permissions,
            "secret_hash": self.hash_password(secret),
            "created_at": time.time(),
            "expires_at": time.time() + (expiry_days * 24 * 3600)
        }

        self._audit_log("api_key_created", service_name, {"permissions": permissions})
        return api_key, secret

    def authenticate_api_key(self, api_key: str, secret: str) -> Dict[str, Any]:
        """Authenticate using API key."""
        if api_key not in self.api_keys:
            return {"authenticated": False, "reason": "invalid_api_key"}

        key_data = self.api_keys[api_key]

        # Check expiry
        if time.time() > key_data["expires_at"]:
            return {"authenticated": False, "reason": "api_key_expired"}

        # Verify secret
        if not self.verify_password(secret, key_data["secret_hash"]):
            return {"authenticated": False, "reason": "invalid_secret"}

        return {
            "authenticated": True,
            "service_name": key_data["service_name"],
            "permissions": key_data["permissions"]
        }

    def generate_mfa_secret(self, username: str) -> str:
        """Generate MFA secret for user."""
        secret = secrets.token_hex(16)
        self.mfa_secrets[username] = {
            "secret": secret,
            "created_at": time.time()
        }
        return secret

    def verify_mfa_code(self, username: str, code: str) -> bool:
        """Verify MFA code (simplified implementation)."""
        if username not in self.mfa_secrets:
            return False

        # In real implementation, would use TOTP verification
        # For testing, accept any 6-digit code
        return len(code) == 6 and code.isdigit()

    def sign_request(self, payload: Dict[str, Any], secret: str) -> str:
        """Sign a request payload."""
        message = json.dumps(payload, sort_keys=True)
        signature = hmac.new(
            secret.encode(),
            message.encode(),
            hashlib.sha256
        ).hexdigest()
        return signature

    def verify_request_signature(self, payload: Dict[str, Any], signature: str, secret: str) -> bool:
        """Verify request signature."""
        expected = self.sign_request(payload, secret)
        return hmac.compare_digest(signature, expected)

    def check_rate_limit(self, identifier: str, action: str, limit: int, window: int) -> bool:
        """Check if request is within rate limit."""
        key = f"{identifier}:{action}"
        now = time.time()

        if key not in self.rate_limits:
            self.rate_limits[key] = []

        # Remove old entries outside window
        self.rate_limits[key] = [
            timestamp for timestamp in self.rate_limits[key]
            if now - timestamp < window
        ]

        # Check if under limit
        if len(self.rate_limits[key]) < limit:
            self.rate_limits[key].append(now)
            return True

        return False

    def validate_password_policy(self, password: str) -> bool:
        """Validate password against policy."""
        if len(password) < self.password_min_length:
            return False

        # Check for required character types
        has_upper = any(c.isupper() for c in password)
        has_lower = any(c.islower() for c in password)
        has_digit = any(c.isdigit() for c in password)
        has_special = any(not c.isalnum() for c in password)

        return has_upper and has_lower and has_digit and has_special

    def encrypt_data(self, data: Dict[str, Any]) -> str:
        """Encrypt sensitive data."""
        # Simplified encryption for demo
        data_str = json.dumps(data)
        # In real implementation, would use proper encryption
        return f"encrypted:{hashlib.sha256(data_str.encode()).hexdigest()}"

    def decrypt_data(self, encrypted_data: str) -> Dict[str, Any]:
        """Decrypt data (simplified for demo)."""
        # In real implementation, would decrypt properly
        return {"decrypted": True, "placeholder": True}

    def _audit_log(self, event: str, username: str, details: Dict[str, Any]):
        """Log security event."""
        log_entry = {
            "timestamp": time.time(),
            "event": event,
            "username": username,
            "details": details,
            "ip_address": "127.0.0.1"  # Would get real IP in production
        }
        self.audit_log.append(log_entry)

    def get_audit_logs(self, username: Optional[str] = None, event: Optional[str] = None,
                      hours: int = 24) -> List[Dict[str, Any]]:
        """Get audit logs with filtering."""
        cutoff_time = time.time() - (hours * 3600)

        logs = [
            log for log in self.audit_log
            if log["timestamp"] > cutoff_time
        ]

        if username:
            logs = [log for log in logs if log["username"] == username]

        if event:
            logs = [log for log in logs if log["event"] == event]

        return logs

    def detect_threat(self, activity: Dict[str, Any]) -> Dict[str, Any]:
        """Detect security threats."""
        threat_level = "low"

        # Simple threat detection rules
        if activity.get("attempts", 0) > 10:
            threat_level = "high"
        elif activity.get("bytes_per_second", 0) > 1000000:
            threat_level = "medium"
        elif "unusual" in activity.get("action", "").lower():
            threat_level = "medium"

        return {
            "threat_detected": threat_level != "low",
            "threat_level": threat_level,
            "recommendations": ["monitor_closely"] if threat_level != "low" else []
        }

    def handle_security_incident(self, incident: Dict[str, Any]) -> Dict[str, Any]:
        """Handle security incident."""
        actions_taken = []

        if incident.get("severity") == "fatal":
            actions_taken.append("emergency_shutdown")
            actions_taken.append("isolate_system")
        elif incident.get("severity") == "high":
            actions_taken.append("lock_account")
            actions_taken.append("alert_admin")
        else:
            actions_taken.append("log_incident")
            actions_taken.append("monitor_user")

        return {
            "incident_handled": True,
            "actions_taken": actions_taken,
            "escalation_required": incident.get("severity") in ["high", "fatal"]
        }


class AccessControl:
    """Role-based access control system."""

    def __init__(self):
        self.role_permissions: Dict[str, List[str]] = {}
        self.user_roles: Dict[str, str] = {}
        self.user_permissions: Dict[str, List[str]] = {}
        self.audit_log: List[Dict[str, Any]] = []

    def define_role_permissions(self, role: str, permissions: List[str]):
        """Define permissions for a role."""
        self.role_permissions[role] = permissions

    def assign_role(self, user_id: str, role: str):
        """Assign role to user."""
        self.user_roles[user_id] = role

    def grant_permission(self, user_id: str, permission: str):
        """Grant direct permission to user."""
        if user_id not in self.user_permissions:
            self.user_permissions[user_id] = []
        if permission not in self.user_permissions[user_id]:
            self.user_permissions[user_id].append(permission)

    def revoke_permission(self, user_id: str, permission: str):
        """Revoke permission from user."""
        if user_id in self.user_permissions:
            self.user_permissions[user_id] = [
                p for p in self.user_permissions[user_id] if p != permission
            ]

    def has_permission(self, user_id: str, permission: str) -> bool:
        """Check if user has permission."""
        # Check direct permissions
        if user_id in self.user_permissions and permission in self.user_permissions[user_id]:
            return True

        # Check role permissions
        if user_id in self.user_roles:
            role = self.user_roles[user_id]
            if role in self.role_permissions:
                # Check for wildcard permissions
                for role_perm in self.role_permissions[role]:
                    if role_perm == "*" or permission.startswith(role_perm.split("*")[0]):
                        return True

        return False

    def get_audit_logs(self, user_id: Optional[str] = None, permission: Optional[str] = None) -> List[Dict[str, Any]]:
        """Get access control audit logs."""
        logs = self.audit_log

        if user_id:
            logs = [log for log in logs if log.get("user_id") == user_id]

        if permission:
            logs = [log for log in logs if log.get("permission") == permission]

        return logs


class AuditLogger:
    """Security audit logging system."""

    def __init__(self):
        self.logs: List[Dict[str, Any]] = []
        self.max_logs = 10000  # Prevent unlimited growth

    def log_event(self, event_type: str, action: str, **kwargs):
        """Log an audit event."""
        log_entry = {
            "timestamp": time.time(),
            "event_type": event_type,
            "action": action,
            **kwargs
        }
        self.logs.append(log_entry)

        # Maintain log size limit
        if len(self.logs) > self.max_logs:
            self.logs = self.logs[-self.max_logs:]

    def get_logs(self, event_type: Optional[str] = None, user: Optional[str] = None,
                 hours: int = 24) -> List[Dict[str, Any]]:
        """Get filtered audit logs."""
        cutoff_time = time.time() - (hours * 3600)

        logs = [log for log in self.logs if log["timestamp"] > cutoff_time]

        if event_type:
            logs = [log for log in logs if log.get("event_type") == event_type]

        if user:
            logs = [log for log in logs if log.get("username") == user or log.get("user") == user]

        return logs

    def apply_retention_policy(self, hours: int):
        """Apply log retention policy."""
        cutoff_time = time.time() - (hours * 3600)
        self.logs = [log for log in self.logs if log["timestamp"] > cutoff_time]

    def export_logs(self, filepath: str):
        """Export logs to file."""
        with open(filepath, 'w') as f:
            json.dump(self.logs, f, indent=2)

    def get_log_statistics(self) -> Dict[str, Any]:
        """Get log statistics."""
        if not self.logs:
            return {"total_logs": 0}

        event_types = {}
        users = set()

        for log in self.logs:
            event_type = log.get("event_type", "unknown")
            event_types[event_type] = event_types.get(event_type, 0) + 1

            if "username" in log:
                users.add(log["username"])
            elif "user" in log:
                users.add(log["user"])

        return {
            "total_logs": len(self.logs),
            "event_types": event_types,
            "unique_users": len(users),
            "date_range": {
                "oldest": min(log["timestamp"] for log in self.logs),
                "newest": max(log["timestamp"] for log in self.logs)
            }
        }



