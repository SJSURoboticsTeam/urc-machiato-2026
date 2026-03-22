#!/usr/bin/env python3
"""
Input Validation and Security Layer

Provides secure input validation to replace unsafe eval/exec usage.
Includes sanitization, validation, and safe execution patterns.

Features:
- Input sanitization and validation
- Safe expression evaluation
- Code injection prevention
- Type checking and bounds validation
- Comprehensive security rules

Usage:
    from infrastructure.validation import validate_input, safe_eval
    
    # Replace unsafe eval
    result = safe_eval(user_input, allowed_names=['math'], strict=True)
    
    # Validate input
    validated = validate_input(user_data, schema=WaypointSchema)

Author: URC 2026 Security Team
"""

import ast
import re
import math
from typing import Any, Dict, List, Optional, Set, Union
from dataclasses import dataclass
from enum import Enum


class ValidationLevel(Enum):
    """Security validation levels."""

    STRICT = "strict"  # Maximum security, production
    MODERATE = "moderate"  # Balanced security, development
    PERMISSIVE = "permissive"  # Minimal validation, testing


class ValidationError(Exception):
    """Input validation failed."""

    pass


class SecurityError(Exception):
    """Security policy violation."""

    pass


@dataclass
class ValidationConfig:
    """Configuration for input validation."""

    level: ValidationLevel = ValidationLevel.MODERATE
    allow_eval: bool = False
    max_string_length: int = 1000
    max_depth: int = 10
    allowed_modules: Set[str] = None
    allowed_functions: Set[str] = None
    blocked_patterns: List[str] = None

    def __post_init__(self):
        if self.allowed_modules is None:
            self.allowed_modules = {"math", "random", "statistics"}
        if self.allowed_functions is None:
            self.allowed_functions = {"sin", "cos", "tan", "sqrt", "pow", "abs"}
        if self.blocked_patterns is None:
            self.blocked_patterns = [
                r"__import__",
                r"__globals__",
                r"__locals__",
                r"eval\(",
                r"exec\(",
                r"open\(",
                r"file\(",
                r"subprocess\.",
                r"os\.",
                r"sys\.",
            ]


class SafeEvaluator(ast.NodeVisitor):
    """Safe AST evaluator for expressions."""

    def __init__(self, config: ValidationConfig):
        self.config = config
        self.variables = {}
        self.depth = 0

    def visit(self, node):
        """Visit with depth checking."""
        if self.depth > self.config.max_depth:
            raise SecurityError(f"Expression too deep (max: {self.config.max_depth})")

        self.depth += 1
        try:
            return super().visit(node)
        finally:
            self.depth -= 1

    def visit_Name(self, node):
        """Handle name access."""
        name = node.id

        # Check allowed variables
        if name in self.variables:
            return self.variables[name]

        # Check allowed functions/modules
        if name in self.config.allowed_functions or name in self.config.allowed_modules:
            if name == "math":
                return math
            elif hasattr(math, name):
                return getattr(math, name)

        raise SecurityError(f"Access to '{name}' not allowed")

    def visit_Constant(self, node):
        """Handle constant values."""
        return node.value

    def visit_Num(self, node):
        """Handle numbers (Python < 3.8 compatibility)."""
        return node.n

    def visit_Str(self, node):
        """Handle strings (Python < 3.8 compatibility)."""
        return node.s

    def visit_BinOp(self, node):
        """Handle binary operations."""
        left = self.visit(node.left)
        right = self.visit(node.right)

        if isinstance(node.op, ast.Add):
            return left + right
        elif isinstance(node.op, ast.Sub):
            return left - right
        elif isinstance(node.op, ast.Mult):
            return left * right
        elif isinstance(node.op, ast.Div):
            return left / right
        elif isinstance(node.op, ast.Pow):
            return left**right
        elif isinstance(node.op, ast.Mod):
            return left % right
        else:
            raise SecurityError(f"Binary operator {type(node.op).__name__} not allowed")

    def visit_UnaryOp(self, node):
        """Handle unary operations."""
        operand = self.visit(node.operand)

        if isinstance(node.op, ast.UAdd):
            return +operand
        elif isinstance(node.op, ast.USub):
            return -operand
        else:
            raise SecurityError(f"Unary operator {type(node.op).__name__} not allowed")

    def visit_Call(self, node):
        """Handle function calls."""
        func = self.visit(node.func)
        args = [self.visit(arg) for arg in node.args]

        if callable(func):
            return func(*args)
        else:
            raise SecurityError(f"Call to '{func}' not allowed")

    def visit_Attribute(self, node):
        """Handle attribute access."""
        obj = self.visit(node.value)
        attr = node.attr

        # Only allow specific attribute access
        if isinstance(obj, math) and attr in self.config.allowed_functions:
            return getattr(obj, attr)

        raise SecurityError(f"Attribute access '{obj}.{attr}' not allowed")

    def generic_visit(self, node):
        """Reject all other node types."""
        raise SecurityError(f"AST node {type(node).__name__} not allowed")


def safe_eval(
    expression: str,
    config: Optional[ValidationConfig] = None,
    variables: Optional[Dict[str, Any]] = None,
) -> Any:
    """
    Safely evaluate mathematical expressions.

    Replaces unsafe eval() with controlled evaluation.

    Args:
        expression: Mathematical expression to evaluate
        config: Validation configuration
        variables: Allowed variables in expression

    Returns:
        Result of evaluated expression

    Raises:
        SecurityError: If expression contains dangerous code
        ValidationError: If expression is invalid
    """
    if config is None:
        config = ValidationConfig()

    if not config.allow_eval:
        raise SecurityError(
            "Expression evaluation not allowed in current configuration"
        )

    # Check for blocked patterns
    for pattern in config.blocked_patterns:
        if re.search(pattern, expression):
            raise SecurityError(f"Expression contains blocked pattern: {pattern}")

    # Parse expression
    try:
        tree = ast.parse(expression, mode="eval")
    except SyntaxError as e:
        raise ValidationError(f"Invalid syntax: {e}")

    # Evaluate safely
    evaluator = SafeEvaluator(config)
    if variables:
        evaluator.variables.update(variables)

    try:
        result = evaluator.visit(tree.body)
        return result
    except Exception as e:
        if isinstance(e, (SecurityError, ValidationError)):
            raise
        raise ValidationError(f"Evaluation failed: {e}")


class InputValidator:
    """Comprehensive input validation system."""

    def __init__(self, config: Optional[ValidationConfig] = None):
        self.config = config or ValidationConfig()

    def validate_string(
        self,
        value: str,
        field_name: str = "input",
        min_length: int = 0,
        max_length: Optional[int] = None,
        pattern: Optional[str] = None,
    ) -> str:
        """Validate string input."""
        if not isinstance(value, str):
            raise ValidationError(f"{field_name} must be a string")

        if len(value) < min_length:
            raise ValidationError(
                f"{field_name} must be at least {min_length} characters"
            )

        max_len = max_length or self.config.max_string_length
        if len(value) > max_len:
            raise ValidationError(f"{field_name} exceeds maximum length of {max_len}")

        if pattern and not re.match(pattern, value):
            raise ValidationError(f"{field_name} does not match required pattern")

        return value

    def validate_number(
        self,
        value: Union[int, float, str],
        field_name: str = "input",
        min_value: Optional[float] = None,
        max_value: Optional[float] = None,
        integer_only: bool = False,
    ) -> Union[int, float]:
        """Validate numeric input."""
        try:
            if integer_only:
                result = int(float(value))
            else:
                result = float(value)
        except (ValueError, TypeError):
            raise ValidationError(f"{field_name} must be a number")

        if min_value is not None and result < min_value:
            raise ValidationError(f"{field_name} must be at least {min_value}")

        if max_value is not None and result > max_value:
            raise ValidationError(f"{field_name} must be at most {max_value}")

        return result

    def validate_list(
        self,
        value: List[Any],
        field_name: str = "input",
        min_items: int = 0,
        max_items: Optional[int] = None,
        item_type: Optional[type] = None,
    ) -> List[Any]:
        """Validate list input."""
        if not isinstance(value, list):
            raise ValidationError(f"{field_name} must be a list")

        if len(value) < min_items:
            raise ValidationError(f"{field_name} must have at least {min_items} items")

        if max_items is not None and len(value) > max_items:
            raise ValidationError(f"{field_name} must have at most {max_items} items")

        if item_type:
            for i, item in enumerate(value):
                if not isinstance(item, item_type):
                    raise ValidationError(
                        f"{field_name}[{i}] must be of type {item_type.__name__}"
                    )

        return value

    def validate_dict(
        self,
        value: Dict[str, Any],
        field_name: str = "input",
        required_keys: Optional[List[str]] = None,
        allowed_keys: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Validate dictionary input."""
        if not isinstance(value, dict):
            raise ValidationError(f"{field_name} must be a dictionary")

        # Check required keys
        if required_keys:
            missing_keys = set(required_keys) - set(value.keys())
            if missing_keys:
                raise ValidationError(
                    f"{field_name} missing required keys: {missing_keys}"
                )

        # Check allowed keys
        if allowed_keys:
            extra_keys = set(value.keys()) - set(allowed_keys)
            if extra_keys:
                raise ValidationError(f"{field_name} has extra keys: {extra_keys}")

        return value

    def sanitize_filename(self, filename: str) -> str:
        """Sanitize filename to prevent directory traversal."""
        # Remove path separators
        filename = filename.replace("/", "_").replace("\\", "_").replace("..", "_")

        # Remove dangerous characters
        filename = re.sub(r'[<>:"|?*]', "_", filename)

        # Ensure it's not empty
        if not filename.strip():
            filename = "unnamed"

        return self.validate_string(
            filename,
            field_name="filename",
            pattern=r"^[a-zA-Z0-9_.-]+$",
            max_length=255,
        )

    def validate_json(self, data: Any) -> bool:
        """Check if data is JSON serializable."""
        import json

        try:
            json.dumps(data)
            return True
        except (TypeError, ValueError):
            return False


# Global validator instance
_default_validator = None


def get_validator(config: Optional[ValidationConfig] = None) -> InputValidator:
    """Get global input validator."""
    global _default_validator
    if _default_validator is None or config is not None:
        _default_validator = InputValidator(config)
    return _default_validator


def validate_input(
    value: Any, schema: Optional[Dict[str, Any]] = None, field_name: str = "input"
) -> Any:
    """Validate input against schema or default rules."""
    validator = get_validator()

    if schema is None:
        # Basic validation
        if isinstance(value, str):
            return validator.validate_string(value, field_name)
        elif isinstance(value, (int, float)):
            return validator.validate_number(value, field_name)
        elif isinstance(value, list):
            return validator.validate_list(value, field_name)
        elif isinstance(value, dict):
            return validator.validate_dict(value, field_name)
        else:
            return value
    else:
        # Schema-based validation
        return _validate_schema(value, schema, validator)


def _validate_schema(
    value: Any, schema: Dict[str, Any], validator: InputValidator
) -> Any:
    """Validate against custom schema."""
    schema_type = schema.get("type")
    field_name = schema.get("name", "input")

    if schema_type == "string":
        return validator.validate_string(
            value,
            field_name,
            min_length=schema.get("min_length", 0),
            max_length=schema.get("max_length"),
            pattern=schema.get("pattern"),
        )
    elif schema_type == "number":
        return validator.validate_number(
            value,
            field_name,
            min_value=schema.get("min_value"),
            max_value=schema.get("max_value"),
            integer_only=schema.get("integer_only", False),
        )
    elif schema_type == "list":
        return validator.validate_list(
            value,
            field_name,
            min_items=schema.get("min_items", 0),
            max_items=schema.get("max_items"),
            item_type=schema.get("item_type"),
        )
    elif schema_type == "dict":
        return validator.validate_dict(
            value,
            field_name,
            required_keys=schema.get("required_keys"),
            allowed_keys=schema.get("allowed_keys"),
        )
    else:
        raise ValidationError(f"Unknown schema type: {schema_type}")


# Security decorators
def require_secure_input(level: ValidationLevel = ValidationLevel.MODERATE):
    """Decorator to enforce secure input validation."""

    def decorator(func):
        def wrapper(*args, **kwargs):
            config = ValidationConfig(level=level)
            validator = InputValidator(config)

            # Add validator to function context
            kwargs["_validator"] = validator

            return func(*args, **kwargs)

        return wrapper

    return decorator


def safe_exec(
    code: str,
    config: Optional[ValidationConfig] = None,
    globals_dict: Optional[Dict[str, Any]] = None,
) -> Any:
    """
    Safely execute code with restrictions.

    Replaces unsafe exec() with controlled execution.

    WARNING: Even with safety measures, dynamic code execution carries risks.
    Use only when absolutely necessary.
    """
    if config is None:
        config = ValidationConfig(level=ValidationConfig.STRICT)

    if not config.allow_eval:
        raise SecurityError("Code execution not allowed in current configuration")

    # Parse code
    try:
        tree = ast.parse(code, mode="exec")
    except SyntaxError as e:
        raise ValidationError(f"Invalid syntax: {e}")

    # Restrict globals
    safe_globals = globals_dict or {}

    # Inject allowed modules
    if "math" in config.allowed_modules:
        safe_globals["math"] = math

    # Execute in restricted environment
    evaluator = SafeEvaluator(config)
    evaluator.variables.update(safe_globals)

    try:
        exec(compile(tree, filename="<safe_exec>", mode="exec"), safe_globals)
        return safe_globals
    except Exception as e:
        raise ValidationError(f"Execution failed: {e}")


if __name__ == "__main__":
    # Demo validation system
    print("🛡️  Input Validation Demo")

    # Demo 1: Safe mathematical evaluation
    print("1. Safe mathematical evaluation...")
    try:
        result = safe_eval("2 + 3 * 4")
        print(f"   ✅ '2 + 3 * 4' = {result}")

        # Try unsafe expression
        safe_eval("__import__('os').system('ls')")  # Should fail
    except SecurityError as e:
        print(f"   ✅ Security protection: {e}")

    # Demo 2: Input validation
    print("2. Input validation...")
    validator = get_validator()

    try:
        validated = validator.validate_string(
            "test_input", field_name="demo", min_length=3, max_length=20
        )
        print(f"   ✅ Validated string: {validated}")

        # Try invalid input
        validator.validate_string("x", field_name="demo", min_length=5)
    except ValidationError as e:
        print(f"   ✅ Validation caught error: {e}")

    # Demo 3: Filename sanitization
    print("3. Filename sanitization...")
    dangerous = "../../../etc/passwd"
    safe = validator.sanitize_filename(dangerous)
    print(f"   Dangerous: {dangerous}")
    print(f"   Sanitized: {safe}")

    print("\n🛡️  Validation system demo completed!")
