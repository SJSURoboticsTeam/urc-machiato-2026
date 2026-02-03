#!/usr/bin/env python3
"""
URC 2026 Import Migration Script

Migrates imports from legacy configuration modules to the unified
infrastructure.config module.

Usage:
    python scripts/migrate_imports.py --dry-run  # Preview changes
    python scripts/migrate_imports.py            # Apply changes

Author: URC 2026 Migration Team
"""

import argparse
import logging
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Set

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)

# Import mappings: old -> new
IMPORT_MAPPINGS: Dict[str, str] = {
    # Config manager mappings
    "from src.core.config_manager import get_config": "from src.infrastructure.config import get_config",
    "from src.core.config_manager import get_system_config": "from src.infrastructure.config import get_system_config",
    "from src.core.config_manager import RoverConfig": "from src.infrastructure.config import RoverConfig",
    "from src.core.config_manager import SystemConfig": "from src.infrastructure.config import SystemConfig",
    "from src.core.config_manager import ConfigurationManager": "from src.infrastructure.config import get_urc_config",
    "from src.core.config_manager import get_config_manager": "from src.infrastructure.config import get_urc_config",
    "from src.core.config_manager import reload_config": "from src.infrastructure.config import reload_config",
    "from src.core.config_manager import get_sync_config": "from src.infrastructure.config import get_sync_config",
    "from src.core.config_manager import get_network_config": "from src.infrastructure.config import get_network_config",
    "from src.core.config_manager import get_safety_config": "from src.infrastructure.config import get_safety_config",
    "from src.core.config_manager import get_mission_config": "from src.infrastructure.config import get_mission_config",
    "from src.core.config_manager import Environment": "from src.infrastructure.config import Environment",
    "from src.core.config_manager import LogLevel": "from src.infrastructure.config import LogLevel",
    "from src.core.config_manager import SyncConfig": "from src.infrastructure.config import SyncConfig",
    "from src.core.config_manager import NetworkConfig": "from src.infrastructure.config import NetworkConfig",
    "from src.core.config_manager import SafetyConfig": "from src.infrastructure.config import SafetyConfig",
    "from src.core.config_manager import MissionConfig": "from src.infrastructure.config import MissionConfig",
    "from src.core.config_manager import NavigationConfig": "from src.infrastructure.config import NavigationConfig",
    "from src.core.config_manager import DatabaseConfig": "from src.infrastructure.config import DatabaseConfig",
    "from src.core.config_manager import RedisConfig": "from src.infrastructure.config import RedisConfig",
    "from src.core.config_manager import APIConfig": "from src.infrastructure.config import APIConfig",
    "from src.core.config_manager import PerformanceConfig": "from src.infrastructure.config import PerformanceConfig",
    # Dynaconf config mappings
    "from src.config.dynaconf_config import get_urc_config": "from src.infrastructure.config import get_urc_config",
    "from src.config.dynaconf_config import get_settings": "from src.infrastructure.config import get_settings",
    "from src.config.dynaconf_config import URCDynaconf": "from src.infrastructure.config import get_urc_config",
    "from src.config.dynaconf_config import config_get": "from src.infrastructure.config import config_get",
    "from src.config.dynaconf_config import config_set": "from src.infrastructure.config import config_set",
    "from src.config.dynaconf_config import validate_current_config": "from src.infrastructure.config import validate_config",
    "from src.config.dynaconf_config import reset_global_config": "from src.infrastructure.config import reset_settings",
    # Config models mappings
    "from src.core.config_models import RoverMode": "from src.infrastructure.config import RoverMode",
    "from src.core.config_models import SafetyLevel": "from src.infrastructure.config import SafetyLevel",
    "from src.core.config_models import SensorType": "from src.infrastructure.config import SensorType",
    "from src.core.config_models import MotorType": "from src.infrastructure.config import MotorType",
    "from src.core.config_models import CommunicationProtocol": "from src.infrastructure.config import CommunicationProtocol",
    "from src.core.config_models import MissionType": "from src.infrastructure.config import MissionType",
    "from src.core.config_models import WaypointConfig": "from src.infrastructure.config import WaypointConfig",
    "from src.core.config_models import URCConfigManager": "from src.infrastructure.config import get_urc_config",
    "from src.core.config_models import load_urc_config": "from src.infrastructure.config import get_urc_config",
    "from src.core.config_models import validate_config_data": "from src.infrastructure.config import validate_config",
}

# Regex patterns for more complex import replacements
IMPORT_PATTERNS: List[Tuple[str, str]] = [
    # Multi-import from config_manager
    (
        r"from src\.core\.config_manager import (.+)",
        r"from src.infrastructure.config import \1",
    ),
    # Multi-import from dynaconf_config
    (
        r"from src\.config\.dynaconf_config import (.+)",
        r"from src.infrastructure.config import \1",
    ),
    # Multi-import from config_models
    (
        r"from src\.core\.config_models import (.+)",
        r"from src.infrastructure.config import \1",
    ),
    # Module imports
    (r"import src\.core\.config_manager", r"import src.infrastructure.config"),
    (r"import src\.config\.dynaconf_config", r"import src.infrastructure.config"),
    (r"import src\.core\.config_models", r"import src.infrastructure.config"),
]

# Files/directories to exclude
EXCLUDE_PATTERNS: Set[str] = {
    "__pycache__",
    ".git",
    ".venv",
    "venv",
    "node_modules",
    "build",
    "install",
    "log",
    ".pytest_cache",
    ".mypy_cache",
    "dist",
    "*.egg-info",
    # Don't modify the new infrastructure config
    "src/infrastructure/config",
    # Don't modify the migration script itself
    "scripts/migrate_imports.py",
}


def should_process_file(filepath: Path) -> bool:
    """Check if file should be processed."""
    # Check exclusions
    for pattern in EXCLUDE_PATTERNS:
        if pattern in str(filepath):
            return False

    # Only process Python files
    return filepath.suffix == ".py"


def find_python_files(root_dir: Path) -> List[Path]:
    """Find all Python files in directory."""
    files = []
    for filepath in root_dir.rglob("*.py"):
        if should_process_file(filepath):
            files.append(filepath)
    return sorted(files)


def migrate_file_content(content: str) -> Tuple[str, List[str]]:
    """
    Migrate imports in file content.

    Returns:
        Tuple of (new_content, list_of_changes)
    """
    changes = []
    new_content = content

    # Apply exact string replacements first
    for old_import, new_import in IMPORT_MAPPINGS.items():
        if old_import in new_content:
            new_content = new_content.replace(old_import, new_import)
            changes.append(f"  {old_import} -> {new_import}")

    # Apply regex patterns
    for pattern, replacement in IMPORT_PATTERNS:
        matches = re.findall(pattern, new_content)
        if matches:
            new_content = re.sub(pattern, replacement, new_content)
            for match in matches:
                changes.append(f"  [regex] {pattern} matched: {match}")

    return new_content, changes


def process_file(filepath: Path, dry_run: bool = False) -> Tuple[bool, List[str]]:
    """
    Process a single file.

    Returns:
        Tuple of (was_modified, list_of_changes)
    """
    try:
        content = filepath.read_text(encoding="utf-8")
    except Exception as e:
        logger.error(f"Failed to read {filepath}: {e}")
        return False, []

    new_content, changes = migrate_file_content(content)

    if not changes:
        return False, []

    if not dry_run:
        try:
            filepath.write_text(new_content, encoding="utf-8")
        except Exception as e:
            logger.error(f"Failed to write {filepath}: {e}")
            return False, changes

    return True, changes


def run_migration(root_dir: Path, dry_run: bool = False) -> Dict[str, any]:
    """
    Run the migration.

    Returns:
        Migration results dictionary
    """
    logger.info(f"{'[DRY RUN] ' if dry_run else ''}Starting import migration...")
    logger.info(f"Root directory: {root_dir}")

    files = find_python_files(root_dir)
    logger.info(f"Found {len(files)} Python files to process")

    results = {
        "total_files": len(files),
        "modified_files": 0,
        "changes": [],
        "errors": [],
    }

    for filepath in files:
        modified, changes = process_file(filepath, dry_run)

        if modified:
            results["modified_files"] += 1
            relative_path = filepath.relative_to(root_dir)
            logger.info(f"{'Would modify' if dry_run else 'Modified'}: {relative_path}")
            for change in changes:
                logger.debug(change)
            results["changes"].append(
                {
                    "file": str(relative_path),
                    "changes": changes,
                }
            )

    logger.info(
        f"Migration complete: {results['modified_files']}/{results['total_files']} files modified"
    )

    return results


def main():
    parser = argparse.ArgumentParser(
        description="Migrate imports to unified configuration system"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Preview changes without modifying files"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Show detailed changes"
    )
    parser.add_argument(
        "root_dir",
        nargs="?",
        default=".",
        help="Root directory to process (default: current directory)",
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    root_dir = Path(args.root_dir).resolve()

    if not root_dir.exists():
        logger.error(f"Directory does not exist: {root_dir}")
        sys.exit(1)

    results = run_migration(root_dir, dry_run=args.dry_run)

    if args.dry_run and results["modified_files"] > 0:
        logger.info("\nTo apply changes, run without --dry-run flag")

    return 0 if not results.get("errors") else 1


if __name__ == "__main__":
    sys.exit(main())
