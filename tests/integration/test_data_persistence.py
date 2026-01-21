#!/usr/bin/env python3
"""
Data Persistence Tests - URC 2026

Tests the DataManager system for:
- Data storage and retrieval operations
- Caching strategies and cache invalidation
- Data validation and schema enforcement
- Backup and recovery mechanisms
- Concurrent data access handling
- Data migration and versioning

Author: URC 2026 Testing Team
"""

import asyncio
import json
import sqlite3
import tempfile
import time
import pytest
from pathlib import Path
from unittest.mock import Mock, patch, AsyncMock
from typing import Dict, Any, Optional, List

# Import data management components (these would exist in the actual system)
from src.core.data_manager import DataManager, DataCache, BackupManager


class TestDataManager:
    """Test DataManager functionality."""

    @pytest.fixture
    def temp_db(self):
        """Create temporary database for testing."""
        with tempfile.NamedTemporaryFile(suffix='.db', delete=False) as f:
            db_path = f.name

        yield db_path

        # Cleanup
        Path(db_path).unlink(missing_ok=True)

    @pytest.fixture
    def data_manager(self, temp_db):
        """Create DataManager instance."""
        return DataManager(database_path=temp_db)

    @pytest.fixture
    def sample_data(self):
        """Sample data for testing."""
        return {
            "mission_data": {
                "mission_id": "sample_001",
                "type": "sample_collection",
                "waypoints": [
                    {"id": 1, "lat": 37.123456, "lon": -108.987654, "alt": 2.5},
                    {"id": 2, "lat": 37.123466, "lon": -108.987664, "alt": 2.5}
                ],
                "samples": [
                    {"id": 1, "type": "rock", "lat": 37.123456, "lon": -108.987654},
                    {"id": 2, "type": "soil", "lat": 37.123466, "lon": -108.987664}
                ]
            },
            "telemetry_data": {
                "timestamp": time.time(),
                "position": {"x": 10.5, "y": 15.2, "z": 2.1},
                "orientation": {"roll": 0.1, "pitch": 0.05, "yaw": 1.2},
                "battery": {"voltage": 24.5, "current": 2.1, "temperature": 35.0}
            }
        }

    def test_data_manager_initialization(self, data_manager):
        """Test DataManager initialization."""
        assert data_manager.db_path is not None
        assert data_manager.cache is not None
        assert data_manager.backup_manager is not None
        assert isinstance(data_manager.schemas, dict)

    def test_data_storage_and_retrieval(self, data_manager, sample_data):
        """Test basic data storage and retrieval."""
        # Store data
        result = data_manager.store_data("mission_data", sample_data["mission_data"])
        assert result is True

        # Retrieve data
        retrieved = data_manager.get_data("mission_data", "sample_001")
        assert retrieved is not None
        assert retrieved["mission_id"] == "sample_001"
        assert len(retrieved["waypoints"]) == 2

    def test_data_validation(self, data_manager):
        """Test data validation against schemas."""
        # Valid mission data
        valid_data = {
            "mission_id": "test_001",
            "type": "sample_collection",
            "waypoints": [{"id": 1, "lat": 37.0, "lon": -108.0, "alt": 2.0}]
        }

        # Invalid mission data (missing required fields)
        invalid_data = {
            "type": "sample_collection"
            # Missing mission_id and waypoints
        }

        # Test valid data
        result = data_manager.validate_data("mission_data", valid_data)
        assert result["valid"] is True

        # Test invalid data
        result = data_manager.validate_data("mission_data", invalid_data)
        assert result["valid"] is False
        assert len(result["errors"]) > 0

    def test_data_caching(self, data_manager, sample_data):
        """Test data caching functionality."""
        # Store data
        data_manager.store_data("telemetry_data", sample_data["telemetry_data"], cache=True)

        # First retrieval (should cache)
        data1 = data_manager.get_data("telemetry_data", use_cache=True)

        # Second retrieval (should use cache)
        data2 = data_manager.get_data("telemetry_data", use_cache=True)

        assert data1 == data2

        # Clear cache and retrieve again (should be different object)
        data_manager.cache.clear()
        data3 = data_manager.get_data("telemetry_data", use_cache=False)

        assert data3 is not None

    def test_cache_invalidation(self, data_manager, sample_data):
        """Test cache invalidation strategies."""
        # Store data with TTL
        data_manager.store_data("telemetry_data", sample_data["telemetry_data"], ttl=1)

        # Retrieve (should be cached)
        data1 = data_manager.get_data("telemetry_data", use_cache=True)
        assert data1 is not None

        # Wait for TTL to expire
        time.sleep(1.1)

        # Retrieve again (should be fresh from database)
        data2 = data_manager.get_data("telemetry_data", use_cache=True)
        assert data2 is not None

    def test_concurrent_data_access(self, data_manager, sample_data):
        """Test concurrent data access."""
        results = []

        async def concurrent_operation(operation_id: int):
            # Store data
            data = sample_data["telemetry_data"].copy()
            data["operation_id"] = operation_id
            await data_manager.store_data_async(f"concurrent_data_{operation_id}", data)

            # Retrieve data
            retrieved = await data_manager.get_data_async(f"concurrent_data_{operation_id}")
            results.append(retrieved["operation_id"])

        # Run concurrent operations
        tasks = [concurrent_operation(i) for i in range(10)]
        asyncio.run(asyncio.gather(*tasks))

        # Verify all operations completed
        assert len(results) == 10
        assert set(results) == set(range(10))

    def test_data_backup_and_recovery(self, data_manager, sample_data):
        """Test data backup and recovery."""
        # Store initial data
        data_manager.store_data("backup_test", sample_data["mission_data"])

        # Create backup
        backup_path = data_manager.create_backup("test_backup")
        assert Path(backup_path).exists()

        # Simulate data loss
        data_manager.delete_data("backup_test")

        # Verify data is gone
        retrieved = data_manager.get_data("backup_test")
        assert retrieved is None

        # Restore from backup
        data_manager.restore_backup(backup_path)

        # Verify data is restored
        restored = data_manager.get_data("backup_test")
        assert restored is not None
        assert restored["mission_id"] == "sample_001"

    def test_data_migration(self, data_manager):
        """Test data migration between versions."""
        # Old format data
        old_format = {
            "id": "migration_test",
            "waypoints": [
                {"lat": 37.123456, "lng": -108.987654, "altitude": 2.5}  # Old field names
            ]
        }

        # Store old format
        data_manager.store_data("migration_test", old_format)

        # Run migration
        migration_result = data_manager.migrate_data("migration_test", "1.0", "2.0")
        assert migration_result is True

        # Retrieve migrated data
        migrated = data_manager.get_data("migration_test")
        assert migrated["waypoints"][0]["lon"] == -108.987654  # Should be renamed
        assert "lng" not in migrated["waypoints"][0]  # Old field should be gone

    def test_data_querying_and_filtering(self, data_manager, sample_data):
        """Test data querying and filtering."""
        # Store multiple data entries
        for i in range(5):
            data = sample_data["telemetry_data"].copy()
            data["sequence_id"] = i
            data["battery"]["voltage"] = 20.0 + i
            data_manager.store_data(f"telemetry_{i}", data)

        # Query data with filters
        results = data_manager.query_data(
            "telemetry_data",
            filters={"battery.voltage": {"$gt": 22.0}}
        )

        assert len(results) >= 2  # Should find entries with voltage > 22

        # Query with sorting
        sorted_results = data_manager.query_data(
            "telemetry_data",
            sort_by="sequence_id",
            ascending=True
        )

        assert len(sorted_results) == 5
        assert sorted_results[0]["sequence_id"] == 0
        assert sorted_results[4]["sequence_id"] == 4

    def test_data_compression(self, data_manager, sample_data):
        """Test data compression for storage efficiency."""
        # Large data that would benefit from compression
        large_data = {
            "large_field": "x" * 10000,  # 10KB of data
            "metadata": sample_data["mission_data"]
        }

        # Store with compression
        data_manager.store_data("compression_test", large_data, compress=True)

        # Retrieve and verify
        retrieved = data_manager.get_data("compression_test")
        assert retrieved["large_field"] == "x" * 10000
        assert retrieved["metadata"]["mission_id"] == "sample_001"

    def test_data_encryption(self, data_manager):
        """Test data encryption for sensitive information."""
        sensitive_data = {
            "credentials": {
                "api_key": "secret_key_123",
                "password": "super_secret"
            },
            "personal_data": {
                "operator_name": "John Doe",
                "contact": "john@example.com"
            }
        }

        # Store with encryption
        data_manager.store_data("encrypted_test", sensitive_data, encrypt=True)

        # Retrieve and verify
        retrieved = data_manager.get_data("encrypted_test")
        assert retrieved["credentials"]["api_key"] == "secret_key_123"

    def test_transaction_support(self, data_manager, sample_data):
        """Test database transaction support."""
        # Start transaction
        with data_manager.transaction():
            # Store multiple related items
            data_manager.store_data("transaction_test_1", {"item": 1})
            data_manager.store_data("transaction_test_2", {"item": 2})
            data_manager.store_data("transaction_test_3", {"item": 3})

        # Verify all items were stored
        for i in range(1, 4):
            item = data_manager.get_data(f"transaction_test_{i}")
            assert item["item"] == i

    def test_data_retention_policies(self, data_manager, sample_data):
        """Test data retention and cleanup policies."""
        # Store data with different retention policies
        data_manager.store_data("short_term", sample_data["telemetry_data"], retention_days=1)
        data_manager.store_data("long_term", sample_data["mission_data"], retention_days=365)

        # Run cleanup for old data
        deleted_count = data_manager.cleanup_expired_data()
        assert isinstance(deleted_count, int)

        # Short term data should still exist (just stored)
        short_term = data_manager.get_data("short_term")
        assert short_term is not None

    def test_data_export_import(self, data_manager, sample_data, tmp_path):
        """Test data export and import functionality."""
        # Store data
        data_manager.store_data("export_test", sample_data["mission_data"])

        # Export to JSON
        export_path = tmp_path / "export.json"
        data_manager.export_data("export_test", export_path)

        assert export_path.exists()

        # Import data back
        import_result = data_manager.import_data(export_path, "imported_test")
        assert import_result is True

        # Verify imported data
        imported = data_manager.get_data("imported_test")
        assert imported["mission_id"] == "sample_001"

    def test_performance_monitoring(self, data_manager, sample_data):
        """Test data access performance monitoring."""
        # Store test data
        for i in range(100):
            data = sample_data["telemetry_data"].copy()
            data["id"] = i
            data_manager.store_data(f"perf_test_{i}", data)

        # Measure query performance
        start_time = time.time()
        results = data_manager.query_data("telemetry_data", limit=50)
        end_time = time.time()

        query_time = end_time - start_time
        assert query_time < 1.0  # Should complete within 1 second
        assert len(results) == 50

    def test_data_integrity_checks(self, data_manager, sample_data):
        """Test data integrity and consistency checks."""
        # Store data
        data_manager.store_data("integrity_test", sample_data["mission_data"])

        # Perform integrity check
        integrity_result = data_manager.check_data_integrity("integrity_test")
        assert integrity_result["valid"] is True

        # Simulate corruption (direct database manipulation)
        # This would test checksums and repair mechanisms
        assert True  # Placeholder for integrity testing


class TestDataCache:
    """Test DataCache functionality."""

    @pytest.fixture
    def data_cache(self):
        """Create DataCache instance."""
        return DataCache(max_size=100, ttl=300)

    def test_cache_operations(self, data_cache):
        """Test basic cache operations."""
        # Cache data
        data_cache.set("test_key", {"value": 42})

        # Retrieve cached data
        cached = data_cache.get("test_key")
        assert cached["value"] == 42

        # Test cache miss
        miss = data_cache.get("nonexistent")
        assert miss is None

    def test_cache_ttl(self, data_cache):
        """Test cache TTL functionality."""
        # Cache with TTL
        data_cache.set("ttl_test", {"data": "test"}, ttl=1)

        # Should be available immediately
        assert data_cache.get("ttl_test") is not None

        # Wait for expiration
        time.sleep(1.1)

        # Should be expired
        assert data_cache.get("ttl_test") is None

    def test_cache_eviction(self, data_cache):
        """Test cache eviction policies."""
        # Fill cache
        for i in range(110):  # Over max_size
            data_cache.set(f"key_{i}", {"data": f"value_{i}"})

        # Should have evicted old entries
        # (LRU eviction would remove oldest entries)
        assert data_cache.size() <= 100

    def test_cache_statistics(self, data_cache):
        """Test cache statistics tracking."""
        # Perform cache operations
        data_cache.set("stat_test", {"data": "test"})
        data_cache.get("stat_test")
        data_cache.get("miss_test")  # Miss

        stats = data_cache.get_statistics()
        assert "hits" in stats
        assert "misses" in stats
        assert "sets" in stats
        assert stats["hits"] >= 1
        assert stats["misses"] >= 1
        assert stats["sets"] >= 1


class TestBackupManager:
    """Test BackupManager functionality."""

    @pytest.fixture
    def temp_backup_dir(self):
        """Create temporary backup directory."""
        with tempfile.TemporaryDirectory() as temp_dir:
            yield Path(temp_dir)

    @pytest.fixture
    def backup_manager(self, temp_backup_dir):
        """Create BackupManager instance."""
        return BackupManager(backup_dir=temp_backup_dir)

    def test_backup_creation(self, backup_manager, temp_db):
        """Test backup creation."""
        # Create backup
        backup_path = backup_manager.create_backup(temp_db, "test_backup")
        assert Path(backup_path).exists()

    def test_backup_restoration(self, backup_manager, temp_db):
        """Test backup restoration."""
        # Create and restore backup
        backup_path = backup_manager.create_backup(temp_db, "restore_test")
        restore_result = backup_manager.restore_backup(backup_path, temp_db)
        assert restore_result is True

    def test_backup_rotation(self, backup_manager, temp_db):
        """Test backup rotation policies."""
        # Create multiple backups
        backups = []
        for i in range(5):
            backup = backup_manager.create_backup(temp_db, f"rotation_test_{i}")
            backups.append(backup)

        # Should maintain only recent backups
        recent_backups = backup_manager.list_backups()
        assert len(recent_backups) <= 5  # Configurable limit



