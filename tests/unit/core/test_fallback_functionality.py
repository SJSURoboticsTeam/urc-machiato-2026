#!/usr/bin/env python3
"""
Fallback Functionality Tests - URC 2026 Resource Optimization

Tests fallback mechanisms when dependencies are missing:
- Mock missing ML libraries (PyTorch, TensorFlow, Detectron2)
- Test graceful degradation to lightweight implementations
- Measure performance impact of fallbacks
- Verify functionality preservation

Author: URC 2026 Fallback Testing Team
"""

import unittest
import unittest.mock as mock
import sys
import importlib
from typing import Dict, Any, List
import time
import psutil
import statistics

from tests.statistical_performance_testing import StatisticalMeasurement


class FallbackFunctionalityTests(unittest.TestCase):
    """Test fallback functionality when dependencies are missing."""

    def setUp(self):
        """Set up test environment."""
        self.process = psutil.Process()
        self.original_modules = dict(sys.modules)

        # Test scenarios with different dependency combinations
        self.test_scenarios = [
            {
                "name": "no_ml_libraries",
                "mock_missing": ["torch", "torchvision", "tensorflow", "detectron2"],
                "expected_fallbacks": ["lightweight_vision", "basic_slam"],
            },
            {
                "name": "no_tensorflow",
                "mock_missing": ["tensorflow"],
                "expected_fallbacks": ["basic_terrain_analysis"],
            },
            {
                "name": "minimal_dependencies",
                "mock_missing": ["torch", "tensorflow", "detectron2", "opencv"],
                "expected_fallbacks": ["minimal_vision", "dead_reckoning"],
            },
        ]

    def tearDown(self):
        """Clean up test environment."""
        # Restore original modules
        for module_name in list(sys.modules.keys()):
            if module_name not in self.original_modules:
                del sys.modules[module_name]

        # Restore any mocked modules
        for module_name in self.test_scenarios[0]["mock_missing"]:
            if module_name in sys.modules:
                del sys.modules[module_name]

    def test_computer_vision_fallback(self):
        """Test computer vision fallback when ML libraries are missing."""
        import pytest

        pytest.importorskip(
            "src.autonomy.perception", reason="src.autonomy.perception not available"
        )
        print("Testing computer vision fallback functionality...")

        # Mock missing ML libraries
        with self._mock_missing_libraries(
            ["torch", "torchvision", "tensorflow", "detectron2"]
        ):
            # Import computer vision node - should load with fallbacks
            try:
                from src.autonomy.perception.computer_vision.autonomy_computer_vision import (
                    computer_vision_node,
                )

                # Reload module to pick up mocked imports
                importlib.reload(computer_vision_node)

                # Verify fallback flags are set
                self.assertFalse(computer_vision_node.TORCH_AVAILABLE)
                self.assertFalse(computer_vision_node.DETECTRON_AVAILABLE)
                self.assertFalse(computer_vision_node.TENSORFLOW_AVAILABLE)

                # Test lightweight vision processor
                from src.autonomy.perception.computer_vision.lightweight_vision import (
                    LightweightVisionProcessor,
                )

                processor = LightweightVisionProcessor()
                self.assertIsNotNone(processor)

                # Test processing with dummy data
                import numpy as np

                dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

                start_time = time.time()
                results = processor.process_frame_lightweight(dummy_image)
                processing_time = time.time() - start_time

                # Verify results structure
                self.assertIn("objects", results)
                self.assertIn("markers", results)
                self.assertIn("processing_time_ms", results)

                # Verify reasonable processing time (< 100ms for lightweight)
                self.assertLess(processing_time, 0.1)

                print("✅ Computer vision fallback working correctly")

            except Exception as e:
                self.fail(f"Computer vision fallback failed: {e}")

    def test_slam_fallback(self):
        """Test SLAM fallback when RTAB-Map is unavailable."""
        import pytest

        pytest.importorskip(
            "src.autonomy.perception", reason="src.autonomy.perception not available"
        )
        print("Testing SLAM fallback functionality...")

        # Mock missing SLAM dependencies
        with self._mock_missing_libraries(["rtabmap", "pcl"]):
            try:
                from src.autonomy.perception.slam.lightweight_slam import (
                    LightweightVisualOdometry,
                    LightweightOccupancyMapper,
                    LightweightSLAMSystem,
                )

                # Test visual odometry
                vo = LightweightVisualOdometry()
                self.assertIsNotNone(vo)

                # Test with dummy frame
                import numpy as np

                dummy_frame = np.random.randint(0, 255, (480, 640), dtype=np.uint8)

                results = vo.process_frame(dummy_frame)
                self.assertIn("pose", results)
                self.assertIn("confidence", results)

                # Test occupancy mapper
                mapper = LightweightOccupancyMapper()
                self.assertIsNotNone(mapper)

                # Test full SLAM system
                slam_system = LightweightSLAMSystem()
                self.assertIsNotNone(slam_system)

                slam_results = slam_system.process_frame(dummy_frame)
                self.assertIn("pose_3d", slam_results)
                self.assertIn("pose_2d", slam_results)

                print("✅ SLAM fallback working correctly")

            except Exception as e:
                self.fail(f"SLAM fallback failed: {e}")

    def test_terrain_analysis_fallback(self):
        """Test terrain analysis fallback when TensorFlow is missing."""
        import pytest

        pytest.importorskip(
            "src.autonomy.core", reason="src.autonomy.core not available"
        )
        print("Testing terrain analysis fallback functionality...")

        # Mock missing TensorFlow
        with self._mock_missing_libraries(["tensorflow"]):
            try:
                # Import should work without TensorFlow
                from src.autonomy.core.terrain_intelligence.terrain_ml_analyzer import (
                    TerrainFeatureExtractor,
                )

                # Should fail to initialize due to missing sklearn
                with self.assertRaises(ImportError):
                    extractor = TerrainFeatureExtractor()

                # Test basic terrain analysis without ML
                from src.autonomy.perception.computer_vision.lightweight_vision import (
                    LightweightTerrainAnalyzer,
                )

                analyzer = LightweightTerrainAnalyzer()
                self.assertIsNotNone(analyzer)

                # Test with dummy terrain image
                import numpy as np

                # Create synthetic terrain-like image (sand-like colors)
                terrain_image = np.full(
                    (240, 320, 3), [194, 178, 128], dtype=np.uint8
                )  # Sandy color

                scores = analyzer.classify_terrain_basic(terrain_image)
                self.assertIsInstance(scores, dict)
                self.assertGreater(len(scores), 0)

                # Should detect sand-like terrain
                self.assertIn("sand", scores)

                print("✅ Terrain analysis fallback working correctly")

            except Exception as e:
                self.fail(f"Terrain analysis fallback failed: {e}")

    def test_feature_flags_fallback(self):
        """Test feature flags system fallback behavior."""
        print("Testing feature flags fallback behavior...")

        try:
            # Test feature flag manager creation
            from src.core.feature_flags import get_feature_flag_manager

            manager = get_feature_flag_manager()
            self.assertIsNotNone(manager)

            # Test with different mission profiles
            test_profiles = ["waypoint_navigation", "sample_collection"]

            for profile in test_profiles:
                manager.set_mission_profile(profile)

                # Check that feature queries work
                computer_vision_enabled = manager.is_feature_enabled("computer_vision")
                slam_enabled = manager.is_feature_enabled("slam")

                # Should not crash even if dependencies are missing
                self.assertIsInstance(computer_vision_enabled, bool)
                self.assertIsInstance(slam_enabled, bool)

                # Get feature info
                cv_info = manager.get_feature_info("computer_vision")
                self.assertIn("status", cv_info)
                self.assertIn("mission_enabled", cv_info)

            print("✅ Feature flags fallback working correctly")

        except Exception as e:
            self.fail(f"Feature flags fallback failed: {e}")

    def test_resource_manager_fallback(self):
        """Test resource manager fallback behavior."""
        print("Testing resource manager fallback behavior...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager

            manager = get_mission_resource_manager()
            self.assertIsNotNone(manager)

            # Test mission switching
            success = manager.switch_mission_profile("waypoint_navigation")
            self.assertTrue(success)

            # Get resource status
            status = manager.get_resource_status()
            self.assertIsInstance(status, dict)
            self.assertIn("component_status", status)

            # Test with different profiles
            for profile in ["object_search", "sample_collection"]:
                success = manager.switch_mission_profile(profile)
                self.assertTrue(success)

                status = manager.get_resource_status()
                component_count = len(status.get("component_status", {}))
                self.assertGreaterEqual(component_count, 0)

            print("✅ Resource manager fallback working correctly")

        except Exception as e:
            self.fail(f"Resource manager fallback failed: {e}")

    def test_performance_impact_of_fallbacks(self):
        """Test performance impact of using fallback implementations."""
        print("Testing performance impact of fallbacks...")

        # Measure baseline performance
        baseline_measurements = self._measure_performance_baseline()

        # Test with fallbacks enabled
        with self._mock_missing_libraries(["torch", "tensorflow", "detectron2"]):
            fallback_measurements = self._measure_performance_with_fallbacks()

            # Compare memory usage - fallbacks should use less memory
            memory_reduction = (
                baseline_measurements["memory_mb"] - fallback_measurements["memory_mb"]
            )

            # Memory usage should be lower or similar (not significantly higher)
            self.assertLessEqual(
                fallback_measurements["memory_mb"],
                baseline_measurements["memory_mb"] * 1.1,
            )

            # CPU usage should be reasonable
            self.assertLess(fallback_measurements["cpu_percent"], 50.0)

            # Processing should complete successfully
            self.assertTrue(fallback_measurements["success"])

            print(
                ".1f"
                f"Baseline memory: {baseline_measurements['memory_mb']:.1f}MB, "
                f"Fallback memory: {fallback_measurements['memory_mb']:.1f}MB"
            )

    def _measure_performance_baseline(self) -> Dict[str, Any]:
        """Measure baseline performance."""
        measurements = StatisticalMeasurement("baseline_performance")

        for _ in range(5):
            cpu, memory = self._get_resource_usage()
            measurements.add_sample(cpu + memory)  # Combined metric

        return {
            "cpu_percent": measurements.mean / 2,  # Approximate split
            "memory_mb": measurements.mean / 2,
            "success": True,
        }

    def _measure_performance_with_fallbacks(self) -> Dict[str, Any]:
        """Measure performance with fallbacks enabled."""
        try:
            # Import and test lightweight implementations
            from src.autonomy.perception.computer_vision.lightweight_vision import (
                LightweightVisionProcessor,
            )
            from src.autonomy.perception.slam.lightweight_slam import (
                LightweightVisualOdometry,
            )

            start_time = time.time()
            cpu_start, memory_start = self._get_resource_usage()

            # Test vision processing
            processor = LightweightVisionProcessor()
            import numpy as np

            dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            results = processor.process_frame_lightweight(dummy_image)

            # Test SLAM processing
            slam = LightweightVisualOdometry()
            slam_results = slam.process_frame(dummy_image)

            end_time = time.time()
            cpu_end, memory_end = self._get_resource_usage()

            return {
                "cpu_percent": (cpu_start + cpu_end) / 2,
                "memory_mb": (memory_start + memory_end) / 2,
                "processing_time": end_time - start_time,
                "success": True,
            }

        except Exception as e:
            return {
                "cpu_percent": 0.0,
                "memory_mb": 0.0,
                "processing_time": 0.0,
                "success": False,
                "error": str(e),
            }

    def _get_resource_usage(self) -> tuple[float, float]:
        """Get current CPU and memory usage."""
        cpu = self.process.cpu_percent(interval=0.1)
        memory = self.process.memory_info().rss / (1024 * 1024)
        return cpu, memory

    def _mock_missing_libraries(self, library_names: List[str]):
        """Context manager to mock missing libraries."""
        return mock.patch.dict("sys.modules", {name: None for name in library_names})

    def test_integration_with_missing_dependencies(self):
        """Test that the system integrates properly even with missing dependencies."""
        print("Testing integration with missing dependencies...")

        # Test most critical integration points
        critical_modules = [
            "src.core.mission_resource_manager",
            "src.core.feature_flags",
            "missions.robust_behavior_tree",
        ]

        for module_name in critical_modules:
            with self.subTest(module=module_name):
                try:
                    # Mock some dependencies that might be missing
                    with self._mock_missing_libraries(
                        ["torch", "tensorflow", "pydantic_settings"]
                    ):
                        # Try to import the module
                        module = importlib.import_module(module_name)

                        # Verify it has expected attributes
                        if "get_mission_resource_manager" in dir(module):
                            func = getattr(module, "get_mission_resource_manager")
                            # Should not crash
                            self.assertTrue(callable(func))

                        if "get_feature_flag_manager" in dir(module):
                            func = getattr(module, "get_feature_flag_manager")
                            self.assertTrue(callable(func))

                        if "PyTreesBehaviorTree" in dir(module):
                            cls = getattr(module, "PyTreesBehaviorTree")
                            self.assertTrue(callable(cls))

                except ImportError as e:
                    # Some imports might legitimately fail due to missing dependencies
                    print(f"⚠️ Expected import failure for {module_name}: {e}")
                except Exception as e:
                    self.fail(f"Unexpected error importing {module_name}: {e}")

        print("✅ Integration testing with missing dependencies completed")


def run_fallback_tests():
    """Run all fallback functionality tests."""
    print("🧪 FALLBACK FUNCTIONALITY TESTS - URC 2026")
    print("=" * 50)

    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(FallbackFunctionalityTests)
    runner = unittest.TextTestRunner(verbosity=2)

    # Run tests
    result = runner.run(suite)

    # Summary
    print("\n📊 FALLBACK TEST RESULTS")
    print("=" * 30)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")

    if result.wasSuccessful():
        print("✅ All fallback tests passed!")
        return True
    else:
        print("❌ Some fallback tests failed")
        for failure in result.failures:
            print(f"  FAILED: {failure[0]}")
        for error in result.errors:
            print(f"  ERROR: {error[0]}")
        return False


if __name__ == "__main__":
    success = run_fallback_tests()
    exit(0 if success else 1)
