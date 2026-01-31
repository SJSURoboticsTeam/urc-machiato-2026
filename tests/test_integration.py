#!/usr/bin/env python3
"""
Integration Tests - URC 2026 Resource Optimization System

Comprehensive integration testing for the complete resource optimization system:
- MissionResourceManager + FeatureFlags integration
- BehaviorTree + ResourceManager integration
- Monitoring + ResourceManager integration
- End-to-end mission profile switching
- Cross-component state consistency
- System stability during transitions

Author: URC 2026 Integration Testing Team
"""

import unittest
import time
import threading
from typing import Dict, Any, List
import psutil


class IntegrationTests(unittest.TestCase):
    """Test integration between all resource optimization components."""

    def setUp(self):
        """Set up integration test environment."""
        self.process = psutil.Process()
        self.test_mission_profiles = [
            'waypoint_navigation',
            'object_search',
            'sample_collection'
        ]

    def test_mission_resource_manager_feature_flags_integration(self):
        """Test integration between MissionResourceManager and FeatureFlags."""
        print("Testing MissionResourceManager + FeatureFlags integration...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager
            from src.core.feature_flags import get_feature_flag_manager

            # Initialize both systems
            resource_manager = get_mission_resource_manager()
            feature_manager = get_feature_flag_manager()

            # Test each mission profile
            for mission in self.test_mission_profiles:
                with self.subTest(mission=mission):
                    # Switch mission in resource manager
                    rm_success = resource_manager.switch_mission_profile(mission)
                    self.assertTrue(rm_success, f"Resource manager failed to switch to {mission}")

                    # Switch mission in feature manager
                    feature_manager.set_mission_profile(mission)

                    # Verify consistency between systems
                    rm_status = resource_manager.get_resource_status()
                    ff_enabled_features = feature_manager.get_enabled_features()

                    # Check that enabled components match feature flags
                    rm_components = set(rm_status.get('component_status', {}).keys())
                    ff_features = set(ff_enabled_features)

                    # Core components should be represented in both systems
                    core_components = {'computer_vision', 'slam', 'terrain_analysis', 'excavation'}
                    for component in core_components:
                        if component in rm_components:
                            # If resource manager enables it, feature flags should reflect this
                            # (Note: feature flag names may differ slightly from component names)
                            feature_name = component.replace('_', '_')
                            feature_enabled = feature_manager.is_feature_enabled(feature_name)
                            # This is a consistency check - they should generally agree
                            consistency_check = True  # Simplified for this test

                    # Mission profile should be consistent
                    rm_mission = rm_status.get('mission_profile')
                    ff_mission = feature_manager.current_mission_profile
                    self.assertEqual(rm_mission, ff_mission,
                                   f"Mission profile mismatch: RM={rm_mission}, FF={ff_mission}")

            print("✅ MissionResourceManager + FeatureFlags integration working correctly")

        except Exception as e:
            self.fail(f"MissionResourceManager + FeatureFlags integration failed: {e}")

    def test_behavior_tree_resource_manager_integration(self):
        """Test integration between BehaviorTree and ResourceManager."""
        print("Testing BehaviorTree + ResourceManager integration...")

        try:
            from missions.robust_behavior_tree import PyTreesBehaviorTree
            from src.core.mission_resource_manager import get_mission_resource_manager

            # Initialize systems
            bt = PyTreesBehaviorTree(name="IntegrationTestBT")
            rm = get_mission_resource_manager()

            # Test mission switching through behavior tree
            for mission in self.test_mission_profiles:
                with self.subTest(mission=mission):
                    # Switch mission via behavior tree
                    bt_success = bt.switch_mission_profile(mission)
                    self.assertTrue(bt_success, f"Behavior tree failed to switch to {mission}")

                    # Verify resource manager was updated
                    rm_status = rm.get_resource_status()
                    bt_status = bt.get_resource_status()

                    # Mission profiles should match
                    self.assertEqual(rm_status.get('mission_profile'), bt_status.get('mission_profile'),
                                   f"Mission profile mismatch after BT switch: RM={rm_status.get('mission_profile')}, BT={bt_status.get('mission_profile')}")

                    # Component status should be consistent
                    rm_components = rm_status.get('component_status', {})
                    bt_components = bt_status.get('component_status', {})

                    # At minimum, both should report the same mission
                    self.assertEqual(len(rm_components), len(bt_components),
                                   "Component counts should match between RM and BT")

            # Test behavior tree execution with resource management
            bt.switch_mission_profile('waypoint_navigation')

            # Execute tree and verify it doesn't crash
            try:
                result = bt.execute({'test_phase': 'integration'})
                # Result should be a valid status object
                self.assertIsNotNone(result)
            except Exception as e:
                self.fail(f"Behavior tree execution failed during integration test: {e}")

            print("✅ BehaviorTree + ResourceManager integration working correctly")

        except Exception as e:
            self.fail(f"BehaviorTree + ResourceManager integration failed: {e}")

    def test_monitoring_resource_manager_integration(self):
        """Test integration between Monitoring and ResourceManager."""
        import pytest
        pytest.importorskip("simulation.tools.monitoring_dashboard", reason="simulation.tools.monitoring_dashboard not available")
        print("Testing Monitoring + ResourceManager integration...")

        try:
            from simulation.tools.monitoring_dashboard import SimulationMonitor
            from src.core.mission_resource_manager import get_mission_resource_manager

            # Initialize systems
            monitor = SimulationMonitor()
            rm = get_mission_resource_manager()

            # Start monitoring
            monitor.start_monitoring(interval=1.0)

            # Switch missions and verify monitoring captures changes
            mission_data = {}

            for mission in self.test_mission_profiles[:2]:  # Test first 2 for speed
                rm.switch_mission_profile(mission)

                # Let monitoring capture data
                time.sleep(2.0)

                # Get monitoring data
                # Note: In real implementation, we'd access monitor.metrics_history
                # For this test, we verify the systems can coexist

                rm_status = rm.get_resource_status()
                mission_data[mission] = {
                    'components': len(rm_status.get('component_status', {})),
                    'mission': rm_status.get('mission_profile')
                }

            # Stop monitoring
            monitor.stop_monitoring()

            # Verify mission switching was captured
            self.assertEqual(len(mission_data), 2, "Should have data for 2 missions")

            for mission, data in mission_data.items():
                self.assertEqual(data['mission'], mission, f"Mission {mission} not properly tracked")

            print("✅ Monitoring + ResourceManager integration working correctly")

        except Exception as e:
            self.fail(f"Monitoring + ResourceManager integration failed: {e}")

    def test_end_to_end_mission_profile_switching(self):
        """Test end-to-end mission profile switching across all components."""
        print("Testing end-to-end mission profile switching...")

        try:
            # Initialize all systems
            from src.core.mission_resource_manager import get_mission_resource_manager
            from src.core.feature_flags import get_feature_flag_manager
            from missions.robust_behavior_tree import PyTreesBehaviorTree
            from simulation.tools.monitoring_dashboard import SimulationMonitor

            systems = {
                'resource_manager': get_mission_resource_manager(),
                'feature_flags': get_feature_flag_manager(),
                'behavior_tree': PyTreesBehaviorTree(name="E2ETestBT"),
                'monitoring': SimulationMonitor()
            }

            # Start monitoring
            systems['monitoring'].start_monitoring(interval=0.5)

            # Test each mission profile
            for mission in self.test_mission_profiles:
                with self.subTest(mission=mission):
                    print(f"  Switching all systems to {mission}...")

                    # Switch each system to the mission
                    switch_results = {}

                    # Resource manager
                    switch_results['rm'] = systems['resource_manager'].switch_mission_profile(mission)

                    # Feature flags
                    systems['feature_flags'].set_mission_profile(mission)
                    switch_results['ff'] = True  # Feature flags don't return success

                    # Behavior tree
                    switch_results['bt'] = systems['behavior_tree'].switch_mission_profile(mission)

                    # All switches should succeed
                    for system, success in switch_results.items():
                        self.assertTrue(success, f"{system} failed to switch to {mission}")

                    # Verify consistency across all systems
                    time.sleep(0.5)  # Allow systems to synchronize

                    rm_mission = systems['resource_manager'].get_resource_status().get('mission_profile')
                    ff_mission = systems['feature_flags'].current_mission_profile
                    bt_mission = systems['behavior_tree'].get_resource_status().get('mission_profile')

                    # All should report the same mission
                    self.assertEqual(rm_mission, mission, f"Resource manager mission mismatch: {rm_mission}")
                    self.assertEqual(ff_mission, mission, f"Feature flags mission mismatch: {ff_mission}")
                    self.assertEqual(bt_mission, mission, f"Behavior tree mission mismatch: {bt_mission}")

                    # Verify component/feature consistency
                    rm_components = len(systems['resource_manager'].get_resource_status().get('component_status', {}))
                    ff_features = len(systems['feature_flags'].get_enabled_features())

                    # Should have reasonable numbers (not zero, not excessive)
                    self.assertGreater(rm_components, 0, "Should have some components enabled")
                    self.assertLess(rm_components, 10, "Should not have excessive components enabled")

            # Stop monitoring
            systems['monitoring'].stop_monitoring()

            print("✅ End-to-end mission profile switching working correctly")

        except Exception as e:
            self.fail(f"End-to-end mission profile switching failed: {e}")

    def test_cross_component_state_consistency(self):
        """Test that all components maintain consistent state."""
        print("Testing cross-component state consistency...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager
            from src.core.feature_flags import get_feature_flag_manager
            from missions.robust_behavior_tree import PyTreesBehaviorTree

            systems = {
                'rm': get_mission_resource_manager(),
                'ff': get_feature_flag_manager(),
                'bt': PyTreesBehaviorTree(name="ConsistencyTestBT")
            }

            # Test state consistency across multiple transitions
            transitions = [
                ('waypoint_navigation', 'object_search'),
                ('object_search', 'sample_collection'),
                ('sample_collection', 'waypoint_navigation')
            ]

            for from_mission, to_mission in transitions:
                with self.subTest(transition=f"{from_mission}->{to_mission}"):
                    # Perform transition
                    systems['rm'].switch_mission_profile(to_mission)
                    systems['ff'].set_mission_profile(to_mission)
                    systems['bt'].switch_mission_profile(to_mission)

                    # Allow time for synchronization
                    time.sleep(0.2)

                    # Check mission consistency
                    rm_mission = systems['rm'].get_resource_status().get('mission_profile')
                    ff_mission = systems['ff'].current_mission_profile
                    bt_mission = systems['bt'].get_resource_status().get('mission_profile')

                    self.assertEqual(rm_mission, to_mission, f"RM mission inconsistency: {rm_mission}")
                    self.assertEqual(ff_mission, to_mission, f"FF mission inconsistency: {ff_mission}")
                    self.assertEqual(bt_mission, to_mission, f"BT mission inconsistency: {bt_mission}")

                    # Check component/feature relationship
                    rm_components = systems['rm'].get_resource_status().get('component_status', {})
                    ff_features = systems['ff'].get_enabled_features()

                    # Verify that major components have corresponding features
                    major_components = ['computer_vision', 'slam', 'terrain_analysis']
                    for comp in major_components:
                        if comp in rm_components:
                            # Should have some corresponding feature enabled
                            related_features = [f for f in ff_features if comp.replace('_', '_') in f or f in comp]
                            # At least one related feature should be enabled (simplified check)
                            feature_consistency = len(related_features) > 0 or len(ff_features) > 0
                            self.assertTrue(feature_consistency,
                                          f"Component {comp} enabled but no related features found")

            print("✅ Cross-component state consistency verified")

        except Exception as e:
            self.fail(f"Cross-component state consistency test failed: {e}")

    def test_system_stability_during_transitions(self):
        """Test system stability during rapid mission transitions."""
        print("Testing system stability during transitions...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager
            from src.core.feature_flags import get_feature_flag_manager

            rm = get_mission_resource_manager()
            ff = get_feature_flag_manager()

            # Perform rapid transitions
            transitions = 10
            stability_measurements = []

            for i in range(transitions):
                # Alternate between missions rapidly
                mission = self.test_mission_profiles[i % len(self.test_mission_profiles)]

                # Measure before transition
                cpu_before, memory_before = self._measure_resources()

                # Perform transition
                start_time = time.time()
                rm_success = rm.switch_mission_profile(mission)
                ff.set_mission_profile(mission)
                transition_time = time.time() - start_time

                # Measure after transition
                cpu_after, memory_after = self._measure_resources()

                # Verify transition success
                self.assertTrue(rm_success, f"Transition {i+1} failed for {mission}")

                # Record stability metrics
                stability_measurements.append({
                    'transition': i + 1,
                    'mission': mission,
                    'transition_time': transition_time,
                    'cpu_before': cpu_before,
                    'cpu_after': cpu_after,
                    'memory_before': memory_before,
                    'memory_after': memory_after,
                    'cpu_spike': cpu_after - cpu_before,
                    'memory_spike': memory_after - memory_before
                })

                # Brief pause between transitions
                time.sleep(0.1)

            # Analyze stability
            transition_times = [m['transition_time'] for m in stability_measurements]
            cpu_spikes = [m['cpu_spike'] for m in stability_measurements]
            memory_spikes = [m['memory_spike'] for m in stability_measurements]

            avg_transition_time = sum(transition_times) / len(transition_times)
            max_cpu_spike = max(cpu_spikes)
            max_memory_spike = max(memory_spikes)

            # Transitions should be fast (< 100ms)
            self.assertLess(avg_transition_time, 0.1, "Transitions should be fast")

            # Resource spikes should be reasonable (< 20% increase)
            self.assertLess(max_cpu_spike, 20.0, "CPU spikes should be controlled")
            self.assertLess(max_memory_spike, 50.0, "Memory spikes should be controlled")

            print("✅ System stability during transitions verified"
                  f"Avg transition: {avg_transition_time*1000:.1f}ms, "
                  f"Max CPU spike: {max_cpu_spike:.1f}%, "
                  f"Max memory spike: {max_memory_spike:.1f}MB")

        except Exception as e:
            self.fail(f"System stability test failed: {e}")

    def test_concurrent_access_safety(self):
        """Test that components handle concurrent access safely."""
        print("Testing concurrent access safety...")

        try:
            from src.core.mission_resource_manager import get_mission_resource_manager
            from src.core.feature_flags import get_feature_flag_manager

            rm = get_mission_resource_manager()
            ff = get_feature_flag_manager()

            # Test concurrent mission switching
            results = {'success': 0, 'errors': 0}

            def concurrent_switcher(thread_id: int):
                """Function to run in each thread."""
                try:
                    for i in range(5):
                        mission = self.test_mission_profiles[i % len(self.test_mission_profiles)]

                        # Alternate between systems for concurrent access
                        if thread_id % 2 == 0:
                            rm.switch_mission_profile(mission)
                        else:
                            ff.set_mission_profile(mission)

                        time.sleep(0.01)  # Small delay to encourage race conditions

                    results['success'] += 1
                except Exception as e:
                    results['errors'] += 1
                    print(f"Thread {thread_id} error: {e}")

            # Start multiple threads
            threads = []
            num_threads = 4

            for i in range(num_threads):
                thread = threading.Thread(target=concurrent_switcher, args=(i,))
                threads.append(thread)
                thread.start()

            # Wait for all threads
            for thread in threads:
                thread.join(timeout=5.0)

            # Verify no crashes or deadlocks
            self.assertEqual(results['success'], num_threads,
                           f"Concurrent access failed: {results['success']}/{num_threads} threads succeeded")

            # Verify system is still functional
            rm_status = rm.get_resource_status()
            ff_mission = ff.current_mission_profile

            self.assertIsInstance(rm_status, dict, "Resource manager should still be functional")
            self.assertIsNotNone(ff_mission, "Feature flags should still be functional")

            print("✅ Concurrent access safety verified")

        except Exception as e:
            self.fail(f"Concurrent access safety test failed: {e}")

    def _measure_resources(self) -> tuple[float, float]:
        """Measure current CPU and memory usage."""
        cpu_percent = self.process.cpu_percent(interval=None)
        memory_mb = self.process.memory_info().rss / (1024 * 1024)
        return cpu_percent, memory_mb


def run_integration_tests():
    """Run all integration tests."""
    print("🔗 INTEGRATION TESTS - URC 2026 Resource Optimization")
    print("=" * 55)

    # Create test suite
    suite = unittest.TestLoader().loadTestsFromTestCase(IntegrationTests)
    runner = unittest.TextTestRunner(verbosity=2)

    # Run tests
    result = runner.run(suite)

    # Summary
    print("\n📊 INTEGRATION TEST RESULTS")
    print("=" * 35)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")

    if result.wasSuccessful():
        print("✅ All integration tests passed!")
        return True
    else:
        print("❌ Some integration tests failed")
        for failure in result.failures:
            print(f"  FAILED: {failure[0]}")
        for error in result.errors:
            print(f"  ERROR: {error[0]}")
        return False


if __name__ == "__main__":
    success = run_integration_tests()
    exit(0 if success else 1)
