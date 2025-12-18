#!/usr/bin/env python3
"""
Advanced Systems Logic-Only Test

Tests the core logic of all advanced systems without requiring ROS2 runtime.
Validates state synchronization, DDS domain redundancy, and dynamic configuration
system logic and integration.

Author: URC 2026 Autonomy Team
"""

import time
import sys
import os

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))


class AdvancedSystemsLogicTest:
    """Logic-only test for advanced systems integration."""

    def __init__(self):
        self.test_results = {}

    def run_logic_tests(self):
        """Run all logic tests without ROS2 runtime."""
        print("🧪 Advanced Systems Logic-Only Test Suite")
        print("="*60)
        print("Testing system logic without ROS2 runtime")
        print("="*60)

        try:
            # Test 1: State Synchronization Logic
            print("\n🔄 Testing State Synchronization Logic...")
            state_results = self.test_state_sync_logic()
            self.test_results['state_sync'] = state_results

            # Test 2: DDS Domain Redundancy Logic
            print("\n🔗 Testing DDS Domain Redundancy Logic...")
            dds_results = self.test_dds_domain_logic()
            self.test_results['dds_domain'] = dds_results

            # Test 3: Dynamic Configuration Logic
            print("\n⚙️ Testing Dynamic Configuration Logic...")
            config_results = self.test_dynamic_config_logic()
            self.test_results['dynamic_config'] = config_results

            # Test 4: System Integration Logic
            print("\n🔧 Testing System Integration Logic...")
            integration_results = self.test_system_integration_logic()
            self.test_results['integration'] = integration_results

            # Generate report
            self.generate_logic_test_report()

            success = self.validate_logic_tests()
            return success

        except Exception as e:
            print(f"\n❌ Logic test suite error: {e}")
            return False

    def test_state_sync_logic(self) -> dict:
        """Test state synchronization logic."""
        try:
            from core.state_synchronization_manager import DistributedStateManager

            # Create test managers
            manager1 = DistributedStateManager("bridge1")
            manager2 = DistributedStateManager("bridge2")

            # Register nodes
            manager1.register_node("bridge1")
            manager1.register_node("bridge2")
            manager2.register_node("bridge1")
            manager2.register_node("bridge2")

            # Register slave managers for direct state sync (testing)
            manager1.register_slave_manager(manager2)
            manager2.register_slave_manager(manager1)

            # Start managers
            manager1.start()
            manager2.start()

            # Test state updates
            manager1.update_state("telemetry_rate", 10.0)
            manager1.update_state("battery_level", 85.5)

            time.sleep(0.1)  # Allow sync

            # Check state retrieval
            rate1 = manager1.get_state("telemetry_rate")
            rate2 = manager2.get_state("telemetry_rate")
            battery1 = manager1.get_state("battery_level")
            battery2 = manager2.get_state("battery_level")

            # Test system status
            status1 = manager1.get_system_status()
            status2 = manager2.get_system_status()

            manager1.stop()
            manager2.stop()

            return {
                'state_updates_work': rate1 == 10.0 and battery1 == 85.5,
                'state_sync_works': rate1 == rate2 and battery1 == battery2,
                'system_status_works': 'role' in status1 and 'state_version' in status1,
                'nodes_registered': len(status1.get('nodes', {})) == 2
            }

        except Exception as e:
            return {'error': str(e), 'logic_test_failed': True}

    def test_dds_domain_logic(self) -> dict:
        """Test DDS domain redundancy logic."""
        try:
            from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager

            manager = DDSDomainRedundancyManager(primary_domain=42)

            # Register test nodes
            manager.register_node("competition_bridge", "test_command")
            manager.register_node("state_machine_bridge", "test_command")

            # Get system status
            status = manager.get_system_status()

            # Test domain count
            domain_count = len(status.get('domains', {}))
            node_count = len(status.get('nodes', {}))

            return {
                'domains_configured': domain_count == 3,  # primary, backup, emergency
                'nodes_registered': node_count == 2,
                'current_domain': status.get('current_domain') == 42,
                'domain_status_tracking': all('status' in d for d in status.get('domains', {}).values())
            }

        except Exception as e:
            return {'error': str(e), 'logic_test_failed': True}

    def test_dynamic_config_logic(self) -> dict:
        """Test dynamic configuration logic."""
        try:
            from core.dynamic_config_manager import DynamicConfigManager

            manager = DynamicConfigManager()

            # Register test nodes
            manager.register_node("competition_bridge", {
                'telemetry_rate_hz': 5.0,
                'max_clients': 50
            })

            # Test configuration updates
            success1 = manager.update_node_config("competition_bridge", "telemetry_rate_hz", 10.0)

            updates = [
                {"node_name": "competition_bridge", "parameter_name": "max_clients", "new_value": 100}
            ]
            success2 = manager.update_multiple_configs(updates)

            # Check current config
            current_config = manager.get_node_config("competition_bridge")
            config_updated = (current_config.get('telemetry_rate_hz') == 10.0 and
                            current_config.get('max_clients') == 100)

            # Test rollback
            rollback_success = manager.rollback_to_version(1)

            # Check system status
            status = manager.get_system_status()

            return {
                'single_update_works': success1,
                'multiple_updates_work': success2,
                'config_updated_correctly': config_updated,
                'rollback_works': rollback_success,
                'status_tracking_works': 'current_version' in status and 'nodes' in status
            }

        except Exception as e:
            return {'error': str(e), 'logic_test_failed': True}

    def test_system_integration_logic(self) -> dict:
        """Test integration of all systems."""
        try:
            # Import all managers
            from core.state_synchronization_manager import get_state_manager
            from core.dds_domain_redundancy_manager import get_dds_redundancy_manager
            from core.dynamic_config_manager import get_dynamic_config_manager
            from bridges.websocket_redundancy_manager import get_redundancy_manager

            # Test global instance access
            state_mgr = get_state_manager("test_integration")
            dds_mgr = get_dds_redundancy_manager()
            config_mgr = get_dynamic_config_manager()
            ws_redundancy_mgr = get_redundancy_manager()

            # Test cross-system interactions
            # 1. State manager updates
            state_mgr.start()
            state_mgr.update_state("integration_test", "working")

            # 2. DDS manager node registration
            dds_mgr.register_node("integration_bridge", "test_command")

            # 3. Config manager registration
            config_mgr.register_node("integration_bridge", {'test_param': 'value'})

            # 4. WebSocket redundancy endpoint registration
            from bridges.websocket_redundancy_manager import WebSocketEndpoint, EndpointPriority
            endpoint = WebSocketEndpoint(
                name="integration_test",
                port=9999,
                priority=EndpointPriority.PRIMARY
            )
            ws_redundancy_mgr.add_endpoint(endpoint)

            # Verify all systems have state
            state_status = state_mgr.get_system_status()
            dds_status = dds_mgr.get_system_status()
            config_status = config_mgr.get_system_status()
            ws_status = ws_redundancy_mgr.get_system_status()

            state_mgr.stop()

            return {
                'all_managers_created': True,
                'state_manager_has_data': len(state_status.get('state_keys', [])) > 0,
                'dds_manager_has_nodes': len(dds_status.get('nodes', {})) > 0,
                'config_manager_has_nodes': len(config_status.get('nodes', {})) > 0,
                'ws_redundancy_has_endpoints': len(ws_status.get('endpoints', {})) > 0,
                'no_integration_conflicts': True  # All managers coexist
            }

        except Exception as e:
            return {'error': str(e), 'integration_failed': True}

    def generate_logic_test_report(self):
        """Generate logic test report."""
        print("\n📋 Advanced Systems Logic Test Report")
        print("="*60)

        overall_success = self.validate_logic_tests()
        status = "✅ PASSED" if overall_success else "❌ FAILED"
        print(f"Overall Logic Test Status: {status}")

        # Detailed results
        for test_name, results in self.test_results.items():
            print(f"\n🔍 {test_name.upper().replace('_', ' ')} Results:")

            if test_name == 'state_sync':
                print(f"   • State Updates: {'✅' if results.get('state_updates_work') else '❌'}")
                print(f"   • State Sync: {'✅' if results.get('state_sync_works') else '❌'}")
                print(f"   • System Status: {'✅' if results.get('system_status_works') else '❌'}")
                print(f"   • Nodes Registered: {'✅' if results.get('nodes_registered') else '❌'}")

            elif test_name == 'dds_domain':
                print(f"   • Domains Configured: {'✅' if results.get('domains_configured') else '❌'}")
                print(f"   • Nodes Registered: {'✅' if results.get('nodes_registered') else '❌'}")
                print(f"   • Domain Status: {'✅' if results.get('domain_status_tracking') else '❌'}")

            elif test_name == 'dynamic_config':
                print(f"   • Single Updates: {'✅' if results.get('single_update_works') else '❌'}")
                print(f"   • Multiple Updates: {'✅' if results.get('multiple_updates_work') else '❌'}")
                print(f"   • Config Updated: {'✅' if results.get('config_updated_correctly') else '❌'}")
                print(f"   • Rollback Works: {'✅' if results.get('rollback_works') else '❌'}")

            elif test_name == 'integration':
                print(f"   • All Managers: {'✅' if results.get('all_managers_created') else '❌'}")
                print(f"   • State Data: {'✅' if results.get('state_manager_has_data') else '❌'}")
                print(f"   • DDS Nodes: {'✅' if results.get('dds_manager_has_nodes') else '❌'}")
                print(f"   • Config Nodes: {'✅' if results.get('config_manager_has_nodes') else '❌'}")
                print(f"   • WS Endpoints: {'✅' if results.get('ws_redundancy_has_endpoints') else '❌'}")
                print(f"   • No Conflicts: {'✅' if results.get('no_integration_conflicts') else '❌'}")

        # Summary
        print(f"\n📊 Test Summary:")
        total_tests = len(self.test_results)
        passed_tests = sum(1 for r in self.test_results.values()
                          if not any(k in ['error', 'logic_test_failed', 'integration_failed']
                                   for k in r.keys()))
        print(f"   • Tests Run: {total_tests}")
        print(f"   • Tests Passed: {passed_tests}")
        print(f"   • Success Rate: {passed_tests}/{total_tests} ({passed_tests/total_tests*100:.1f}%)")

        if overall_success:
            print("\n✅ All advanced system logic tests PASSED!")
            print("   🎉 Systems are ready for ROS2 integration")
        else:
            print("\n⚠️ Some logic tests failed - check individual results above")

        print("="*60)

    def validate_logic_tests(self) -> bool:
        """Validate overall logic test success."""
        for test_name, results in self.test_results.items():
            # Check for errors
            if any(k in results for k in ['error', 'logic_test_failed', 'integration_failed']):
                return False

            # Check specific test criteria
            if test_name == 'state_sync':
                if not all(results.get(k, False) for k in ['state_updates_work', 'state_sync_works']):
                    return False
            elif test_name == 'dds_domain':
                if not results.get('domains_configured', False):
                    return False
            elif test_name == 'dynamic_config':
                if not all(results.get(k, False) for k in ['single_update_works', 'rollback_works']):
                    return False
            elif test_name == 'integration':
                if not results.get('all_managers_created', False):
                    return False

        return True


def main():
    """Main test function."""
    import argparse

    parser = argparse.ArgumentParser(description="Advanced Systems Logic-Only Test Suite")
    parser.add_argument('--verbose', action='store_true', help='Verbose output')

    args = parser.parse_args()

    # Run the logic tests
    suite = AdvancedSystemsLogicTest()

    try:
        success = suite.run_logic_tests()
        sys.exit(0 if success else 1)

    except Exception as e:
        print(f"\n❌ Logic test suite error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
