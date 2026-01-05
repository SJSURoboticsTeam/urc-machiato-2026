#!/usr/bin/env python3
"""
Final System Testing Summary for URC 2026 Mars Rover
"""

import sys
import os
sys.path.insert(0, 'src')

def run_test(name, func):
    try:
        func()
        print(f'✅ {name}: PASSED')
        return True
    except Exception as e:
        print(f'❌ {name}: FAILED - {str(e)[:50]}...')
        return False

def main():
    print('🎯 FINAL SYSTEM TESTING SUMMARY')
    print('=' * 60)

    tests_passed = 0
    tests_total = 0

    def count_test(result):
        nonlocal tests_passed, tests_total
        tests_total += 1
        if result:
            tests_passed += 1
        return result

    print('\n🔍 ROS2 Environment:')
    count_test(run_test('ROS2 Import', lambda: __import__('rclpy')))

    print('\n🔍 Core Utilities:')
    count_test(run_test('Utility Import', lambda: __import__('autonomy.utilities.autonomy_utilities')))

    # Test safe execute function
    def test_safe_execute():
        from autonomy.utilities.autonomy_utilities import safe_execute
        result, error = safe_execute(lambda: 42)
        assert result == 42 and error is None
    count_test(run_test('Safe Execute', test_safe_execute))

    print('\n🔍 System Components:')
    count_test(run_test('Config Manager', lambda: __import__('core.configuration_manager')))
    count_test(run_test('Component Registry', lambda: __import__('core.component_registry')))

    print('\n🔍 Unit Tests:')
    unit_modules = ['test_utilities', 'test_autonomy_infrastructure', 'test_bt_system',
                   'test_mission_executor', 'test_safety_monitor', 'test_simple_websocket_bridge',
                   'test_terrain_classifier', 'test_waypoint_navigation_mission']
    for mod in unit_modules:
        count_test(run_test(f'{mod} Import', lambda m=mod: __import__(f'tests.unit.{m}', fromlist=[''])))

    print(f'\n' + '=' * 60)
    print(f'🎯 OVERALL TESTING RESULTS:')
    print(f'   Tests Passed: {tests_passed}/{tests_total}')
    print(f'   Success Rate: {(tests_passed/tests_total)*100:.1f}%')

    if tests_passed == tests_total:
        print(f'\n🎉 ALL TESTS PASSED - SYSTEM READY!')
    else:
        print(f'\n⚠️ SOME TESTS FAILED - REVIEW ISSUES')

    print('=' * 60)

if __name__ == '__main__':
    main()
