#!/usr/bin/env python3

"""
Main test runner for CogniPlan system
"""

import unittest
import sys
import os

# Add the project root and test directories to the path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

def run_all_tests():
    """Run all test suites"""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add unit tests
    try:
        from test.unit.test_components import TestPolicyModel, TestMapUtils, TestUtils
        suite.addTests(loader.loadTestsFromTestCase(TestPolicyModel))
        suite.addTests(loader.loadTestsFromTestCase(TestMapUtils))
        suite.addTests(loader.loadTestsFromTestCase(TestUtils))
    except ImportError as e:
        print(f"Warning: Could not load unit tests: {e}")
    
    # Add integration tests (these require ROS environment)
    # These would typically be run separately with rostest
    
    # Add system tests (these require ROS environment)
    # These would typically be run separately with rostest
    
    # Add performance tests
    try:
        from test.performance.test_performance import TestSystemPerformance
        suite.addTests(loader.loadTestsFromTestCase(TestSystemPerformance))
    except ImportError as e:
        print(f"Warning: Could not load performance tests: {e}")
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Return exit code based on test results
    return 0 if result.wasSuccessful() else 1

if __name__ == '__main__':
    exit_code = run_all_tests()
    sys.exit(exit_code)