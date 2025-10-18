#!/usr/bin/env python3

"""
Test script to verify CogniPlan planner node imports correctly
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test that all required modules can be imported"""
    try:
        import rospy
        print("✓ rospy imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import rospy: {e}")
        return False
        
    try:
        import torch
        print("✓ torch imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import torch: {e}")
        return False
        
    try:
        import numpy as np
        print("✓ numpy imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import numpy: {e}")
        return False
        
    try:
        from nav_msgs.msg import OccupancyGrid
        print("✓ nav_msgs/OccupancyGrid imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import nav_msgs/OccupancyGrid: {e}")
        return False
        
    try:
        from sensor_msgs.msg import LaserScan
        print("✓ sensor_msgs/LaserScan imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import sensor_msgs/LaserScan: {e}")
        return False
        
    try:
        from geometry_msgs.msg import Twist
        print("✓ geometry_msgs/Twist imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import geometry_msgs/Twist: {e}")
        return False
        
    try:
        from planner.cogniplan_planner_node import CogniPlanPlannerNode
        print("✓ CogniPlanPlannerNode imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import CogniPlanPlannerNode: {e}")
        return False
        
    return True

if __name__ == "__main__":
    print("Testing CogniPlan planner node imports...")
    success = test_imports()
    
    if success:
        print("\n✓ All imports successful! The node should work correctly.")
    else:
        print("\n✗ Some imports failed. Please check your installation.")
        sys.exit(1)