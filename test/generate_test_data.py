#!/usr/bin/env python3

"""
Test data generation for CogniPlan system
"""

import numpy as np
import yaml
import json
import os

def generate_test_map(width=100, height=100):
    """Generate a test map with obstacles and free space"""
    # Create an empty map
    test_map = np.zeros((height, width), dtype=np.int8)
    
    # Add borders
    test_map[0, :] = 100  # Top border
    test_map[-1, :] = 100  # Bottom border
    test_map[:, 0] = 100  # Left border
    test_map[:, -1] = 100  # Right border
    
    # Add some obstacles
    # Central obstacle
    test_map[40:60, 40:60] = 100
    
    # Horizontal wall with gap
    test_map[30, 10:45] = 100
    test_map[30, 55:90] = 100
    
    # Vertical wall with gap
    test_map[10:45, 70] = 100
    test_map[55:90, 70] = 100
    
    return test_map

def generate_test_laser_scan(num_readings=360):
    """Generate a test laser scan"""
    # Create angle array
    angles = np.linspace(0, 2*np.pi, num_readings, endpoint=False)
    
    # Generate ranges with some obstacles
    ranges = np.full(num_readings, 10.0)  # 10m default range
    
    # Add some obstacles at specific angles
    # Front obstacle
    front_indices = np.where((angles > 0) & (angles < np.pi/4))[0]
    ranges[front_indices] = 2.0  # 2m in front
    
    # Right obstacle
    right_indices = np.where((angles > np.pi/2) & (angles < np.pi))[0]
    ranges[right_indices] = 3.0  # 3m on right
    
    return {
        'angle_min': 0.0,
        'angle_max': 2*np.pi,
        'angle_increment': 2*np.pi/num_readings,
        'range_min': 0.1,
        'range_max': 20.0,
        'ranges': ranges.tolist()
    }

def generate_test_odometry():
    """Generate test odometry data"""
    return {
        'position': {'x': 1.0, 'y': 1.0, 'z': 0.0},
        'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        'linear_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    }

def generate_test_config():
    """Generate test configuration"""
    return {
        'planner': {
            'update_frequency': 10.0,
            'goal_tolerance': 0.2,
            'robot_radius': 0.3
        },
        'sensor': {
            'laser_topic': '/scan',
            'odom_topic': '/odom',
            'map_topic': '/map'
        },
        'debug': {
            'publish_visualizations': True,
            'log_level': 'INFO'
        }
    }

def save_test_data():
    """Save all test data to files"""
    # Create test data directory if it doesn't exist
    test_data_dir = os.path.join(os.path.dirname(__file__), 'data')
    os.makedirs(test_data_dir, exist_ok=True)
    
    # Generate and save test map
    test_map = generate_test_map()
    np.save(os.path.join(test_data_dir, 'test_map.npy'), test_map)
    
    # Generate and save test laser scan
    test_scan = generate_test_laser_scan()
    with open(os.path.join(test_data_dir, 'test_scan.json'), 'w') as f:
        json.dump(test_scan, f)
    
    # Generate and save test odometry
    test_odom = generate_test_odometry()
    with open(os.path.join(test_data_dir, 'test_odom.json'), 'w') as f:
        json.dump(test_odom, f)
    
    # Generate and save test config
    test_config = generate_test_config()
    with open(os.path.join(test_data_dir, 'test_config.yaml'), 'w') as f:
        yaml.dump(test_config, f)
    
    print("Test data generated and saved successfully!")

if __name__ == '__main__':
    save_test_data()