#!/usr/bin/env python3

"""
Test data publisher for CogniPlan system testing
"""

import rospy
import numpy as np
import json
import os
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import tf

class TestDataProvider:
    def __init__(self):
        rospy.init_node('test_data_provider', anonymous=True)
        
        # Publishers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        # Load test data
        self.test_data_dir = os.path.join(os.path.dirname(__file__), 'data')
        self.load_test_data()
        
        # Timer for publishing data
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_test_data)
        
        rospy.loginfo("Test data provider initialized")
        
    def load_test_data(self):
        """Load test data from files"""
        try:
            # Load map data
            self.test_map = np.load(os.path.join(self.test_data_dir, 'test_map.npy'))
            
            # Load laser scan data
            with open(os.path.join(self.test_data_dir, 'test_scan.json'), 'r') as f:
                self.test_scan = json.load(f)
                
            # Load odometry data
            with open(os.path.join(self.test_data_dir, 'test_odom.json'), 'r') as f:
                self.test_odom = json.load(f)
                
            rospy.loginfo("Test data loaded successfully")
        except Exception as e:
            rospy.logerr(f"Failed to load test data: {e}")
            # Generate default test data
            self.generate_default_test_data()
            
    def generate_default_test_data(self):
        """Generate default test data if files are not available"""
        rospy.loginfo("Generating default test data")
        
        # Generate simple test map (10x10 meters, 0.1m resolution)
        self.test_map = np.zeros((100, 100), dtype=np.int8)
        self.test_map[0, :] = 100  # Top border
        self.test_map[-1, :] = 100  # Bottom border
        self.test_map[:, 0] = 100  # Left border
        self.test_map[:, -1] = 100  # Right border
        
        # Simple laser scan data
        self.test_scan = {
            'angle_min': -np.pi,
            'angle_max': np.pi,
            'angle_increment': np.pi/180,
            'range_min': 0.1,
            'range_max': 10.0,
            'ranges': [5.0] * 360
        }
        
        # Simple odometry data
        self.test_odom = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'linear_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}
        }
        
    def create_occupancy_grid_msg(self):
        """Create OccupancyGrid message from test map data"""
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.1
        grid.info.width = self.test_map.shape[1]
        grid.info.height = self.test_map.shape[0]
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = self.test_map.flatten().tolist()
        return grid
        
    def create_laser_scan_msg(self):
        """Create LaserScan message from test scan data"""
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "laser_frame"
        scan.angle_min = self.test_scan['angle_min']
        scan.angle_max = self.test_scan['angle_max']
        scan.angle_increment = self.test_scan['angle_increment']
        scan.range_min = self.test_scan['range_min']
        scan.range_max = self.test_scan['range_max']
        scan.ranges = self.test_scan['ranges']
        return scan
        
    def create_odometry_msg(self):
        """Create Odometry message from test odometry data"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.test_odom['position']['x']
        odom.pose.pose.position.y = self.test_odom['position']['y']
        odom.pose.pose.position.z = self.test_odom['position']['z']
        
        # Orientation
        odom.pose.pose.orientation.x = self.test_odom['orientation']['x']
        odom.pose.pose.orientation.y = self.test_odom['orientation']['y']
        odom.pose.pose.orientation.z = self.test_odom['orientation']['z']
        odom.pose.pose.orientation.w = self.test_odom['orientation']['w']
        
        # Velocity
        odom.twist.twist.linear.x = self.test_odom['linear_velocity']['x']
        odom.twist.twist.linear.y = self.test_odom['linear_velocity']['y']
        odom.twist.twist.linear.z = self.test_odom['linear_velocity']['z']
        odom.twist.twist.angular.x = self.test_odom['angular_velocity']['x']
        odom.twist.twist.angular.y = self.test_odom['angular_velocity']['y']
        odom.twist.twist.angular.z = self.test_odom['angular_velocity']['z']
        
        return odom
        
    def publish_test_data(self, event):
        """Publish all test data"""
        try:
            # Publish map
            map_msg = self.create_occupancy_grid_msg()
            self.map_pub.publish(map_msg)
            
            # Publish laser scan
            scan_msg = self.create_laser_scan_msg()
            self.scan_pub.publish(scan_msg)
            
            # Publish odometry
            odom_msg = self.create_odometry_msg()
            self.odom_pub.publish(odom_msg)
            
            rospy.logdebug("Published test data")
        except Exception as e:
            rospy.logerr(f"Failed to publish test data: {e}")

def main():
    try:
        provider = TestDataProvider()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()