# CogniPlan ROS Deployment Architecture - Design Document

## Overview
This document describes the ROS deployment architecture for the CogniPlan system, which integrates with a real robot equipped with a 3D LiDAR. The system includes sensor data integration, map management, path planning using the trained CogniPlan model, robot control and navigation, and visualization capabilities.

The architecture is designed for ROS 1 Noetic, targeting a differential drive robot operating in mixed indoor/outdoor environments with a 3D LiDAR sensor. The system uses an end-to-end learning-based planning approach where the CogniPlan model directly outputs commands, with Foxglove Studio for visualization.

## Technical Architecture
The architecture follows a distributed node-based approach with clearly defined responsibilities for each component. The system is organized around these core modules:

1. **Sensor Interface Layer**: Handles raw sensor data acquisition and preprocessing
2. **Mapping Layer**: Manages map creation, storage, and updates
3. **Planning Layer**: Implements the CogniPlan model for path generation
4. **Control Layer**: Translates planned paths into robot motion commands
5. **Monitoring Layer**: Provides visualization and system monitoring capabilities

Communication between nodes follows ROS best practices using topics for continuous data streams, services for request-response interactions, and actions for long-running tasks with feedback.

## Component Design

### 1. Sensor Data Integration Node (`lidar_interface_node`)
Handles 3D LiDAR data acquisition and preprocessing:
- Interfaces with physical 3D LiDAR hardware
- Performs basic filtering and noise reduction
- Converts raw sensor data to standardized formats
- Publishes processed sensor data for other nodes

### 2. Map Management Node (`map_manager_node`)
Responsible for map creation, storage, and dynamic updates:
- Maintains occupancy grid maps
- Integrates SLAM for map building and localization
- Handles dynamic obstacle updates
- Provides map services for other components

### 3. CogniPlan Model Node (`cogniplan_planner_node`)
Implements the trained CogniPlan model for path planning:
- Loads and executes the trained planning model
- Receives sensor and map data as inputs
- Generates motion commands based on model predictions
- Provides planning services and publishes command outputs

### 4. Robot Control Node (`robot_controller_node`)
Translates high-level commands into low-level motor controls:
- Implements differential drive kinematics
- Handles velocity smoothing and acceleration limits
- Interfaces with robot hardware drivers
- Monitors robot status and safety conditions

### 5. Navigation Coordinator Node (`navigation_coordinator_node`)
Orchestrates the overall navigation workflow:
- Coordinates between mapping, planning, and control components
- Manages navigation goals and state transitions
- Implements recovery behaviors for failure cases
- Tracks navigation progress and completion

### 6. Visualization Node (`visualization_node`)
Provides system monitoring and debugging interfaces:
- Integrates with Foxglove Studio for visualization
- Publishes diagnostic information
- Provides interactive markers for goal setting
- Displays system status and performance metrics

## Data Models

### Map Data Structure
```
# OccupancyGrid message extension
Header header
MapMetaData info
int8[] data
# Additional fields for dynamic obstacles
geometry_msgs/PoseStamped[] dynamic_obstacles
float32[] obstacle_velocities
```

### Sensor Data Format
```
# PointCloud2 with additional semantic information
std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
# CogniPlan-specific annotations
sensor_msgs/PointCloud2Annotation[] annotations
```

### Planning Command Structure
```
# Custom motion command message
std_msgs/Header header
geometry_msgs/TwistStamped cmd_vel
geometry_msgs/PoseStamped target_pose
float32[] waypoints_x
float32[] waypoints_y
float32 confidence_score
string planning_status
```

## API Specifications

### Topics

#### Sensor Data Topics
- `/lidar/points` [sensor_msgs/PointCloud2] - Raw 3D LiDAR point cloud data
- `/lidar/filtered_points` [sensor_msgs/PointCloud2] - Filtered and processed LiDAR data
- `/imu/data` [sensor_msgs/Imu] - Inertial measurement unit data
- `/odom` [nav_msgs/Odometry] - Odometry information from robot

#### Map Topics
- `/map` [nav_msgs/OccupancyGrid] - Current static map
- `/map_updates` [map_msgs/OccupancyGridUpdate] - Incremental map updates
- `/dynamic_obstacles` [visualization_msgs/MarkerArray] - Dynamic obstacles in environment

#### Planning Topics
- `/planning/commands` [cogniplan_msgs/MotionCommand] - Motion commands from CogniPlan
- `/planning/path` [nav_msgs/Path] - Planned path for visualization
- `/planning/goal` [geometry_msgs/PoseStamped] - Navigation goal

#### Control Topics
- `/cmd_vel` [geometry_msgs/Twist] - Velocity commands to robot base
- `/joint_states` [sensor_msgs/JointState] - Robot joint states
- `/battery_state` [sensor_msgs/BatteryState] - Battery information

#### Monitoring Topics
- `/diagnostics` [diagnostic_msgs/DiagnosticArray] - System diagnostics
- `/tf` [tf2_msgs/TFMessage] - Coordinate frame transformations
- `/visualization_marker` [visualization_msgs/Marker] - Visualization markers

### Services

#### Map Services
- `/map_service/load_map` [nav_msgs/GetMap] - Load a specific map
- `/map_service/save_map` [std_srvs/Empty] - Save current map to file
- `/map_service/clear_map` [std_srvs/Empty] - Clear current map

#### Planning Services
- `/planning/set_goal` [cogniplan_msgs/SetGoal] - Set navigation goal
- `/planning/cancel_goal` [std_srvs/Empty] - Cancel current navigation
- `/planning/get_plan` [nav_msgs/GetPlan] - Get plan between two points

#### System Services
- `/system/start_navigation` [std_srvs/Empty] - Start navigation system
- `/system/stop_navigation` [std_srvs/Empty] - Stop navigation system
- `/system/reset` [std_srvs/Empty] - Reset system state

### Actions

#### Navigation Action
- `/navigate_to_pose` [move_base_msgs/MoveBaseAction] - High-level navigation action
  - Goal: move_base_msgs/MoveBaseGoal
  - Result: move_base_msgs/MoveBaseResult
  - Feedback: move_base_msgs/MoveBaseFeedback

#### Mapping Action
- `/build_map` [map_msgs/BuildMapAction] - Map building action
  - Goal: map_msgs/BuildMapGoal
  - Result: map_msgs/BuildMapResult
  - Feedback: map_msgs/BuildMapFeedback

## Error Handling
The system implements multiple levels of error handling:

1. **Sensor Failures**: 
   - Timeout detection for missing sensor data
   - Fallback to last known good data with degradation warnings
   - Automatic sensor restart attempts

2. **Planning Failures**:
   - Model prediction validation and outlier rejection
   - Fallback to traditional planning algorithms
   - Emergency stop procedures for unsafe commands

3. **Control Failures**:
   - Command timeout monitoring
   - Hardware fault detection
   - Safe shutdown procedures

4. **Communication Errors**:
   - Message loss detection and recovery
   - Network partition handling
   - Graceful degradation of non-critical components

Recovery behaviors include:
- Replanning with alternative strategies
- Returning to safe zones
- Requesting human intervention for critical failures

## Testing Strategy
The testing approach includes multiple layers:

1. **Unit Testing**:
   - Individual node functionality verification
   - Message serialization/deserialization tests
   - Algorithm validation with synthetic data

2. **Integration Testing**:
   - Node communication and data flow validation
   - Sensor-to-control pipeline testing
   - Recovery behavior verification

3. **System Testing**:
   - End-to-end navigation scenario testing
   - Performance benchmarking under various conditions
   - Stress testing with simulated sensor failures

4. **Simulation Testing**:
   - Gazebo-based robot simulation
   - Various environment and obstacle configurations
   - Automated test scenario execution

## Implementation Notes
1. All nodes should follow ROS best practices for naming, parameterization, and lifecycle management
2. Use TF2 for coordinate transformations with proper frame hierarchy
3. Implement proper logging and diagnostic publishing for system monitoring
4. Design nodes to be as stateless as possible to enable easier testing and debugging
5. Use dynamic reconfiguration for tuning parameters during operation
6. Consider computational requirements when deploying nodes across multiple machines
7. Implement proper shutdown procedures to ensure graceful termination of all components