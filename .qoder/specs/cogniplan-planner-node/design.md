# CogniPlan Planner Node - Design Document

## Overview
This document describes the design of the `cogniplan_planner_node`, a ROS node that integrates the trained CogniPlan model into a robotic system. The node loads the trained PolicyNet and QNet neural networks implemented in PyTorch and interfaces with ROS topics for input data (sensor and map information) to generate motion commands based on the CogniPlan approach.

The node subscribes to occupancy grid maps and sensor data, processes this information into the format required by the CogniPlan model, runs inference to generate navigation decisions, and publishes motion commands for the robot to follow.

## Technical Architecture
The architecture follows a standard ROS node pattern with clearly defined responsibilities:

1. **ROS Interface Layer**: Handles subscription to ROS topics and publication of commands
2. **Data Processing Layer**: Converts ROS messages to CogniPlan model input format
3. **Model Inference Layer**: Loads and executes the trained CogniPlan model
4. **Command Generation Layer**: Translates model outputs to robot motion commands
5. **Configuration Layer**: Manages parameters and model loading

The implementation uses Python to match the existing CogniPlan codebase, with full integration including actions, services, and dynamic reconfigure capabilities. The node supports GPU acceleration when available and implements multi-threaded execution for better responsiveness.

## Component Design

### 1. CogniPlanPlannerNode Class
Main node class that orchestrates all functionality:

#### Key Responsibilities:
- Initialize ROS interfaces (publishers, subscribers, services, actions)
- Load and manage the trained CogniPlan model
- Process incoming data and trigger planning
- Generate and publish motion commands
- Handle parameter configuration and dynamic reconfigure

#### Key Methods:
- `__init__`: Node initialization and ROS interface setup
- `map_callback`: Process incoming occupancy grid maps
- `sensor_callback`: Process sensor data inputs
- `plan`: Execute the planning algorithm using the CogniPlan model
- `publish_commands`: Publish generated motion commands

### 2. ModelManager Class
Handles loading and execution of the trained CogniPlan model:

#### Key Responsibilities:
- Load PolicyNet and QNet models from checkpoint files
- Manage model state and device placement (CPU/GPU)
- Execute model inference on processed input data
- Handle model versioning and reloading

#### Key Methods:
- `load_model`: Load model from checkpoint file
- `infer`: Run model inference on input tensors
- `to_device`: Move model to appropriate compute device

### 3. DataProcessor Class
Converts ROS messages to CogniPlan model input format:

#### Key Responsibilities:
- Convert nav_msgs/OccupancyGrid to node-based representation
- Process sensor data into feature vectors
- Generate graph structures for model input
- Handle coordinate transformations and normalization

#### Key Methods:
- `process_map`: Convert occupancy grid to node features
- `process_sensor_data`: Extract features from sensor inputs
- `create_graph`: Generate node graph with connectivity information
- `normalize_inputs`: Normalize data for model consumption

### 4. CommandGenerator Class
Translates model outputs to robot motion commands:

#### Key Responsibilities:
- Convert model action probabilities to motion commands
- Apply safety constraints and limits
- Generate Twist messages for robot control
- Handle emergency situations and fallback behaviors

#### Key Methods:
- `generate_twist`: Create geometry_msgs/Twist from model output
- `apply_constraints`: Apply velocity and acceleration limits
- `handle_emergency`: Implement emergency stop procedures
- `validate_command`: Verify command safety

## Data Models

### ROS Message Types

#### Input Messages:
- `/map` [nav_msgs/OccupancyGrid] - Occupancy grid map
- `/scan` [sensor_msgs/LaserScan] - 2D laser scan data
- `/points` [sensor_msgs/PointCloud2] - 3D point cloud data
- `/odom` [nav_msgs/Odometry] - Robot odometry

#### Output Messages:
- `/cmd_vel` [geometry_msgs/Twist] - Velocity commands
- `/planning/path` [nav_msgs/Path] - Planned path (optional)
- `/planning/status` [std_msgs/String] - Planning status information

### Internal Data Structures

#### Node Representation:
```python
{
    'coords': [x, y],                    # Node coordinates
    'utility': float,                    # Node utility value
    'frontiers': [[x, y], ...],         # Observable frontiers
    'visited': bool,                     # Visited status
    'neighbors': [node_id, ...]         # Connected nodes
}
```

#### Model Input Tensors:
```python
{
    'node_inputs': torch.Tensor,         # Node features (batch_size, node_count, features)
    'node_padding_mask': torch.Tensor,   # Padding mask for variable node counts
    'edge_mask': torch.Tensor,           # Adjacency matrix for graph connectivity
    'current_index': torch.Tensor,       # Index of current node
    'current_edge': torch.Tensor,        # Available actions from current node
    'edge_padding_mask': torch.Tensor    # Padding mask for variable action counts
}
```

## API Specifications

### Parameters

#### Model Parameters:
- `model_path` (string, default: "checkpoints/cogniplan_exp_pred7_test/checkpoint.pth") - Path to trained model checkpoint
- `use_gpu` (bool, default: true) - Enable GPU acceleration if available
- `node_input_dim` (int, default: 6) - Dimension of node input features
- `embedding_dim` (int, default: 128) - Dimension of node embeddings

#### Planning Parameters:
- `planning_frequency` (double, default: 10.0) - Planning rate in Hz
- `sensor_range` (double, default: 16.0) - Sensor range in meters
- `node_resolution` (double, default: 3.6) - Node resolution in meters
- `utility_range` (double, default: 12.8) - Utility calculation range

#### Control Parameters:
- `max_linear_velocity` (double, default: 1.0) - Maximum linear velocity (m/s)
- `max_angular_velocity` (double, default: 1.0) - Maximum angular velocity (rad/s)
- `safety_distance` (double, default: 0.5) - Minimum distance from obstacles

### Topics

#### Subscribed Topics:
- `/map` [nav_msgs/OccupancyGrid] - Current occupancy grid map
- `/scan` [sensor_msgs/LaserScan] - Laser scan data
- `/points` [sensor_msgs/PointCloud2] - Point cloud data
- `/odom` [nav_msgs/Odometry] - Robot odometry

#### Published Topics:
- `/cmd_vel` [geometry_msgs/Twist] - Velocity commands to robot
- `/planning/path` [nav_msgs/Path] - Generated path for visualization
- `/planning/status` [std_msgs/String] - Planning status information

### Services
- `/planning/set_goal` [cogniplan_msgs/SetGoal] - Set navigation goal
- `/planning/cancel_goal` [std_srvs/Empty] - Cancel current navigation
- `/planning/get_plan` [nav_msgs/GetPlan] - Get current plan

### Actions
- `/navigate_to_pose` [move_base_msgs/MoveBaseAction] - High-level navigation action

## Error Handling
The system implements comprehensive error handling at multiple levels:

1. **Data Validation**:
   - Input message validation and sanitization
   - Coordinate transformation error checking
   - Map data consistency verification

2. **Model Errors**:
   - Model loading failure handling
   - Inference error detection and recovery
   - Invalid output validation and filtering

3. **Planning Failures**:
   - No valid path detection
   - Obstacle collision prevention
   - Fallback to safe behaviors

4. **System Errors**:
   - ROS communication error handling
   - Resource allocation failures
   - Graceful degradation strategies

Recovery behaviors include:
- Emergency stop for critical failures
- Replanning with alternative strategies
- Returning to safe zones
- Requesting human intervention for persistent failures

## Testing Strategy
The testing approach includes multiple layers:

1. **Unit Testing**:
   - Individual component functionality verification
   - Model loading and inference validation
   - Data processing and conversion tests
   - Command generation and constraint application

2. **Integration Testing**:
   - ROS topic subscription and publication validation
   - Service and action interface testing
   - Parameter configuration and dynamic reconfigure
   - Multi-node communication verification

3. **System Testing**:
   - End-to-end planning and control validation
   - Performance benchmarking under various conditions
   - Stress testing with simulated sensor failures
   - Safety constraint verification

4. **Simulation Testing**:
   - Gazebo-based robot simulation
   - Various environment and obstacle configurations
   - Automated test scenario execution
   - Regression testing for model updates

## Implementation Notes
1. The node follows ROS best practices for naming, parameterization, and lifecycle management
2. Uses TF2 for coordinate transformations with proper frame hierarchy
3. Implements proper logging and diagnostic publishing for system monitoring
4. Designed to be as stateless as possible to enable easier testing and debugging
5. Uses dynamic reconfiguration for tuning parameters during operation
6. Considers computational requirements for real-time performance
7. Implements proper shutdown procedures to ensure graceful termination
8. Supports both CPU and GPU execution with automatic fallback
9. Handles variable-sized input data through padding mechanisms
10. Integrates with existing CogniPlan codebase for minimal duplication