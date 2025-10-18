# CogniPlan Testing

This directory contains all the tests for the CogniPlan ROS system.

## Test Structure

- `unit/` - Unit tests for individual components
- `integration/` - Integration tests for ROS node communication
- `system/` - System tests for complete navigation functionality
- `performance/` - Performance tests for response time and resource usage
- `data/` - Test data files

## Running Tests

### Generate Test Data

First, generate the test data:

```bash
cd ~/CLionProjects/CogniPlan
python3 test/generate_test_data.py
```

### Run Unit Tests

```bash
cd ~/CLionProjects/CogniPlan
python3 -m pytest test/unit/
```

Or run all unit tests:

```bash
cd ~/CLionProjects/CogniPlan
python3 test/run_tests.py
```

### Run Integration Tests

Integration tests require a ROS environment:

```bash
cd ~/CLionProjects/CogniPlan
rostest test/test_cogniplan.launch
```

### Run System Tests

System tests also require a ROS environment:

```bash
# In one terminal
roscore

# In another terminal
roslaunch cogniplan test_cogniplan.launch

# In a third terminal
rostest test/system/test_navigation.test
```

### Run Performance Tests

```bash
cd ~/CLionProjects/CogniPlan
python3 -m pytest test/performance/
```

## Test Descriptions

### Unit Tests

Unit tests verify the functionality of individual components:

1. Model loading and inference
2. Map processing utilities
3. Sensor data processing
4. Path planning algorithms

### Integration Tests

Integration tests verify that ROS nodes communicate correctly:

1. Topic subscriptions and publications
2. Service availability
3. Parameter configuration
4. Message passing between nodes

### System Tests

System tests verify complete navigation functionality:

1. End-to-end navigation from start to goal
2. Obstacle avoidance
3. Dynamic obstacle handling
4. Recovery behaviors

### Performance Tests

Performance tests measure system characteristics:

1. Response time for planning
2. CPU usage during operation
3. Memory usage during operation
4. Real-time performance constraints

## Test Data

Test data includes:

1. Sample maps in various formats
2. Simulated sensor data (LaserScan, Odometry)
3. Configuration files
4. Expected output data for verification

## Adding New Tests

To add new tests:

1. Create a new test file in the appropriate directory
2. Follow the naming convention `test_*.py`
3. Use the unittest framework
4. Add any required test data to the `data/` directory
5. Update the test runner if necessary