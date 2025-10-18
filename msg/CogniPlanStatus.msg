# Status message for the CogniPlan system
std_msgs/Header header

# Planning status
string status            # "idle", "planning", "executing", "completed", "error"
string error_message     # Detailed error message if status is "error"

# Goal information
geometry_msgs/Pose goal_pose
bool goal_reached

# Current robot state
geometry_msgs/Pose current_pose
geometry_msgs/Twist current_velocity

# Planning information
float32 planning_time    # Time taken for last planning cycle (seconds)
int32 path_length        # Number of waypoints in current path