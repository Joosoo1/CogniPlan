cmake_minimum_required(VERSION 3.0.2)
project(cogniplan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  CogniPlanStatus.msg
)

add_service_files(
  FILES
  SetGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  planner/cogniplan_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY checkpoints/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/checkpoints
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  
  add_rostest(test/test_cogniplan.launch)
  
  catkin_install_python(PROGRAMS
    test/test_data_publisher.py
    test/run_tests.py
    test/generate_test_data.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
  install(DIRECTORY test/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
  )
endif()