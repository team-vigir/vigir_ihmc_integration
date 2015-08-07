cmake_minimum_required(VERSION 2.8.3)
project(vigir_ihmc_control_integration)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  sensor_msgs
  trajectory_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vigir_ihmc_control_integration
#  CATKIN_DEPENDS action_lib control_msgs roscpp sensor_msgs trajectory_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(ihmc_arm_action_node src/ihmc_arm_action_node.cpp src/ihmc_arm_action_server.cpp)
target_link_libraries(ihmc_arm_action_node ${catkin_LIBRARIES})

install(TARGETS ihmc_arm_action_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)