cmake_minimum_required(VERSION 2.8.3)
project(teleop_set_velocity)

find_package(catkin REQUIRED COMPONENTS
  gecko_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   gecko_msgs#   sensor_msgs#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleop_set_velocity
#  CATKIN_DEPENDS gecko_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_library(teleop_set_velocity
#   src/${PROJECT_NAME}/teleop_set_velocity.cpp
# )

# add_dependencies(teleop_set_velocity ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(teleop_set_velocity_node src/teleop_set_velocity_node.cpp)

# add_dependencies(teleop_set_velocity_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(teleop_set_velocity_node
  ${catkin_LIBRARIES}
)
