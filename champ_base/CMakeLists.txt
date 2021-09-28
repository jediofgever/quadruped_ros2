cmake_minimum_required(VERSION 3.5)
project(champ_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  include 
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(dependencies
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs)

add_library(quadruped_controller src/quadruped_controller.cpp)
add_library(message_relay src/message_relay.cpp)
add_library(state_estimation src/state_estimation.cpp)

add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
ament_target_dependencies(quadruped_controller_node ${dependencies})
ament_target_dependencies(quadruped_controller ${dependencies})

add_executable(message_relay_node src/message_relay_node.cpp)
ament_target_dependencies(message_relay_node ${dependencies})
ament_target_dependencies(message_relay ${dependencies})

add_executable(state_estimation_node src/state_estimation_node.cpp)
ament_target_dependencies(state_estimation_node ${dependencies})
ament_target_dependencies(state_estimation ${dependencies})

install(TARGETS 
    quadruped_controller
    quadruped_controller_node
    message_relay
    message_relay_node
    state_estimation
    state_estimation_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)