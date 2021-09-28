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
find_package(champ_msgs REQUIRED)
find_package(champ REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(urdf REQUIRED)


include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/ 
)

install(DIRECTORY config  
  DESTINATION share/${PROJECT_NAME}
)

set(dependencies
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
  sensor_msgs
  trajectory_msgs
  champ_msgs
  champ)

add_executable(message_relay src/message_relay.cpp)
ament_target_dependencies(message_relay ${dependencies})

add_executable(quadruped_controller src/quadruped_controller.cpp)
ament_target_dependencies(quadruped_controller ${dependencies})

install(TARGETS 
    quadruped_controller
    message_relay
    #state_estimation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)