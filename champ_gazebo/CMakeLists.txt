cmake_minimum_required(VERSION 3.5)
project(champ_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}) 

set(dependencies
  rclcpp  
  std_msgs
  std_srvs
  tf2_ros
  geometry_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros)

install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
