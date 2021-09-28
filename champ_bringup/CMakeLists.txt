cmake_minimum_required(VERSION 3.5)
project(champ_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(champ_description REQUIRED)
 
install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME})

ament_package()