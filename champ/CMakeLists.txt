
cmake_minimum_required(VERSION 3.5)
project(champ)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/)

ament_export_include_directories(include) 

ament_package()