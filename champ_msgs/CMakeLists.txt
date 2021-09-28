cmake_minimum_required(VERSION 3.5)
project(champ_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Contacts.msg"
  "msg/ContactsStamped.msg"
  "msg/Imu.msg"
  "msg/Joints.msg"
  "msg/PID.msg"
  "msg/Point.msg"
  "msg/PointArray.msg"
  "msg/Pose.msg"
  "msg/Velocities.msg"
  DEPENDENCIES 
    rosidl_default_generators
    builtin_interfaces 
    std_msgs
    sensor_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
