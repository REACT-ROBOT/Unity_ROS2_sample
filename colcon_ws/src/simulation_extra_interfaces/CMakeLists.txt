cmake_minimum_required(VERSION 3.8)
project(simulation_extra_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ContactRecord.msg"
  "srv/ApplyLinkWrench.srv"
  "srv/GetContactEvents.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
