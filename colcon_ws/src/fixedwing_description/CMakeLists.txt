cmake_minimum_required(VERSION 3.5)
project(fixedwing_description)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots)

install(DIRECTORY ros2_control/
  DESTINATION share/${PROJECT_NAME}/ros2_control)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY simulation/
  DESTINATION share/${PROJECT_NAME}/simulation)

install(PROGRAMS scripts/generate_wing_stl.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
