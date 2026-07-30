cmake_minimum_required(VERSION 3.5)
project(servo_demo_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots)

install(DIRECTORY simulation/
  DESTINATION share/${PROJECT_NAME}/simulation)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/servo_demo_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
