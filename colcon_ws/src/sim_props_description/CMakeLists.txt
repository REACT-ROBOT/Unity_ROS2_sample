cmake_minimum_required(VERSION 3.5)
project(sim_props_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf courses launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gen_tape_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
