cmake_minimum_required(VERSION 3.8)
project(amr_safety)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts as executables
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/collision_warning_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME collision_warning_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
