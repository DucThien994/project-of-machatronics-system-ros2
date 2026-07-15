cmake_minimum_required(VERSION 3.8)
project(amr_navigation)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
endif()

# Tool test pose goal (nav2_random_pose_test.py) - chay duoc qua:
#   ros2 run amr_navigation nav2_random_pose_test --samples 200
install(PROGRAMS
  scripts/nav2_random_pose_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_random_pose_test
)

ament_package()
