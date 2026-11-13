cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    meshes
    urdf
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/motion_planning/
  DESTINATION lib/${PROJECT_NAME}/motion_planning
  FILES_MATCHING
  PATTERN "*.py"
)

ament_package()
