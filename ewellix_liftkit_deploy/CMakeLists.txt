cmake_minimum_required(VERSION 3.8)
project(ewellix_liftkit_deploy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install configuration, launch, and rviz files
install(
  DIRECTORY config launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS scripts/test_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies so they're available to packages that depend on this one
ament_export_dependencies(ament_cmake_python rclpy)

ament_package()