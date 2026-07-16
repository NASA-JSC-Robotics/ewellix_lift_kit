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
install(PROGRAMS 
  scripts/test_publisher.py
  scripts/sine_wave_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create symbolic link for sine_wave_publisher executable name
install(CODE "
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    sine_wave_command_publisher.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/sine_wave_publisher
  )
")

# Export dependencies so they're available to packages that depend on this one
ament_export_dependencies(ament_cmake_python rclpy)

ament_package()