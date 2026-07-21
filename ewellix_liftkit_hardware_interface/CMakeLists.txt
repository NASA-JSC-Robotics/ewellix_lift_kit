cmake_minimum_required(VERSION 3.8)
project(liftkit_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- ElmoController library ---
add_library(ElmoController SHARED
  src/ElmoController.cpp
)
target_include_directories(ElmoController PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Hardware interface plugin ---
add_library(elmo_liftkit_hardware_interface SHARED
  src/elmo_liftkit_hardware_interface.cpp
)
target_include_directories(elmo_liftkit_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(elmo_liftkit_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(elmo_liftkit_hardware_interface
  ElmoController
)

# --- Calibration executable ---
add_executable(elmo_calibration
  src/ElmoHoming.cpp
)
target_include_directories(elmo_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(elmo_calibration
  ElmoController
  pthread
)

# --- Sine Wave Tester executable (standalone, no ROS 2) ---
add_executable(sine_wave_tester
  test/main.cpp
)
target_include_directories(sine_wave_tester PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sine_wave_tester
  ElmoController
  yaml-cpp
  pthread
)

# --- Set RPATH for all executables ---
set_target_properties(elmo_liftkit_hardware_interface PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)
set_target_properties(elmo_calibration PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)
set_target_properties(sine_wave_tester PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# --- Install ALL targets ---
install(TARGETS
  ElmoController
  elmo_liftkit_hardware_interface
  elmo_calibration
  sine_wave_tester
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  elmo_liftkit_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- pluginlib export ---
pluginlib_export_plugin_description_file(
  hardware_interface
  elmo_liftkit_hardware_interface.xml
)

ament_export_include_directories(include)
ament_export_libraries(ElmoController elmo_liftkit_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle yaml-cpp)
ament_package()