cmake_minimum_required(VERSION 3.8)
project(liftkit_hardware_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(serial REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  rcutils
  std_msgs
  std_srvs
  control_toolbox
  serial
  )

add_library(${PROJECT_NAME} SHARED
  src/liftkit_hardware_interface.cpp
  src/serial_com_tlt.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_toolbox
  rcpputils
  rcutils
  serial
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JOINT_STATE_BROADCASTER_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(hardware_interface liftkit_hardware_interface.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_executable(controller_stopper_node src/controller_stopper.cpp src/controller_stopper_node.cpp)
target_include_directories(controller_stopper_node PUBLIC include)
ament_target_dependencies(controller_stopper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(test_liftkit test/test_liftkit_hardware_interface.cpp src/serial_com_tlt.cpp)
target_include_directories(test_liftkit PUBLIC include)
ament_target_dependencies(test_liftkit ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

install(
  TARGETS controller_stopper_node test_liftkit
  DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  rcutils
  serial
)

ament_package()
