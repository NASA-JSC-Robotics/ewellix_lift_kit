cmake_minimum_required(VERSION 3.5)
project(ewellix_liftkit_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    launch
    meshes
    rviz
    urdf
    config
    DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
