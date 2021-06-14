cmake_minimum_required(VERSION 3.5)
project(ipa_demo_cell_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY calibration launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
