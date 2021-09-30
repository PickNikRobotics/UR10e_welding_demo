cmake_minimum_required(VERSION 3.5)
project(processit_program)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(processit_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  geometry_msgs
  interactive_markers
  moveit_ros_planning_interface
  # moveit_visual_tools
  processit_msgs
  std_srvs
  visualization_msgs
)

include_directories(
  include
)


# Add Marker plugin
add_executable(pose_marker
  src/pose_marker.cpp
)
ament_target_dependencies(pose_marker ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_include_directories(include)
ament_export_libraries(TaskDescription)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install header files
install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS
    pose_marker
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
