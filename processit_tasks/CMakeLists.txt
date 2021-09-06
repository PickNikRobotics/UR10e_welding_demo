cmake_minimum_required(VERSION 3.5)
project(processit_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosparam_shortcuts REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  rosparam_shortcuts
  geometry_msgs
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  tf2_geometry_msgs
)

include_directories(
  include
)

add_library(processit_tasks SHARED
  src/cartesian_task.cpp
)
ament_target_dependencies(processit_tasks ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_include_directories(include)
ament_export_libraries(processit_tasks)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install header files
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    processit_tasks
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
