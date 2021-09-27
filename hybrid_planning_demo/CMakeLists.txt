cmake_minimum_required(VERSION 3.5)
project(hybrid_planning_demo)

find_package(ament_cmake REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(processit_msgs REQUIRED)
find_package(processit_tasks REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_hybrid_planning
  moveit_servo
  moveit_task_constructor_core
  pluginlib
  processit_msgs
  processit_tasks
  rclcpp
  rclcpp_action
  rclcpp_components
)

include_directories(include)

add_executable(hybrid_planning_test_node src/hybrid_planning_test_node.cpp)
ament_target_dependencies(hybrid_planning_test_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(global_mtc_planner_component SHARED src/global_mtc_planner.cpp)
set_target_properties(global_mtc_planner_component PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(global_mtc_planner_component ${THIS_PACKAGE_INCLUDE_DEPENDS})
# rclcpp_components_register_nodes(global_mtc_planner_component "hybrid_planning_demo::GlobalMTCPlannerComponent")

add_library(servo_local_solver SHARED src/servo_solver.cpp)
set_target_properties(servo_local_solver PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(servo_local_solver ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_library(servo_sampler SHARED src/simple_servo_sampler.cpp)
set_target_properties(servo_sampler PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(servo_sampler ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    hybrid_planning_test_node
    global_mtc_planner_component
    servo_local_solver
    servo_sampler
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(moveit_hybrid_planning plugins.xml)

ament_export_libraries(
  global_mtc_planner_component
  servo_local_solver
  servo_sampler
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
