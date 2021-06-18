cmake_minimum_required(VERSION 3.5)
project(hybrid_planning_demo)

find_package(ament_cmake REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_hybrid_planning
  moveit_servo
  rclcpp
)

add_executable(hybrid_planning_test_node src/hybrid_planning_test_node.cpp)
ament_target_dependencies(hybrid_planning_test_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(hybrid_planning_test_node ${LIBRARIES})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    hybrid_planning_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
