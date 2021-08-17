cmake_minimum_required(VERSION 3.5)
project(ipa_bringup)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  control_msgs
  geometry_msgs
  rclcpp
)

add_executable(servo_keyboard_input src/keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    servo_keyboard_input
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
