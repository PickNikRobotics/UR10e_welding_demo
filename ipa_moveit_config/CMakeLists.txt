cmake_minimum_required(VERSION 3.5)
project(ipa_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
