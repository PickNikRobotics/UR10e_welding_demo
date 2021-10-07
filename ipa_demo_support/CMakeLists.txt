cmake_minimum_required(VERSION 3.5)
project(ipa_demo_support)

find_package(ament_cmake REQUIRED)

install(DIRECTORY workpieces
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
