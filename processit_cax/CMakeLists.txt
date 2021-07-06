cmake_minimum_required(VERSION 3.5)
project(processit_cax)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

include_directories(include ${EIGEN3_INCLUDE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS Eigen3)

# Add TaskDescription Library (non-ROS!)
add_library (TaskDescription SHARED
  include/TaskDescription/Contour.h
  include/TaskDescription/TaskDefinition.h
  include/TaskDescription/Templates.h
  include/TaskDescription/WeldSegment.h
  include/tinyxml/tinyxml.cpp
  include/tinyxml/tinyxml.h
  include/tinyxml/tinystr.cpp
  include/tinyxml/tinystr.h
  include/tinyxml/tinyxmlparser.cpp
  include/tinyxml/tinyxmlerror.cpp
  src/TaskDescription/Contour.cpp
  src/TaskDescription/TaskDefinition.cpp
  src/TaskDescription/WeldSegment.cpp
)
target_link_libraries(TaskDescription ${TinyXML_LIBS} Eigen3::Eigen)


ament_export_include_directories(include)
ament_export_libraries(TaskDescription)
# ament_export_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()