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
find_package(rclcpp_components REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(processit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS Eigen3)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  visualization_msgs
  processit_msgs
  geometry_msgs
  moveit_ros_planning
  rviz_visual_tools
  Eigen3
)

include_directories(include)

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

# Add ROS wrapper for task description
add_executable(plugin_task_description
  # include/plugin_task_description.h
  src/plugin_task_description.cpp
)
ament_target_dependencies(plugin_task_description ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(plugin_task_description Eigen3::Eigen TaskDescription)

# Add Test node for task description plugin
add_executable(plugin_task_description_test_node src/plugin_task_description_test_node.cpp)
ament_target_dependencies(plugin_task_description_test_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_include_directories(include)
ament_export_libraries(TaskDescription)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    TaskDescription
    plugin_task_description
    plugin_task_description_test_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
