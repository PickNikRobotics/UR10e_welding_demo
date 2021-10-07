cmake_minimum_required(VERSION 3.5)
project(processit_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/WeldSeam.msg"
)
set(srv_files
  "srv/AddPoseMarker.srv"
  "srv/EditPoseMarker.srv"
  "srv/LoadTaskDescription.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
