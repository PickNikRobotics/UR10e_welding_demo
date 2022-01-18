# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Denis Stogl

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
        )
    )
    # TODO(anyone): enable this when added into ROS2-foxy
    # choices=['ur3', 'ur3e', 'ur5', 'ur5e', 'ur10', 'ur10e', 'ur16e']))
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="yyy.yyyy.yyyy.yyyy",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ipa_demo_cell_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
      is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ipa_ur_demo.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ipa_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
      is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
      multi-robot setup. If changed than also joint names in the controllers' configuration \
      have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
          Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "workpiece_name",
            default_value="Workpiece_Demo_nominal",
            description="Folder of workpiece to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch RViz?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    # Demo Arguments
    workpiece_name = LaunchConfiguration("workpiece_name")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "calibrated_kinematics.yaml",
        ]
    )
    physical_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "physical_parameters.yaml",
        ]
    )
    visual_params = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "config",
            ur_type,
            "visual_parameters.yaml",
        ]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml("ipa_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Compose the path to the workpiece
    workpiece_path_param = PathJoinSubstitution(
        [
            FindPackageShare("ipa_demo_support"),
            "workpieces",
            workpiece_name,
            workpiece_name,
        ]
    )

    # Test Node
    test_request_node = Node(
        package="hybrid_planning_demo",
        executable="hybrid_planning_test_node",
        name="hybrid_planning_test_node",
        output="screen",
        parameters=[
            {"workpiece_path": workpiece_path_param},
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    nodes_to_start = [
        test_request_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
