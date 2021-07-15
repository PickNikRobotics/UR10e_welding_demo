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


def generate_launch_description():
    declared_arguments = []

    # Declare Arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "folder_name",
            default_value="Workpiece_Demo_nominal",
            description="Folder of workpiece to load",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "scene_file_name",
            default_value="Workpiece_Demo_nominal.scene",
            description="Name of scene to load",
        )
    )

    # Initialize Arguments
    folder_name = LaunchConfiguration("folder_name")
    scene_file_name = LaunchConfiguration("scene_file_name")

    scene_file_param = PathJoinSubstitution(
        [
            FindPackageShare("ipa_demo_support"),
            "workpieces",
            folder_name,
            scene_file_name,
        ]
    )

    # Start the actual move_group node/action server
    moveit_publish_scene_from_text = Node(
        package="moveit_ros_planning",
        executable="moveit_publish_scene_from_text",
        output="screen",
        arguments=[scene_file_param],
        parameters=[
            {"scene": scene_file_param},
        ],
    )

    plugin_task_description_test_node = Node(
        package="processit_cax",
        executable="plugin_task_description_test_node",
        output="screen",
        parameters=[{"scene": scene_file_param}, {"test": True}],
    )

    nodes_to_start = [
        moveit_publish_scene_from_text,
        plugin_task_description_test_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
