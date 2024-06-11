import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    A = LaunchConfiguration("A")
    A_cmd = DeclareLaunchArgument("A", default_value=
                                                  os.path.join(get_package_share_directory("PKG_A"),"A","A.yaml"))
    
    

    # MoveGroupInterface demo executable
    A = Node(
        name="A_name",
        package="A_pkg",
        executable="A_file",
        output="screen",
        parameters=[
            A,
            {"A": A}
        ],
    )

    return LaunchDescription([
        # A,
        ])

