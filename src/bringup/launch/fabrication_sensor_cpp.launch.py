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

    tcpTolink6_yaml_path = LaunchConfiguration("tcpTolink6_yaml_path")
    tcpTolink6_yaml_path_cmd = DeclareLaunchArgument("tcpTolink6_yaml_path", default_value=
                                                  os.path.join(get_package_share_directory("abb_ros2_moveit_config"),"config","tcpTolink6.yaml"))
    
    planning_filename = LaunchConfiguration("planning_filename")
    planning_filename_cmd = DeclareLaunchArgument("planning_filename", default_value="path.txt")

    including_pose_estimation = LaunchConfiguration("including_pose_estimation")
    including_pose_estimation_cmd = DeclareLaunchArgument("including_pose_estimation", default_value="false")

    moveit_config = MoveItConfigsBuilder("abb_ros2").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="flow_sensor_fabrication_demo",
        package="fabrication_sensor_cpp",
        executable="flow_sensor_fabrication_demo",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            tcpTolink6_yaml_path,
            {"planning_filename": planning_filename,
             "including_pose_estimation": including_pose_estimation}
        ],
    )

    return LaunchDescription([
        tcpTolink6_yaml_path_cmd,
        planning_filename_cmd,
        including_pose_estimation_cmd,
        move_group_demo,
        ])

