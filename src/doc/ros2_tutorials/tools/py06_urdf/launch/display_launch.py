from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch.substitutions import Command

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import RegisterEventHandler, LogInfo

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_model = DeclareLaunchArgument(name="robot_model", default_value=get_package_share_directory("py06_urdf") + '/demo02_link.urdf')
    # ros2 launch py06_urdf display_launch.py model:='ros2 pkg prefix --share py06_urdf/<--->.urdf'
    p_value = ParameterValue(Command({"xacro ",LaunchConfiguration("robot_model")}))
    robot_state_pub=Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[{"robot_description":p_value}]
        )
    
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    rviz2=Node(package="rviz2", executable="rviz2")
            #    ,arguments=["-d", get_package_share_directory("py06_urdf") + '/default1.rviz'])    
    return LaunchDescription([robot_model,robot_state_pub,rviz2,joint_state_pub])