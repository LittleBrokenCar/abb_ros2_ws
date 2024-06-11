from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import RegisterEventHandler, LogInfo

def generate_launch_description():

    turtle_back = DeclareLaunchArgument(name="turtle_back", default_value="turtle_back")
    turtle_left = DeclareLaunchArgument(name="turtle_left", default_value="turtle_left")
    turtle_right = DeclareLaunchArgument(name="turtle_right", default_value="turtle_right")
    
    turtle1 = Node(package="turtlesim", executable="turtlesim_node")
    turtle1_world = Node(package="py05_exercise", executable="exer02_tf_broadcaster_py", name="turtle1_world")

    spawn_back = Node(package="py05_exercise", executable="exer01_spawn_py",
                      name="spawn_back",
                      parameters=[{"x":1.5, "y":3.5, "turtle_name":LaunchConfiguration("turtle_back")}])
    turtle_back_world = Node(package="py05_exercise", executable="exer02_tf_broadcaster_py", name="turtle_back_world",
                             parameters=[{"turtle":LaunchConfiguration("turtle_back")}])
    escort_goal_back = Node(package="tf2_ros", executable="static_transform_publisher", name="escort_goal_back",
                            arguments=["--frame-id", "turtle1", "--child-frame-id", "escort_goal_back","--x","-1.5"])
    turtle_back_listener = Node(package="py05_exercise", executable="exer03_tf_listener_py", name="turtle_back_listener",
                                parameters=[{"father_frame":LaunchConfiguration("turtle_back"),"child_frame":"escort_goal_back"}])

    spawn_left = Node(package="py05_exercise", executable="exer01_spawn_py",
                      name="spawn_left",
                      parameters=[{"x":11.5, "y":10.5, "turtle_name":LaunchConfiguration("turtle_left")}])
    turtle_left_world = Node(package="py05_exercise", executable="exer02_tf_broadcaster_py", name="turtle_left_world",
                             parameters=[{"turtle":LaunchConfiguration("turtle_left")}])
    escort_goal_left = Node(package="tf2_ros", executable="static_transform_publisher", name="escort_goal_left",
                            arguments=["--frame-id", "turtle1", "--child-frame-id", "escort_goal_left","--x","-1.2", "--y","0.9"])
    turtle_left_listener = Node(package="py05_exercise", executable="exer03_tf_listener_py", name="turtle_left_listener",
                                parameters=[{"father_frame":LaunchConfiguration("turtle_left"),"child_frame":"escort_goal_left"}])

    spawn_right = Node(package="py05_exercise", executable="exer01_spawn_py",
                      name="spawn_right",
                      parameters=[{"x":7.5, "y":0.5, "turtle_name":LaunchConfiguration("turtle_right")}])
    turtle_right_world = Node(package="py05_exercise", executable="exer02_tf_broadcaster_py", name="turtle_right_world",
                             parameters=[{"turtle":LaunchConfiguration("turtle_right")}])
    escort_goal_right= Node(package="tf2_ros", executable="static_transform_publisher", name="escort_goal_right",
                            arguments=["--frame-id", "turtle1", "--child-frame-id", "escort_goal_right","--x","-1.2", "--y","-0.9"])
    turtle_right_listener = Node(package="py05_exercise", executable="exer03_tf_listener_py", name="turtle_right_listener",
                                parameters=[{"father_frame":LaunchConfiguration("turtle_right"),"child_frame":"escort_goal_right"}])


        
    return LaunchDescription([turtle_back, turtle_left, turtle_right,
                              turtle1, turtle1_world,
                              spawn_back, turtle_back_world, escort_goal_back,turtle_back_listener,
                              spawn_left, turtle_left_world, escort_goal_left,turtle_left_listener,
                              spawn_right, turtle_right_world, escort_goal_right,turtle_right_listener,
                              ])