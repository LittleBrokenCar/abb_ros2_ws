from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    bg_r_launch_arg = DeclareLaunchArgument(name="bg_r", default_value="255") # default_value must be string
    bg_g_launch_arg = DeclareLaunchArgument(name="bg_g", default_value="255") # default_value must be string
    bg_b_launch_arg = DeclareLaunchArgument(name="bg_b", default_value="255") # default_value must be string
    
    t=Node(package='turtlesim', 
           executable='turtlesim_node',
           parameters=[{"background_r": LaunchConfiguration("bg_r"),
                        "background_g": LaunchConfiguration("bg_g"),
                        "background_b": LaunchConfiguration("bg_b")}])
    
    return LaunchDescription([bg_r_launch_arg, bg_g_launch_arg, bg_b_launch_arg, t])