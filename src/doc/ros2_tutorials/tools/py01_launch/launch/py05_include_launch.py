from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    t=IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(launch_file_path=
                                                                os.path.join(get_package_share_directory("py01_launch"),
                                                                             "py04_args_launch.py")),
        launch_arguments=[("bg_r", "80"),("bg_g", "100"),("bg_b", "200")] 
        )
    
    return LaunchDescription([t])