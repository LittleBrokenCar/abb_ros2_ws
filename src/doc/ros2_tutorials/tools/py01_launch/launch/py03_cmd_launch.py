from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    
    t=Node(package='turtlesim', 
           executable='turtlesim_node', 
    )
    cmd = ExecuteProcess(
        # cmd=["ros2 topic echo /turtle1/pose"], # method 1
        # cmd=["ros2 topic", "echo", "/turtle1/pose"], # method 2
        cmd=[FindExecutable(name="ros2"),"topic", "echo", "/turtle1/pose"], # method 3
        output="both",
        shell=True
        )
    
    return LaunchDescription([t, cmd])