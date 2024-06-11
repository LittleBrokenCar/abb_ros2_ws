from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import RegisterEventHandler, LogInfo


def generate_launch_description():
    
    t=Node(package='turtlesim', executable='turtlesim_node')
    
    spawn = ExecuteProcess(cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': 8.0,'y': 3.0}\""],
                           output="both",
                           shell=True)
    
    event_start = RegisterEventHandler(     # register event 1 
        event_handler=OnProcessStart(
            target_action=t,
            on_start=[spawn]))
    
    event_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=t,
            on_exit=[LogInfo(msg="turtlesim_node exit")]
        )
    )
    
    return LaunchDescription([t, event_start, event_exit])