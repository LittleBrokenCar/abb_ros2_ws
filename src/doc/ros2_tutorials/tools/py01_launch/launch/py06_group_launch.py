from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    
    t1=Node(package='turtlesim', executable='turtlesim_node', name="t1")
    t2=Node(package='turtlesim', executable='turtlesim_node', name="t2")
    t3=Node(package='turtlesim', executable='turtlesim_node', name="t3")

    g1 = GroupAction(actions=[PushRosNamespace("g1"), t1, t2]) 
    g2 = GroupAction(actions=[PushRosNamespace("g2"), t3]) # g2/t3

    
    return LaunchDescription([g1, g2])