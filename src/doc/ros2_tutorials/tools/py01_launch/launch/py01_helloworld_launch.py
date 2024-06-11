from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    t1 = Node(package='turtlesim', executable='turtlesim_node', namespace='t1', name='my_turtle')
    t2 = Node(package='turtlesim', executable='turtlesim_node', namespace='t2', name='my_turtle')

    return LaunchDescription([t1, t2])
