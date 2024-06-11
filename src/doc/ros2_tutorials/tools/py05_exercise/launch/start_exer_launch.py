from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


x = DeclareLaunchArgument(name="x", default_value="1.0") # default_value must be string
y = DeclareLaunchArgument(name="y", default_value="3.0") # default_value must be string
name = DeclareLaunchArgument(name="turtle_name", default_value="turtle2") # default_value must be string
turtle = DeclareLaunchArgument(name="turtle", default_value="turtle2") # default_value must be string
    

def generate_launch_description():
    
    t=Node(package='turtlesim', executable='turtlesim_node')

    t2=Node(package='py05_exercise', executable='exer01_spawn_py', name="spawn1",
            parameters=[{"x": LaunchConfiguration("x"),
                         "y": LaunchConfiguration("y"),
                         "turtle_name": LaunchConfiguration("turtle_name"),
                         }])
    
    tf1=Node(package='py05_exercise', executable='exer02_tf_broadcaster_py', name="bro1")

    tf2=Node(package='py05_exercise', executable='exer02_tf_broadcaster_py', name="bro2",
             parameters=[{"turtle": LaunchConfiguration("turtle")
                         }])
    
    listener = Node(package="py05_exercise", executable="exer03_tf_listener_py",
                    parameters=[{"father_frame": LaunchConfiguration("turtle"), "child_frame":"turtle1"}]
                    )
    
    return LaunchDescription([x,y,name,turtle,t, t2, tf1,tf2, listener])