from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    """
        :param: executable 
            the name of the executable to find if a package
            is provided or otherwise a path to the executable to run.
        :param: package 
            the package in which the node executable can be found
        :param: name 
            the name of the node
        :param: namespace 
            the ROS namespace for this Node
        :param: exec_name 
            the label used to represent the process.
            Defaults to the basename of node executable.
        :param: parameters 
            list of names of yaml files with parameter rules,
            or dictionaries of parameters.
        :param: remappings 
            ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: respawn 
            restart the node after closing it
        :param: ros_arguments 
            list of ROS arguments for the node
        :param: arguments 
            list of extra arguments for the node
        Using `ros_arguments` is equivalent to using `arguments` with a prepended '--ros-args' item.
        """
    
    # t=Node(package='turtlesim', 
    #        executable='turtlesim_node', 
    #        exec_name="my_label",
    #        ros_arguments=["--remap", "__ns:=/t2"])  # equal to "ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/t2"


    t=Node(package='turtlesim', 
           executable='turtlesim_node', 
           name="haha",
           respawn=True,  # keep running
        #    parameters=[{"background_r": 255, "background_g": 0, "background_b": 0}]   # method 1 
        #    parameters=["/home/maple/Documents/ws02_tools/install/py01_launch/share/py01_launch/haha.yaml"] # method 2
           parameters=[os.path.join(get_package_share_directory("py01_launch"),"haha.yaml")] # method 3 suggest dynamic
           )
    return LaunchDescription([t])