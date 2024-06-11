import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    handeye_tf_service_node = Node(
        package='handeye_tf_service', 
        executable='handeye_tf_server',
        output='screen')
    rqt_config = os.path.join(get_package_share_directory('handeye_dashboard'), 'config', 'Default.perspective')
    rqt_handeye_dashboard_node= Node(
        package='rqt_gui', 
        executable='rqt_gui',
        output='screen', 
        arguments=['--perspective-file', rqt_config])
    
    rviz_config = os.path.join(
        get_package_share_directory('handeye_target_detection'), 
        'cfg', 'handeye.rviz'
    )

    yaml_config = os.path.join(
        get_package_share_directory('handeye_target_detection'), 
        'launch', 'pose_estimation.yaml'
    )

    cameraToobject_calibration_node= Node(
        package='handeye_target_detection', 
        executable='pose_estimation', 
        output='screen', 
        parameters=[yaml_config])

    rviz2_cameraToobject_calibration_node=Node(
        package='rviz2', 
        executable='rviz2', 
        output='screen', 
        arguments=['-d', rviz_config])
    
    return LaunchDescription( [
        handeye_tf_service_node,
        rqt_handeye_dashboard_node,
        rviz2_cameraToobject_calibration_node,
        cameraToobject_calibration_node
    ])
