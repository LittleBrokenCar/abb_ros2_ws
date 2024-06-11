import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # params
    rws_ip = LaunchConfiguration("rws_ip")
    rws_ip_cmd = DeclareLaunchArgument("rws_ip", default_value="None", description='IP for RWS computer')

    rws_port = LaunchConfiguration("rws_port")
    rws_port_cmd = DeclareLaunchArgument("rws_port", default_value="80", description="Port at which RWS can be found.")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_fake_hardware_cmd = DeclareLaunchArgument("use_fake_hardware", default_value="true",
                                                  description="label for connecting with physical controller")
    
    load_workspace_model = LaunchConfiguration("load_workspace_model")
    load_workspace_model_cmd = DeclareLaunchArgument("load_workspace_model",default_value="false",
                                                     description="label for loading workspace model")

    model = LaunchConfiguration("model")
    model_cmd = DeclareLaunchArgument("model", default_value="yolov8m.pt", description="Model name or path")

    device = LaunchConfiguration("device")
    device_cmd = DeclareLaunchArgument("device", default_value="cuda:0", description="Device to use (GPU/CPU)")

    threshold = LaunchConfiguration("threshold")
    threshold_cmd = DeclareLaunchArgument("threshold", default_value="0.5",
                                          description="Minimum probability of a detection to be published")

    canvas_width = LaunchConfiguration("canvas_width")
    canvas_width_cmd = DeclareLaunchArgument("canvas_width", default_value="640", description="width of the canvas")

    canvas_height = LaunchConfiguration("canvas_height")
    canvas_height_cmd = DeclareLaunchArgument("canvas_height", default_value="480", description="height of the canvas")

    frame_rate = LaunchConfiguration("frame_rate")
    frame_rate_cmd = DeclareLaunchArgument("frame_rate", default_value="30",
                                           description="the frequency of image refreshment")
    
    link6Tocamera_yaml_path = LaunchConfiguration("link6Tocamera_yaml_path")
    link6Tocamera_yaml_path_cmd = DeclareLaunchArgument("link6Tocamera_yaml_path", default_value=
                                                  os.path.join(get_package_share_directory("abb_ros2_moveit_config"),"config","link6Tocamera.yaml"))

    cameraToobject_yaml_path = LaunchConfiguration("cameraToobject_yaml_path")
    cameraToobject_yaml_path_cmd = DeclareLaunchArgument("cameraToobject_yaml_path", default_value=
                                                  os.path.join(get_package_share_directory("abb_ros2_moveit_config"),"config","cameraToobject.yaml"))


    offsets_yaml_path = LaunchConfiguration("offsets_yaml_path")
    offsets_yaml_path_cmd = DeclareLaunchArgument("offsets_yaml_path", default_value=
                                                  os.path.join(get_package_share_directory("abb_ros2_moveit_config"),"config","offsets.yaml"))
    


    # Moveit configurations
    moveit_config = (
        MoveItConfigsBuilder("abb_ros2")
        .robot_description(
            file_path="config/abb_irb1600_10_12.urdf.xacro",
            mappings={
                "rws_ip": rws_ip,
                "rws_port": rws_port,
                "use_fake_hardware": use_fake_hardware,
                "load_workspace_model": load_workspace_model
            },
        )
        .robot_description_semantic(file_path="config/abb_irb1600_10_12.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"])
        .to_moveit_configs()
    )

    # Nodes for Controllers Initialization
    ros2_controllers_path = os.path.join(
        get_package_share_directory("abb_ros2_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # Nodes for Moveit Initialization
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Nodes for Rviz Initialization
    rviz_config = os.path.join(get_package_share_directory("abb_ros2_moveit_config"), "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Nodes for Vision Initialization
    vision_node = Node(
        package="yolo_realsense_calibration",
        executable="yolo_realsense",
        # IfCondition
        name="yolo_realsense_node",
        output="both",
        parameters=[{"model": model,
                     "device": device,
                     "threshold": threshold,
                     "canvas_width": canvas_width,
                     "canvas_height": canvas_height,
                     "frame_rate": frame_rate,
                     }],
        # remappings=[("raw_sub",new_sub),("raw_pub",new_pub)]
    )

    eye_in_hand_node = Node(
        package="yolo_realsense_calibration",
        executable="eye_in_hand",
        name="Link6ToCamera_static_frame_node",
        output="both",
        parameters=[link6Tocamera_yaml_path],
    )

    object_in_eye_node = Node(
        package="yolo_realsense_calibration",
        executable="object_in_eye",
        name="CameraToObject_static_frame_node",
        output="both",
        parameters=[cameraToobject_yaml_path],
    )

    substrate_location_node = Node(
        package="yolo_realsense_calibration",
        executable="substrate_location",
        name="substrate_location_node",
        output="both",
        parameters=[offsets_yaml_path]
        # remappings=[("raw_sub",new_sub),("raw_pub",new_pub)]
    )

    return LaunchDescription(
        [
            rws_ip_cmd,
            rws_port_cmd,
            use_fake_hardware_cmd,
            load_workspace_model_cmd,
            model_cmd,
            device_cmd,
            threshold_cmd,
            canvas_height_cmd,
            canvas_width_cmd,
            frame_rate_cmd,
            link6Tocamera_yaml_path_cmd,
            cameraToobject_yaml_path_cmd,
            offsets_yaml_path_cmd,
            
            ros2_control_node,
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner,

            rviz_node,
            static_tf_node,
            robot_state_publisher,
            move_group_node,

            # vision_node,
            eye_in_hand_node,
            object_in_eye_node,
            substrate_location_node,

        ]
    )
