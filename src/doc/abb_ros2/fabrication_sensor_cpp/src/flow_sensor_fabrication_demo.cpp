#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

# include<iostream>
# include<fstream>
# include<sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("node name: flow_sensor_fabrication_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("flow_sensor_fabrication_demo", node_options);


  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "abb_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  // Start the demo
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to initialize the workspace");

  double tcpTl6_x,tcpTl6_y,tcpTl6_z, tcpTl6_qx,tcpTl6_qy,tcpTl6_qz,tcpTl6_qw;

  move_group_node->get_parameter("tcpTl6_x",tcpTl6_x);
  move_group_node->get_parameter("tcpTl6_y",tcpTl6_y);
  move_group_node->get_parameter("tcpTl6_z",tcpTl6_z);
  move_group_node->get_parameter("tcpTl6_qx",tcpTl6_qx);
  move_group_node->get_parameter("tcpTl6_qy",tcpTl6_qy);
  move_group_node->get_parameter("tcpTl6_qz",tcpTl6_qz);
  move_group_node->get_parameter("tcpTl6_qw",tcpTl6_qw);

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(move_group_node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  geometry_msgs::msg::TransformStamped transformstamped_bTs;
  transformstamped_bTs = tf_buffer->lookupTransform("base_link", "substrate_frame", tf2::TimePointZero, tf2::durationFromSec(10.0));
  tf2::Vector3 translation_bTs(transformstamped_bTs.transform.translation.x,transformstamped_bTs.transform.translation.y,transformstamped_bTs.transform.translation.z);
  tf2::Quaternion quaternion_bTs(transformstamped_bTs.transform.rotation.x,transformstamped_bTs.transform.rotation.y,transformstamped_bTs.transform.rotation.z,transformstamped_bTs.transform.rotation.w);
  tf2::Transform transform_bTs(quaternion_bTs, translation_bTs);

  tf2::Vector3 translation_tcpTl6(tcpTl6_x,tcpTl6_y,tcpTl6_z);
  tf2::Quaternion quaternion_tcpTl6(tcpTl6_qx,tcpTl6_qy,tcpTl6_qz,tcpTl6_qw);
  tf2::Transform transform_tcpTl6(quaternion_tcpTl6, translation_tcpTl6);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  std::vector<geometry_msgs::msg::Pose> waypoints, visual_waypoints;

  // geometry_msgs::msg::Pose start_pose;
  // start_pose.orientation.w = 1.0;
  // start_pose.position.x = bTs_x+tcpTl6_x;  // meter
  // start_pose.position.y = bTs_y+tcpTl6_y;  // meter
  // start_pose.position.z = bTs_z+tcpTl6_z;  // meter
  // waypoints.push_back(start_pose); 
  // geometry_msgs::msg::Pose target_pose = start_pose;

  geometry_msgs::msg::Pose target_pose, visual_pose;

  std::string planning_filename;
  move_group_node->get_parameter("planning_filename", planning_filename);
  bool including_pose_estimation;
  move_group_node->get_parameter("including_pose_estimation", including_pose_estimation);
  std::string planning_filepath = ament_index_cpp::get_package_share_directory("abb_ros2_moveit_config") + "/config/plan/" + planning_filename ; 
  // RCLCPP_INFO(LOGGER, "planning_filepath %s", planning_filepath.c_str());
  std::ifstream inputFile(planning_filepath); 
  if (!inputFile.is_open()) {
      RCLCPP_INFO(LOGGER, "Fail to load file");
      return 1;
    }
  std::string line;
  while (std::getline(inputFile, line)) {
      std::istringstream iss(line);
      tf2::Quaternion q;
      double x, y, z,q_x,q_y,q_z, q_w;
      char dummy;
      iss >> dummy >> x;
      iss >> dummy >> y;
      iss >> dummy >> z;
      iss >> dummy >> q_x;
      iss >> dummy >> q_y;
      iss >> dummy >> q_z;
      iss >> dummy >> q_w;
      tf2::Vector3 translation_sTtcp(x, y, z);
      tf2::Quaternion quaternion_sTtcp;
      if (including_pose_estimation){
        quaternion_sTtcp = tf2::Quaternion(q_x, q_y, q_z, q_w); // with pose estimation
      } 
      else{
        quaternion_sTtcp=quaternion_bTs.inverse(); // without pose estimation
      }
      tf2::Transform transform_sTtcp(quaternion_sTtcp, translation_sTtcp);
      tf2::Transform transform_bTtcp = transform_bTs * transform_sTtcp;
      tf2::Transform transform_bTl6 = transform_bTtcp * transform_tcpTl6;

      target_pose.position.x = transform_bTl6.getOrigin().x();
      target_pose.position.y = transform_bTl6.getOrigin().y();
      target_pose.position.z = transform_bTl6.getOrigin().z();
      target_pose.orientation.x = transform_bTl6.getRotation().x();
      target_pose.orientation.y = transform_bTl6.getRotation().y();
      target_pose.orientation.z = transform_bTl6.getRotation().z();
      target_pose.orientation.w = transform_bTl6.getRotation().w();
      // q.setRPY(roll_x, pitch_y, yaw_z);   # method 2
      // q.setValue(0.0,0.0,0.0,1.0);
      // target_pose.orientation = tf2::toMsg(q);
      waypoints.push_back(target_pose);

      visual_pose.position.x = transform_bTtcp.getOrigin().x();
      visual_pose.position.y = transform_bTtcp.getOrigin().y();
      visual_pose.position.z = transform_bTtcp.getOrigin().z();
      visual_pose.orientation.w = 1.0;
      visual_waypoints.push_back(visual_pose);
  }
  inputFile.close();  

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  RCLCPP_INFO(LOGGER, "Visualizing plan Cartesian path (%.2f%% achieved)", fraction * 100.0);

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian_Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.publishPath(visual_waypoints, rvt::RED, rvt::SMALL);
  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to fabricate the sensor");
  // API_set_laser_on()
  move_group.execute(trajectory); // execute the plan
  // API_set_laser_on()
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  rclcpp::shutdown();
  return 0;
}
