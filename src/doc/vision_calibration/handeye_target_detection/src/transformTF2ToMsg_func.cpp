#include "PoseEstimator.h"
 // the function "transformTF2ToMsg" has been removed in the ros-iron version, and the code stored in this file pursuant to https://docs.ros.org/en/lunar/api/tf2/html/buffer__core_8cpp_source.html  ---- 2024/03/13 by Qi'ao
 namespace tf2
{
void transformMsgToTF2(const geometry_msgs::msg::Transform& msg, tf2::Transform& tf2)
 {tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}
 

void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::TransformStamped& msg,
                       builtin_interfaces::msg::Time stamp, const std::string& frame_id,
                       const std::string& child_frame_id)
                       {
			transformTF2ToMsg(tf2, msg.transform);
			msg.header.stamp = stamp;
			msg.header.frame_id = frame_id;
			msg.child_frame_id = child_frame_id;
                       }
                       
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::Transform& msg)
			{
			msg.translation.x = tf2.getOrigin().x();
			msg.translation.y = tf2.getOrigin().y();
			msg.translation.z = tf2.getOrigin().z();
			msg.rotation.x = tf2.getRotation().x();
			msg.rotation.y = tf2.getRotation().y();
			msg.rotation.z = tf2.getRotation().z();
			msg.rotation.w = tf2.getRotation().w();
			}
			}
