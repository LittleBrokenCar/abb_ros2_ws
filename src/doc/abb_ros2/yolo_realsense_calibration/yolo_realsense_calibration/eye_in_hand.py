import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
# import tf_transformations # FAIL TO fIND


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('Link6ToCamera_static_frame_node')

        # params
        self.declare_parameter("l6Tc_translation", [0.0,0.0,0.0])
        self.translation = self.get_parameter(
            "l6Tc_translation").get_parameter_value().double_array_value
        
        self.declare_parameter("l6Tc_quaternion", [0.0,0.0,0.0,1.0])
        self.quaternion = self.get_parameter(
            "l6Tc_quaternion").get_parameter_value().double_array_value

        # broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

        # initialization
        
        self.broadcast_tf()

    def broadcast_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_6'
        t.child_frame_id = 'camera_frame'

        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]

        t.transform.rotation.x = self.quaternion[0]
        t.transform.rotation.y = self.quaternion[1]
        t.transform.rotation.z = self.quaternion[2]
        t.transform.rotation.w = self.quaternion[3]

        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

