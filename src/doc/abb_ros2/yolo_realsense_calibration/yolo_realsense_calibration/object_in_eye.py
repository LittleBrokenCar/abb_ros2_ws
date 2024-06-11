import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation


class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('CameraToObject_static_frame_node')

        # params
        self.declare_parameter("cTo_translation", [0.0,0.0,0.0])
        self.translation = self.get_parameter(
            "cTo_translation").get_parameter_value().double_array_value
        
        self.declare_parameter("cTo_quaternion", [0.0,0.0,0.0,1.0]) # x y z w
        self.quaternion = self.get_parameter(
            "cTo_quaternion").get_parameter_value().double_array_value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)

        # broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

        # initialization
        self.broadcast_tf()

    def broadcast_tf(self):
        future = self.tf_buffer.wait_for_transform_async("base_link", "camera_frame", rclpy.time.Time())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done() and future.result() is not None:
            transform_b2c = self.tf_buffer.lookup_transform("base_link", "camera_frame", rclpy.time.Time())
            translation_b2c = np.array([transform_b2c.transform.translation.x,transform_b2c.transform.translation.y,transform_b2c.transform.translation.z])
            quaternion_b2c=np.array([transform_b2c.transform.rotation.x,transform_b2c.transform.rotation.y,
                                     transform_b2c.transform.rotation.z,transform_b2c.transform.rotation.w])
            
            # print(f"b2c:{translation_b2c,quaternion_b2c}")
            
            translation_c2o = np.array([self.translation[0],self.translation[1],self.translation[2]])
            quaternion_c2o = np.array([self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]]) 

            # print(f"c2o:{translation_c2o,quaternion_c2o}")

            translation_b2o = Rotation.from_quat(quaternion_b2c).apply(translation_c2o) + translation_b2c # r=Pq+p
            quaternion_b2o = Rotation.from_quat(quaternion_b2c) * Rotation.from_quat(quaternion_c2o) # R=PQ
            quaternion_b2o = quaternion_b2o.as_quat()
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'object_frame'

            t.transform.translation.x = translation_b2o[0]  
            t.transform.translation.y = translation_b2o[1]
            t.transform.translation.z = translation_b2o[2]

            t.transform.rotation.x = quaternion_b2o[0]
            t.transform.rotation.y = quaternion_b2o[1]
            t.transform.rotation.z = quaternion_b2o[2]
            t.transform.rotation.w = quaternion_b2o[3]

            self.broadcaster.sendTransform(t)

        else:
            self.get_logger().error('Transformation failed or timeout')


def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

