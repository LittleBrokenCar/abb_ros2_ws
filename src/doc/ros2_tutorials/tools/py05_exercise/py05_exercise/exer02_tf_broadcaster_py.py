import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TFDynamicBroadcasterPy(Node):
    def __init__(self):
        super().__init__('tf_dynamic_broad_caster_py')
        self.get_logger().info('tf_dynamic_broad_caster_py has been established')
        
        self.declare_parameter("turtle","turtle1")
        self.turtle = self.get_parameter("turtle").get_parameter_value().string_value

        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Pose, '/'+self.turtle+'/pose' ,self.pose_callback, 10)
    
    def pose_callback(self, pose):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "world"
        ts.child_frame_id = self.turtle

        ts.transform.translation.x = pose.x
        ts.transform.translation.y = pose.y
        ts.transform.translation.z = 0.0

        qtn = tf_transformations.quaternion_from_euler(0.0,0.0,pose.theta)
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]
        
        self.broadcaster.sendTransform(ts)

def main():
    rclpy.init()

    rclpy.spin(TFDynamicBroadcasterPy())

    rclpy.shutdown()

if __name__ == '__main__':
    main()