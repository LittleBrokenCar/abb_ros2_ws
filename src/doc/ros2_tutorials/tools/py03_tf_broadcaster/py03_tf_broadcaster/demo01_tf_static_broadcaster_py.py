import rclpy
from rclpy.node import Node
import sys
from rclpy.logging import get_logger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TFStaticBroadcasterPy(Node):
    def __init__(self, argv):
        super().__init__('tf_static_broadcaster_py')
        self.get_logger().info('tf_static_broadcaster_py has been established')

        self.broadcaster = StaticTransformBroadcaster(self)  # topic name /tf_static
        self.pub_static_tf(argv)

    def pub_static_tf(self, argv):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()  # set Time Stamp
        ts.header.frame_id = argv[7]
        ts.child_frame_id = argv[8]

        ts.transform.translation.x = float(argv[1])
        ts.transform.translation.y = float(argv[2])
        ts.transform.translation.z = float(argv[3])

        qtn=tf_transformations.quaternion_from_euler(
            float(argv[4]),
            float(argv[5]),
            float(argv[6]),
            )
        ts.transform.rotation.x = qtn[0]
        ts.transform.rotation.y = qtn[1]
        ts.transform.rotation.z = qtn[2]
        ts.transform.rotation.w = qtn[3]

        self.broadcaster.sendTransform(ts)



def main():
    if len(sys.argv) != 9:  # execu x y z r p y frame-id child-frame-id
        get_logger("rclpy").error("input is invalid")
        sys.exit()
    rclpy.init()

    rclpy.spin(TFStaticBroadcasterPy(sys.argv))

    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
