import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class TFPointBroadcasterPy(Node):
    def __init__(self):
        super().__init__('tf_point_broadcaster_py')
        self.get_logger().info('tf_point_broadcaster_py has been established')

        self.pub = self.create_publisher(PointStamped, "point", 10)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.x=0.2

    def on_timer(self):
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "laser"
        self.x += 0.05
        ps.point.x = self.x
        ps.point.y = 0.0
        ps.point.z = 0.3

        self.pub.publish(ps)

def main():
    rclpy.init()

    rclpy.spin(TFPointBroadcasterPy())

    rclpy.shutdown()

if __name__ == '__main__':
    main()