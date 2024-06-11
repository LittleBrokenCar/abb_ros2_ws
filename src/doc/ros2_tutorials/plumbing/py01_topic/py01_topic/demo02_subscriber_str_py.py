import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class Subscriber(Node):
    def __init__(self):
        super().__init__("str_subscriber_node_py")
        self.get_logger().info("subscriber has been established")

        self.subscriber = self.create_subscription(String, "chatter", self.do_callback, 10)

    def do_callback(self,msg):
        self.get_logger().info("sub msg:{}".format(msg.data))



def main():
    rclpy.init()

    rclpy.spin(Subscriber())

    rclpy.shutdown()


if __name__ == "__main__":
    main()
