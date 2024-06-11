import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publisher(Node):
    def __init__(self):
        super().__init__("str_publisher_node_py")
        self.get_logger().info("publisher has been established")

        self.count = 0
        self.publisher = self.create_publisher(String, "chatter", 10)
        self.timer = self.create_timer(1.0, self.pubmsg)
    
    def pubmsg(self):
        message = String()
        message.data = "hello world" + str(self.count)
        self.publisher.publish(message)
        self.count += 1
        self.get_logger().info("pub msg: {}".format(message.data))

def main():
    # print('Hi from py01_topic.')
    rclpy.init()

    rclpy.spin(Publisher())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
