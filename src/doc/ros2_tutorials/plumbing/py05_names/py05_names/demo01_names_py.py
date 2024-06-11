import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__('MyNode_py', namespace="py05_names_py")
        self.get_logger().info('MyNode has been established')
        # global topic
        self.pub_global = self.create_publisher(String,"/global", 10)
        # relative topic
        self.pub_relative = self.create_publisher(String,"relative", 10)
        # private topic
        self.pub_private = self.create_publisher(String,"~/private", 10)

def main():
    rclpy.init()

    rclpy.spin(MyNode())

    rclpy.shutdown()

if __name__ == '__main__':
    main()
