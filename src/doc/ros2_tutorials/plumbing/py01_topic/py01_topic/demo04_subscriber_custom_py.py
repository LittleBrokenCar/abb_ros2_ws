import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from base_interfaces.msg import Student

class Subscriber(Node):
    def __init__(self):
        super().__init__("custom_msg_subscriber_node_py")
        self.get_logger().info("subscriber has been established")

        self.subscriber = self.create_subscription(Student, "chatter_student", self.do_callback, 10)

    def do_callback(self,msg):
        self.get_logger().info("sub msg: {} {} {}".format(msg.name, msg.age, msg.height))



def main():
    rclpy.init()

    rclpy.spin(Subscriber())

    rclpy.shutdown()


if __name__ == "__main__":
    main()
