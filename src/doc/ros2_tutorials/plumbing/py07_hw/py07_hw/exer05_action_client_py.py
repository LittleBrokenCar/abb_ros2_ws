import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Exer05ActionClient(Node):
    def __init__(self):
        super().__init__('exer05_action_client_py')
        self.get_logger().info('exer05_action_client has been established')

        self.count = 0
        self.timer = self.create_timer(1.0, self.func)

    def func(self):
        self.count += 1
        self.get_logger().info('func')

def main():
    rclpy.init()

    rclpy.spin(Exer05ActionClient())

    rclpy.shutdown()

if __name__ == '__main__':
    main()