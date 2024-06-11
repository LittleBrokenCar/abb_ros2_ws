import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyParam(Node):
    def __init__(self):
        super().__init__('my_param_node_py')
        self.get_logger().info('Param has been established')

        p1 = rclpy.Parameter("car_name", value="Tiger")
        p2 = rclpy.Parameter("car_width", value=1.5)
        p3 = rclpy.Parameter("car_height", value=2)
        
        self.get_logger().info("car_name = {}".format(p1.value))
        self.get_logger().info("car_width = {}".format(p2.value))
        self.get_logger().info("car_height = {}".format(p3.value))

        self.get_logger().info("key = {}".format(p1.name))

def main():
    rclpy.init()

    rclpy.spin(MyParam())

    rclpy.shutdown()

if __name__ == '__main__':
    main()