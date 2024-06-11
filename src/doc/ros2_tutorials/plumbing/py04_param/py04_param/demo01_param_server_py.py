import rclpy
from rclpy.node import Node


class ParamServer(Node):
    def __init__(self):
        super().__init__('param_server_node_py', allow_undeclared_parameters=True)  # able to delete params
        self.get_logger().info('param_server has been established')

    def declare_param(self):
        self.get_logger().info("---------declare params---------")
        self.declare_parameter("car_name", "tiger")
        self.declare_parameter("car_width", 2.5)
        self.declare_parameter("car_length", 5)

        self.set_parameters([rclpy.Parameter("haha", value="xixi")])


    def get_param(self):
        self.get_logger().info("---------get params---------")
        # get one param
        car_name = self.get_parameter("car_name")
        self.get_logger().info("{}={}".format(car_name.name, car_name.value))
        # get params
        params = self.get_parameters(["car_name", "car_width", "car_length"])
        for item in params:
            self.get_logger().info("{} ========= {}".format(item.name, item.value))
        # judge whether a params inculded
        self.get_logger().info("include car_name? {}".format(self.has_parameter("car_name")))
        self.get_logger().info("include wheels? {}".format(self.has_parameter("wheels")))

        

    def update_param(self):
        self.get_logger().info("---------update params---------")
        self.set_parameters([rclpy.Parameter("car_name", value="Mouse")])
        car_name = self.get_parameter("car_name")
        self.get_logger().info("update {}={}".format(car_name.name, car_name.value))

    def del_param(self):
        self.get_logger().info("---------delete params---------")
        self.get_logger().info("include car_name? {}".format(self.has_parameter("car_name")))
        self.undeclare_parameter("car_name")
        self.get_logger().info("include car_name? {}".format(self.has_parameter("car_name")))



def main():
    rclpy.init()
    node = ParamServer()
    node.declare_param()
    node.get_param()
    node.update_param()
    node.del_param()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()