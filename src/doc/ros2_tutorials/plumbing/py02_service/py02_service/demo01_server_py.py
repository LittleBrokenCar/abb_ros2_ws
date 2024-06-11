import rclpy
from rclpy.node import Node
from base_interfaces.srv import AddInts


class AddIntsServer(Node):
    def __init__(self):
        super().__init__('add_ints_server_node_py')
        self.get_logger().info('server has been established')

        self.server = self.create_service(AddInts, "add_ints", self.do_sum)

    def do_sum(self, request, response):
        response.sum = request.num1 + request.num2
        self.get_logger().info("{}+{}={}".format(request.num1, request.num2, response.sum))
        return response
    
def main():
    rclpy.init()

    rclpy.spin(AddIntsServer())

    rclpy.shutdown()

if __name__ == '__main__':
    main()