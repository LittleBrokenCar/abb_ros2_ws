import rclpy
from rclpy.node import Node
import sys
from rclpy.logging import get_logger
from base_interfaces.srv import AddInts

class AddIntsClient(Node):
    def __init__(self):
        super().__init__('add_ints_client_node_py')
        self.get_logger().info('client has been established')

        self.client = self.create_client(AddInts, "add_ints")

        while not self.client.wait_for_service(1.0):
            self.get_logger().info("service is being connected, please stand by")
        self.get_logger().info("service has been successfully connected, please wait for response")

    def send_request(self):
        request = AddInts.Request()
        request.num1 = int(sys.argv[1])
        request.num2 = int(sys.argv[2])
        self.future = self.client.call_async(request)  # send the request
    

def main():
    if len(sys.argv) != 3:
        get_logger("rclpy").error("please input only two integers")
        sys.exit()
    rclpy.init()
    client = AddIntsClient()
    client.send_request()

    rclpy.spin_until_future_complete(client, client.future) # process the response

    try:
        response = client.future.result()
        client.get_logger().info("respose:{}".format(response.sum))
    except Exception as e:
        client.get_logger().error("response failed")
    rclpy.shutdown()

if __name__ == '__main__':
    main()