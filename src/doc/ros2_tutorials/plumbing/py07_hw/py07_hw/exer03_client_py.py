import rclpy
from rclpy.node import Node
from base_interfaces.srv import Distance
from rclpy.logging import get_logger
import sys

class Exer03Client(Node):
    def __init__(self):
        super().__init__('exer03_client_node_py')
        self.get_logger().info('Exer03Client has been established')

        self.client = self.create_client(Distance, "distance")

        while not self.client.wait_for_service(1.0):
            self.get_logger().info("service is being connected, please stand by")
        self.get_logger().info("service has been successfully connected, please wait for response")

    def send_request(self):
        request = Distance.Request()
        request.x = float(sys.argv[1])
        request.y = float(sys.argv[2])
        request.theta = float(sys.argv[3])
        self.future = self.client.call_async(request)  # send the request
    

def main():
    if len(sys.argv) != 5 or len(sys.argv) != 4:  # 5 correspond to launch_file--client | 4 correspond to command in terminal without --ros2-args
        get_logger("rclpy").error("please input only three float")
        sys.exit()
    rclpy.init()
    client = Exer03Client()
    client.send_request()

    rclpy.spin_until_future_complete(client, client.future) # process the response

    try:
        response = client.future.result()
        client.get_logger().info("respose:{}".format(response.distance))
    except Exception as e:
        client.get_logger().error("response failed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()