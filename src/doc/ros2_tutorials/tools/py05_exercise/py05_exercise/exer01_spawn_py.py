import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn


class Exer01SpawnPy(Node):
    def __init__(self):
        super().__init__('exer01_spawn_py')
        self.get_logger().info('exer01_spawn_py has been established')
        self.declare_parameter("x", 3.5)
        self.declare_parameter("y", 3.5)
        self.declare_parameter("theta", 1.57)
        self.declare_parameter("turtle_name", "turtle2")
        self.x = self.get_parameter("x").get_parameter_value().double_value
        self.y = self.get_parameter("y").get_parameter_value().double_value
        self.theta = self.get_parameter("theta").get_parameter_value().double_value
        self.turtle_name = self.get_parameter("turtle_name").get_parameter_value().string_value
        
        self.client = self.create_client(Spawn, "/spawn")
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("---service is connecting, please stand by---")
            
    def request(self):
        request = Spawn.Request()
        request.x = self.x
        request.y = self.y
        request.theta = self.theta
        request.name = self.turtle_name

        self.future = self.client.call_async(request)

def main():
    rclpy.init()

    spawn = Exer01SpawnPy()
    spawn.request()
    rclpy.spin_until_future_complete(spawn, spawn.future)
    response = spawn.future.result()
    if len(response.name) == 0:
        spawn.get_logger().info("the name has already exsisted")
    else:
        spawn.get_logger().info("a new turtle has been successfully created")

    # rclpy.spin(Exer01SpawnPy())

    rclpy.shutdown()

if __name__ == '__main__':
    main()