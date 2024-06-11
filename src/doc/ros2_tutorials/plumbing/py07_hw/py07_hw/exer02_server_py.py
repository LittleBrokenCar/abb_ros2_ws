import rclpy
from rclpy.node import Node
from base_interfaces.srv import Distance

from turtlesim.msg import Pose
from math import sqrt

"""
t position : /turtle/pose
"""


class Exer02Server(Node):
    def __init__(self):
        super().__init__('exer02_server_node_py')
        self.get_logger().info('Exer02Server has been established')

        self.t_x = 0.0
        self.t_y = 0.0
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.server = self.create_service(Distance, "distance", self.distance_callback)

    def pose_callback(self, pose):
        self.t_x = pose.x
        self.t_y = pose.y

    def distance_callback(self, request, response):
        goal_x = request.x
        goal_y = request.y
        distance_x = abs(goal_x - self.t_x)
        distance_y = abs(goal_y - self.t_y)
        distance = sqrt(pow(distance_x,2)+ pow(distance_y,2))
        response.distance = distance
        self.get_logger().info("target:({:.2f},{:.2f}), origin:({:.2f},{:.2f}), distance:{:.2f}".format(goal_x, goal_y, self.t_x, self.t_y, distance))
        return response


def main():
    rclpy.init()

    rclpy.spin(Exer02Server())

    rclpy.shutdown()

if __name__ == '__main__':
    main()