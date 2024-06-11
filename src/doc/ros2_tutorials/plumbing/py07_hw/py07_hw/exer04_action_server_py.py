import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from base_interfaces.action import Nav
import threading
import time


class Exer04ActionServer(Node):
    def __init__(self):
        super().__init__('exer04_action_server_py')
        self.get_logger().info('exer04_action_server_py has been established')

        self.t_x = 0.0
        self.t_y = 0.0
        self.sub = self.create_subscription(Pose, "/turtle1/pose",self.pose_callback, 10)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd/vel",10)
        self.action_server = ActionServer(
            self, 
            Nav, 
            "nav",
            execute_callback=self.execute_callback,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel)
    def pose_callback(self, pose):
        self.t_x = pose.x
        self.t_y = pose.y

    def execute_callback(self, goal_handle):
        self.get_logger().info("main func has been running")
        feedback = Nav.Feedback()

        while 1:
            if goal_handle.is_canceling:

                time.sleep(1.0)  # ## 3.8.6 STOP 

        

    def handle_goal(self, goal_request):
        if goal_request.request.goal_x < 0 or goal_request.request.goal_x > 11.08 or goal_request.request.goal_y <0 or goal_request.request.goal_y > 11.08:
            self.get_logger().warning("The goal is not inside the window!")
            return GoalResponse.REJECT
        else:
            self.get_logger().info("The goal is inside the window, keep running")
            return GoalResponse.ACCEPT
        
    def handle_cancel(self, cancel_request):
        self.get_logger().info("The processing is canceling")
        return CancelResponse.ACCEPT
def main():
    rclpy.init()

    rclpy.spin(Exer04ActionServer())

    rclpy.shutdown()

if __name__ == '__main__':
    main()