import rclpy
from rclpy.node import Node
import sys
from rclpy.logging import get_logger
from rclpy.action import ActionClient
from base_interfaces.action import Progress
from rclpy.action.client import ClientGoalHandle

class ProgressActionClient(Node):
    def __init__(self):
        super().__init__('progress_action_client_node_py')
        self.get_logger().info('action_client has been established')
        self.client = ActionClient(self, Progress, "action_get_sum")

    def send_goal(self, num):
        self.client.wait_for_server() # connect to the server
        goal = Progress.Goal()        # goal handle
        goal.num = num              
        self.future = self.client.send_goal_async(goal, self.fb_callback)  # return a future instance
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("goal failed")
            return
        self.get_logger().info("goal has been being processed")

        # get final result
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("final result:{}".format(result.sum))


    def fb_callback(self, fb_msg):
        progress = fb_msg.feedback.progress
        self.get_logger().info("consecutive feedback:{}".format(progress))



def main(): 
    if len(sys.argv) != 2:
        get_logger("rclpy").error("please input only one integer")
        sys.exit(1)

        return
    rclpy.init()
    action_client = ProgressActionClient()
    action_client.send_goal(int(sys.argv[1]))
    rclpy.spin(action_client)

    rclpy.shutdown()

if __name__ == '__main__':
    main()  