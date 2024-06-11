import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from base_interfaces.action import Progress
from rclpy.action import ActionServer
import time

class ProgressActionServer(Node):
    def __init__(self):
        super().__init__('progress_action_server_node_py')
        self.get_logger().info('action_server has been established')

        self.server = ActionServer(
            self,
            Progress,
            "action_get_sum",
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        feedback = Progress.Feedback()
        num = goal_handle.request.num
        sum = 0
        for i in range(1,num+1):
            sum +=i
            feedback.progress = i/num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("consecutive feedback:{}%".format(feedback.progress*100))
            time.sleep(1.0)

        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum

        self.get_logger().info("calculation result:{}".format(result.sum))
        return result

def main(): 
    rclpy.init()

    rclpy.spin(ProgressActionServer())

    rclpy.shutdown()

if __name__ == '__main__':
    main()  