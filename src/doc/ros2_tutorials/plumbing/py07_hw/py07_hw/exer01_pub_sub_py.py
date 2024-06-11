import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
"""  
sub topic: /t1/turtle1/pose
sub msg: turtlesim/msg/Pose
        x: 0.0
        y: 0.0
        theta: 0.0
        linear_velocity: 0.0
        angular_velocity: 0.0

pub topic: /t2/turtle1/cmd_vel       ros2 topic type
pub msg: geometry_msgs/msg/Twist   ros2 interface proto
        linear:
        x: 0.0
        y: 0.0
        z: 0.0
        angular:
        x: 0.0
        y: 0.0
        z: 0.0
"""

class Exer01PubSub(Node):
    def __init__(self):
        super().__init__('Exer01PubSub_py')
        self.get_logger().info('Exe r01PubSub has been established')

        self.pub = self.create_publisher(Twist, "/t2/turtle1/cmd_vel", 10)
        self.sub = self.create_subscription(Pose, "/turtle1/pose",self.do_callback,10)

    def do_callback(self, pose):
        twist = Twist()
        twist.linear.x = pose.linear_velocity
        twist.angular.z = -pose.angular_velocity
        self.pub.publish(twist)

def main():
    rclpy.init()

    rclpy.spin(Exer01PubSub())

    rclpy.shutdown()

if __name__ == '__main__':
    main()