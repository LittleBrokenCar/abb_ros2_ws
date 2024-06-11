import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from rclpy.time import Time
from rclpy.duration import Duration


class TimeNode(Node):
    def __init__(self):
        super().__init__('time_node_py')
        self.get_logger().info('TimeNode has been established')

        # self.demo_rate()
        # self.demo_time()
        # self.demo_duration()
        self.demo_opt()

    def demo_opt(self):
        t1=Time(seconds=20)
        t2=Time(seconds=15)

        du1 = Duration(seconds=7)
        du2 = Duration(seconds=13)

        self.get_logger().info("t1>=t2? {}".format(t1>=t2))
        self.get_logger().info("t1<t2? {}".format(t1<t2))

        self.get_logger().info("t1-t2=? {}".format((t1-t2).nanoseconds))
        self.get_logger().info("t1+du1=? {}".format((t1+du1).nanoseconds))
        self.get_logger().info("t1-du1=? {}".format((t1-du1).nanoseconds))

        self.get_logger().info("du1<du2? {}".format(du1<du2))
        self.get_logger().info("du1>=du2? {}".format(du1>=du2))


    def demo_duration(self):
        du1 = Duration(seconds=10, nanoseconds=800000000)
        self.get_logger().info("ns={}".format(du1.nanoseconds))


    def demo_time(self):

        t1 = Time(seconds=5, nanoseconds=500000000)
              
        right_now = self.get_clock().now()

        self.get_logger().info("s={}, ns={}".format(t1.seconds_nanoseconds()[0],t1.seconds_nanoseconds()[1]))
        self.get_logger().info("s={}, ns={}".format(right_now.seconds_nanoseconds()[0],right_now.seconds_nanoseconds()[1]))
        self.get_logger().info("ns={}".format(t1.nanoseconds))
        self.get_logger().info("ns={}".format(right_now.nanoseconds))

    def demo_rate(self):
        self.rate = self.create_rate(1.0)
        # while rclpy.ok():
        #     self.get_logger().info("+++++++")
            # self.rate.sleep()  # make main procedure stand by, must start a child threading
        thread = threading.Thread(target=self.do_wait)
        thread.start()

    def do_wait(self):
        while rclpy.ok():
            self.get_logger().info("+++++++")
        self.rate.sleep() 


def main():
    rclpy.init()

    rclpy.spin(TimeNode())

    rclpy.shutdown()

if __name__ == '__main__':
    main() 