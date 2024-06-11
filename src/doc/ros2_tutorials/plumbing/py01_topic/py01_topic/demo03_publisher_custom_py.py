import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from base_interfaces.msg import Student


class Publisher(Node):
    def __init__(self):
        super().__init__("custom_msg_publisher_node_py")
        self.get_logger().info("publisher has been established")
        
        
        self.count = 0
        self.publisher = self.create_publisher(Student, "chatter_student", 10)
        self.timer = self.create_timer(1.0, self.pubmsg)
    
    def pubmsg(self):
        student = Student()
        student.name="maple"
        student.age=self.count
        student.height=180.5
        self.publisher.publish(student)
        self.count += 1
        self.get_logger().info("pub msg: {} {} {}".format(student.name, student.age, student.height))

def main():
    # print('Hi from py01_topic.')
    rclpy.init()

    rclpy.spin(Publisher())

    rclpy.shutdown()


if __name__ == '__main__':
    main()
