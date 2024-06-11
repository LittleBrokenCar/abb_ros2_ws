import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.time import Time


class TFListenerPy(Node):
    def __init__(self):
        super().__init__('tf_listener_py')
        self.get_logger().info('tf_listener_py has been established')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        if self.buffer.can_transform("camera", "laser", Time()):  # judge whether able to transform
            ts = self.buffer.lookup_transform("camera", "laser", Time())
            self.get_logger().info("---transformed result---")
            self.get_logger().info("frame_id:{},child_frame_id:{},T:({},{},{})".format(
                ts.header.frame_id, ts.child_frame_id,
                ts.transform.translation.x,
                ts.transform.translation.y,
                ts.transform.translation.z
            ))
        else:
            self.get_logger().info("---wait to transform---")



def main():
    rclpy.init()

    rclpy.spin(TFListenerPy())

    rclpy.shutdown()

if __name__ == '__main__':
    main()