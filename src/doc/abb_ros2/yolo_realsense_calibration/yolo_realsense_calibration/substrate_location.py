import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
from scipy.spatial.transform import Rotation
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class PointTransformer(Node):
    def __init__(self):
        super().__init__('substrate_location_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

        # self.substrate_in_object_frame = PointStamped()
        # self.substrate_in_object_frame.header.frame_id = "object_frame"
        # self.transformed_point = PointStamped()
        # self.transformed_point.header.frame_id = "base_link"

        self.declare_parameter("offsets", [0.0, 0.0, 0.0])  # quaternion from object_frame to substrate_from is set to [0,0,0,1], decided by the model
        self.offsets = self.get_parameter(
            "offsets").get_parameter_value().double_array_value

        # pubs
        # self.publisher = self.create_publisher(PointStamped, 'visualization_transformed_point', 10)
        
        self.transform_point_to_robot_base_frame()

    def transform_point_to_robot_base_frame(self):

        # self.substrate_in_object_frame.header.stamp = self.get_clock().now().to_msg()
        # self.substrate_in_object_frame.point.x = self.offsets[0]
        # self.substrate_in_object_frame.point.y = self.offsets[1]
        # self.substrate_in_object_frame.point.z = self.offsets[2]
        future = self.tf_buffer.wait_for_transform_async("base_link", "object_frame", rclpy.time.Time())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done() and future.result() is not None:
            trans = self.tf_buffer.lookup_transform("base_link", "object_frame", rclpy.time.Time())
            self.get_logger().info(
                "frame_id:{},child_frame_id:{},translation:({},{},{}),quaternion:({},{},{},{})".format(
                    trans.header.frame_id, trans.child_frame_id,
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ))
            translation_bTo = [trans.transform.translation.x,
                           trans.transform.translation.y,
                           trans.transform.translation.z
                           ]
            quaternion_bTo = [trans.transform.rotation.x,
                          trans.transform.rotation.y,
                          trans.transform.rotation.z,
                          trans.transform.rotation.w
                          ]

            translation_oTs = [self.offsets[0],self.offsets[1],self.offsets[2],1]
            translation_bTs = PointTransformer.do_transform_point(translation_oTs, translation_bTo, quaternion_bTo)
            quaternion_bTs = quaternion_bTo
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'substrate_frame'
            t.transform.translation.x = translation_bTs[0]  
            t.transform.translation.y = translation_bTs[1]
            t.transform.translation.z = translation_bTs[2]
            t.transform.rotation.x = quaternion_bTs[0]
            t.transform.rotation.y = quaternion_bTs[1]
            t.transform.rotation.z = quaternion_bTs[2]
            t.transform.rotation.w = quaternion_bTs[3]
            self.broadcaster.sendTransform(t)
            self.get_logger().info(f"substrate_frame in base_link: {t}")
            # rpy = Rotation.from_quat(quaternion_bTs).as_euler("xyz",degrees=True)
            # self.get_logger().info(f"substrate_frame in base_link RPY: {rpy}")

            # point = [self.substrate_in_object_frame.point.x, self.substrate_in_object_frame.point.y,
            #          self.substrate_in_object_frame.point.z, 1]
            # transformed_result = PointTransformer.do_transform_point(translation_oTs, translation, quaternion)
            # self.transformed_point.header.stamp = self.get_clock().now().to_msg()
            # self.transformed_point.point.x = transformed_result[0]
            # self.transformed_point.point.y = transformed_result[1]
            # self.transformed_point.point.z = transformed_result[2]
            # self.publisher.publish(self.transformed_point)
            # self.get_logger().info(f"substrate_frame in base_link: {self.transformed_point.point}")

        else:
            self.get_logger().info("---wait to transform---")

    @staticmethod
    def quaternion_to_rotation_matrix(quaternion):
        x, y, z, w = quaternion
        rotation_matrix = np.array([
            [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
            [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
            [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
        ])
        return rotation_matrix

    @staticmethod
    def do_transform_point(point, translation, quaternion):
        T = np.eye(4)
        T[:3, :3] = np.asarray(PointTransformer.quaternion_to_rotation_matrix(quaternion))
        T[:3, 3] = np.asarray(translation)
        return np.dot(T, np.asarray(point))

    @staticmethod
    def rotate_quaternion(quaternion, axis, angle):
        """
        Rotate a quaternion by a given angle around a specified axis.

        Parameters:
        quaternion (np.array): The original quaternion as an array [x, y, z, w].
        axis (np.array): The axis of rotation as an array [x, y, z].
        angle (float): The rotation angle in degrees.

        Returns:
        np.array: The rotated quaternion.
        """
        axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) != 0 else axis
        rot = Rotation.from_rotvec(np.radians(angle) * np.array(axis))
        new_quaternion = rot * Rotation.from_quat(quaternion)
        return new_quaternion.as_quat()

    @staticmethod
    def rotation_matrix_to_quaternion(rotation_matrix):
        rotation_matrix = np.asarray(rotation_matrix)
        # if np.allclose(np.dot(rotation_matrix.T, rotation_matrix), np.eye(3)) and np.isclose(
        #         np.linalg.det(rotation_matrix), 1):
        try:
            rotation = Rotation.from_matrix(rotation_matrix)
            quaternion = rotation.as_quat()  # Returns numpy array [x, y, z, w]
            return quaternion
        except Exception as e:
            print(f"Error in func rotation_matrix_to_quaternion:{e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    # rotation_matrix_sa = np.array([
    #     [0.0, 0.0, -1.0],
    #     [0.0, 1.0, 0.0],
    #     [1.0, 0.0, 0.0]
    # ])
    # rotation_matrix_sb = np.array([
    #     [0.0, 0.0, 1.0],
    #     [-1.0, 0.0, 0.0],
    #     [0.0, -1.0, 0.0]
    # ])
    # rotation_matrix_ab = np.linalg.inv(rotation_matrix_sa) @ rotation_matrix_sb
    # print(PointTransformer.rotation_matrix_to_quaternion(rotation_matrix_ab))

    rotation_matrix_sb = np.array([
        [-0.06256779,  0.99784887,  0.01956779],
        [ 0.99770652,  0.06304215, -0.02464525],
        [-0.02582583,  0.01798091, -0.99950473]
    ])
    print(PointTransformer.rotation_matrix_to_quaternion(rotation_matrix_sb))