# 出于简单考虑，暂不引入ros-realsense节点与ros-yolo节点，将两节点合成一个，但手眼标定后的静态坐标变换节点放在另一节点文件中
# 出于简单考虑，暂不引入深度的区域判断算法，直接以中心点作为目标深度
# 出于简单考虑，暂不引入目标跟踪与二维转三维的算法，直接使用自带pixel_to_pointer()计算深度
# 出于简单考虑，暂不引入mask、keypoint、probs的结果分析
# 出于简单考虑，暂不引入QoS
# 出于简单考虑，未区分按照私有属性约定格式命名"self._private_attribute"，当前命名为"self.private_attribute"
# 默认相机内参准确
# ------2024.3.6
import random

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import BoundingBox2D
from geometry_msgs.msg import PointStamped

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from std_msgs.msg import String

import pyrealsense2 as rs
import numpy as np
import cv2

class YoloRealsenseNode(Node):
    def __init__(self):
        super().__init__("yolo_realsense_node")

        # params
        self.declare_parameter("model", "yolov8n.pt")
        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.declare_parameter("device", "cuda:0")
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("canvas_width", 640)
        self.canvas_width = self.get_parameter(
            "canvas_width").get_parameter_value().integer_value

        self.declare_parameter("canvas_height", 480)
        self.canvas_height = self.get_parameter(
            "canvas_height").get_parameter_value().integer_value

        self.declare_parameter("frame_rate", 30)
        self.frame_rate = self.get_parameter(
            "frame_rate").get_parameter_value().integer_value

        # yolov8
        self.yolo = YOLO(self.model)
        self.yolo.fuse()  # convolutional neural networks optimization
        self.color = {}

        # realsense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.canvas_width, self.canvas_height, rs.format.z16,
                                  self.frame_rate)
        self.config.enable_stream(rs.stream.color, self.canvas_width, self.canvas_height, rs.format.bgr8,
                                  self.frame_rate)

        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        self.intrinsics = None
        # pub
        """3.5 先发到终端"""
        self.str_publisher = self.create_publisher(String, "topic_target_point_position_str", 10)
        self.color_img_publisher = self.create_publisher(Image, "topic_target_point_position_color_img", 10)
        self.depth_img_publisher = self.create_publisher(Image, "topic_target_point_position_depth_img", 10)
        # timer
        self.timer = self.create_timer(float(1 / self.frame_rate), self.pubcb)

        # CVBridge
        self.bridge = CvBridge()


    def pubcb(self) -> None:
        message = String()
        message.data = ""

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            message.data = "Errors in frames"
            self.str_publisher.publish(message)
            self.get_logger().info("pub msg: {}".format(message.data))
        else:
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # cv_image = self.cv_bridge.imgmsg_to_cv2(color_image)
            results = self.yolo.predict(source=color_image, save=False, show_conf=False, verbose=False,
                                        stream=False, conf=self.threshold, device=self.device)
            results: Results = results[0].cpu()
            if results.boxes:
                item_num = len(results.boxes.cls)
                hypothesis_list = []
                target_position_list = []
                for i in range(item_num):
                    hypothesis = {
                        "class_id": int(results.boxes.cls[i]),
                        "class_name": self.yolo.names[int(results.boxes.cls[i])],
                        "confidence": float(results.boxes.conf[i])
                    }
                    hypothesis_list.append(hypothesis)

                    x1, y1, x2, y2 = results.boxes.xyxy.tolist()[i]
                    x_center = int((x1 + x2) / 2)
                    y_center = int((y1 + y2) / 2)
                    depth = aligned_depth_frame.get_distance(x_center, y_center)
                    if not self.intrinsics:
                        self.intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
                    x, y, z = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_center, y_center], depth)
                    target_position_list.append({"X": x, "Y": y, "Z": z, "x_center": x_center, "y_center": y_center})
                    message.data += "class_name:{}, confidence:{}, ({.4f},{.4f},{.4f})\n".format(
                        hypothesis["class_id"], hypothesis["confidence"], x, y, z)
                    cv2.circle(color_image, (x_center, y_center), 5, (0, 255, 0), -1)
                    cv2.putText(color_image, f"({x:.2f}, {y:.2f}, {z:.2f})", (x_center-100, y_center-10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0), 2)

                self.str_publisher.publish(message)
                self.get_logger().info("pub msg: {}".format(message.data))
                self.color_img_publisher.publish(self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8"))   
                self.depth_img_publisher.publish(self.bridge.cv2_to_imgmsg(depth_image, encoding="passthrough"))   
                
            else:
                message.data = "Errors in results"
                self.str_publisher.publish(message)
                self.get_logger().info("pub msg: {}".format(message.data))


def main():
    rclpy.init()
    node = YoloRealsenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # # yolo tutorials
    # """tutorial 1"""
    # # Load a model
    # model = YOLO('yolov8n.pt')  # pretrained YOLOv8n model
    # # model = torch.hub.load('ultralytics/yolov8', 'yolov8n', pretrained=True)
    # color_image = 'robotics.jpg'
    # # Run batched inference on a list of images
    # results = model(color_image, save=False, show_conf=False)  # return a list of Results objects
    # results = results[0].cpu()
    # detections = results.boxes.xyxy.tolist()
    # """Method 1"""
    # # # Process results list
    # # for result in results:
    # #     boxes = result.boxes  # Boxes object for bbox outputs
    # #     masks = result.masks  # Masks object for segmentation masks outputs
    # #     keypoints = result.keypoints  # Keypoints object for pose outputs
    # #     probs = result.probs  # Probs object for classification outputs
    # #     im_array = result.plot()  # plot a BGR numpy array of predictions
    # #     im = Image.fromarray(im_array[..., ::-1])  # RGB PIL image
    # #     im.show()  # show image
    # """Method 2"""
    # im_array = results.plot()
    # im = Image.fromarray(im_array[..., ::-1])
    # im.show()
    # """Method 3"""
    # # im_array = results[0].plot()[..., ::1]
    # # resized_image = cv2.resize(im_array, (480, 640))
    # # cv2.circle(resized_image, (100, 100), 5, (0, 255, 0), -1)
    # # cv2.imshow('YOLOv8 Real-Time Detection with 3D Coordinates', resized_image)
    # # cv2.waitKey(0)
    # # cv2.destroyAllWindows()
    #
    # """tutorial 2"""
    # # # Create a new YOLO model from scratch
    # # model = YOLO('yolov8n.yaml')
    # #
    # # # Load a pretrained YOLO model (recommended for training)
    # # model = YOLO('yolov8n.pt')
    # #
    # # # Train the model using the 'coco128.yaml' dataset for 3 epochs
    # # results = model.train(data='coco128.yaml', epochs=3)
    # #
    # # # Evaluate the model's performance on the validation set
    # # results = model.val()
    # #
    # # # Perform object detection on an image using the model
    # # results = model('https://ultralytics.com/images/bus.jpg')
    # #
    # # # Export the model to ONNX format
    # # success = model.export(format='onnx')

