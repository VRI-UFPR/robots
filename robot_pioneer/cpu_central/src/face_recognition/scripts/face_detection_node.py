#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from mtcnn import MTCNN
import numpy as np

class face_detection_node(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # initialize OpenCV bridge and MTCNN detector
        self.bridge = CvBridge()
        self.detector = MTCNN()

        # subscribe to webcam image topic
        self.subscription = self.create_subscription(Image, '/webcam_image', self.listener_callback, 10)
        # publish face coordinates
        self.publisher = self.create_publisher(String, '/face_recognition/face_detected', 10)



    def listener_callback(self, msg):
        # convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # detect faces in the image using MTCNN
        results = self.detector.detect_faces(cv_image)
        # the result is a list of dictionaries, each containing the bounding box of
        # the face detected in the image

        # extract the bounding box coordinates of the detected faces
        faces = []
        for result in results:
            x, y, w, h = result['box']
            faces.append(f"{x},{y},{w},{h}")

        # publish face coordinates
        face_msg = String()
        face_msg.data = ";".join(faces)
        self.publisher.publish(face_msg)

        self.get_logger().info(f'Detected faces: {face_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = face_detection_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
