#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from facenet_pytorch import MTCNN, InceptionResnetV1
import torch
import cv2
import numpy as np
from face_recognition.msg import FaceEmbedding

class face_embedding_node(Node):
    def __init__(self):
        super().__init__('face_embedding_node')

        # initialize OpenCV bridge, MTCNN detector, and InceptionResnetV1 model
        self.bridge = CvBridge()
        self.detector = MTCNN()
        self.model = InceptionResnetV1(pretrained='vggface2').eval()

        # subscribe to webcam image topic and face detection topic
        self.image_subscription = self.create_subscription(Image, '/webcam_image', self.image_callback, 10)
        self.faces_subscription = self.create_subscription(String, '/face_recognition/face_detected', self.face_detected_callback, 10)
        
        # publish face embeddings
        self.publisher = self.create_publisher(FaceEmbedding, '/face_recognition/face_embedding', 10)
        
        # current image being processed
        self.current_image = None



    def image_callback(self, msg):
        # convert ROS image message to OpenCV image
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')



    def face_detected_callback(self, msg):
        if self.current_image is None:
            # no image received yet, ignore
            self.get_logger().warning('No image received yet')
            return

        # process the coordinates and extract faces
        face_coords = msg.data.split(';')
        embeddings = FaceEmbedding()

	    # calculate embeddings, which are the features of each face
        for coord in face_coords:
            if coord:
                x, y, w, h = map(int, coord.split(','))
                face = self.current_image[y:y+h, x:x+w]
                face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
                # normalize the face image
                face = torch.from_numpy(face).permute(2, 0, 1).float() / 255.0
                # add a dimension to the face tensor
                face = face.unsqueeze(0)

                # calculate the embedding using the InceptionResnetV1 model
                with torch.no_grad():
                    embedding = self.model(face).numpy()
                    embeddings.embeddings.append(embedding.flatten())

        # publish embeddings
        self.publisher.publish(embeddings)


        self.get_logger().info(f'Published embeddings: {embeddings.embeddings}')

def main(args=None):
    rclpy.init(args=args)
    node = face_embedding_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
