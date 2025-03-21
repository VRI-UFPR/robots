#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import joblib
from sklearn import svm
from face_recognition.msg import FaceEmbedding
from face_recognition.msg import RecognitionResult

class face_recognition_node(Node):
    def __init__(self):
        super().__init__('face_recognition_node')

        # Carregar o modelo SVM e a biblioteca de embeddings
        self.model = joblib.load('svm_model.pkl')
        self.face_embeddings = np.load('face_embeddings.npy')
        self.face_labels = np.load('face_labels.npy')

        # Assinatura para receber embeddings de rostos
        self.subscription = self.create_subscription(FaceEmbedding, '/face_recognition/face_embedding', self.listener_callback, 10)
        # Publicador para publicar os resultados de reconhecimento
        self.publisher = self.create_publisher(RecognitionResult, '/face_recognition/recognition_result', 10)



    def listener_callback(self, msg):
        # Receber os embeddings faciais
        results = RecognitionResult()
        for embedding in msg.embeddings:
            # Fazer a previsão com o modelo SVM
            label = self.model.predict([embedding])
            results.append(label[0])

        # Publicar os resultados
        self.publisher.publish(results)

        self.get_logger().info(f'Published recognition results: {results}')

def main(args=None):
    rclpy.init(args=args)
    node = face_recognition_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
