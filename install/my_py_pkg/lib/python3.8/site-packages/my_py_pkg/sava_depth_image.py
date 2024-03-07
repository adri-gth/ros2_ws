#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import os
import cv2
from cv_bridge import CvBridge

class DepthImageSaver(Node):
    def __init__(self):
        super().__init__('depth_image_saver')
        self.subscription = self.create_subscription(Image,'/front_camera/depth/image_raw',self.image_callback,10)
        self.save_directory = '/home/data/polytunnel_dataset/depth_images'
        os.makedirs(self.save_directory, exist_ok=True)
        self.bridge = CvBridge()
        self.image_count = 0

    def image_callback(self, msg):
        print("Received a depth image!")
        # Convertir la imagen de ROS a un array de NumPy usando CvBridge
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Asumiendo que la imagen de profundidad es de 16 bits
        # Si es de 32 bits flotantes, puedes cambiar el dtype a np.float32
        np_image = np.array(cv_image, dtype=np.uint16)

        # Guardar la imagen de profundidad como un archivo .png
        file_path = os.path.join(self.save_directory, f'{self.image_count:04d}.png')
        cv2.imwrite(file_path, np_image)
        print(f"Saved image to")
        
        self.image_count += 1

def main(args=None):
    rclpy.init(args=args)
    depth_image_saver = DepthImageSaver()
    print("Depth image saver Node Running... ")
    rclpy.spin(depth_image_saver)
    
    depth_image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
