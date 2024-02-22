#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import numpy as np
import os
import time

class ImageToVideoConverter(Node):
    def __init__(self):
        super().__init__('image_to_video_converter')
        self.bridge = cv_bridge.CvBridge()
        self.subscription = self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 10)
        self.video_writer = None
        self.image_count = 0  # Contador para nombres de archivos de imagen

    def image_callback(self, msg):
        print("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.video_writer is None:
                self.init_video_writer(cv_image)
            self.video_writer.write(cv_image)
            
            # Guardar la imagen recibida como un archivo
            self.save_image(cv_image)
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_format = 'mp4'
            video_filename = '/home/data/polytunnel_dataset/video_rgb/output_video.' + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            
            fps = 30  # Frames per second
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
        except Exception as e:
            self.get_logger().error('Error initializing video writer: %s' % str(e))

    def save_image(self, cv_image):
        # Crear directorio si no existe
        image_dir = '/home/data/polytunnel_dataset/image_rbg'
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)
        
        # Generar un nombre de archivo único para la imagen
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        image_path = os.path.join(image_dir, f'image_{timestamp}_{self.image_count:04d}.png')
        cv2.imwrite(image_path, cv_image)
        self.image_count += 1
        

    def destroy_node(self):
        print("Releasing VideoWrite")
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_to_video_converter = ImageToVideoConverter()
    print("Save_image2video Rode Running...")
    
    rclpy.spin(image_to_video_converter)
    image_to_video_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()