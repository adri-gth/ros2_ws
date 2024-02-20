#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
import cv_bridge 
import cv2 
import numpy as np


class ImageToVideoConverter(Node): 
    def __init__(self): 
        super().__init__('image_to_video_converter') 
        self.bridge = cv_bridge.CvBridge() 
        self.subscription = self.create_subscription( Image, '/front_camera/image_raw', self.image_callback, 10 ) 
        self.video_writer = None
    
    def image_callback(self, msg):
        print("Received an image!")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            #np_arr = np.frombuffer(msg.data, np.uint8)
            #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if self.video_writer is None:
                self.init_video_writer(cv_image)
            self.video_writer.write(cv_image)
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def init_video_writer(self, image):
        try:
            height, width, _ = image.shape
            video_format = 'mp4'  # or any other video format supported by OpenCV
            video_filename = '/home/data/polytunnel_dataset/video_rgb/output_video.' + video_format
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            
            fps = 30  # Frames per second
            self.video_writer = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))
            
            
        except Exception as e:
            self.get_logger().error('Error initializing video writer: %s' % str(e))


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