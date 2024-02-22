#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import cv2  # Importa OpenCV
import os

class PointCloudToPCDConverter(Node):
    def __init__(self):
        super().__init__('point_cloud_to_pcd_converter')
        self.subscription = self.create_subscription(PointCloud2, '/front_camera/points', self.points_callback, 10)
        self.video_writer = None
        self.frame_count = 0

    def points_callback(self, msg):
        print("Save point cloud!")
        # Convertir PointCloud2 a un array de Numpy
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        np_points = np.array(points_list, dtype=np.float64)

        # Crear una nube de puntos con Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)

        # Guardar la nube de puntos en un archivo .pcd (opcional)
        # o3d.io.write_point_cloud("/home/data/polytunnel_dataset/point_cloud_video/output.pcd", pcd)

        # Generar y guardar una imagen de profundidad
        depth_image = self.generate_depth_image(np_points)
        if self.video_writer is None:
            self.init_video_writer(depth_image.shape[1], depth_image.shape[0])
        self.video_writer.write(depth_image)

        # Incrementar el contador de frames
        self.frame_count += 1
        
    def generate_depth_image(self, points):
        # Suponiendo que Z es la profundidad
        depth = points[:, 2]

    # Asumiendo que `depth` es tu array de profundidades

        # Manejar NaN e infinitos
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        # Escalar y limitar los valores de profundidad al rango de uint8
        depth_min, depth_max = np.min(depth), np.max(depth)
        depth_scaled = (depth - depth_min) / (depth_max - depth_min)  # Normalizar al rango [0, 1]
        depth_scaled = np.clip(depth_scaled * 255, 0, 255).astype(np.uint8)  # Escalar a [0, 255] y convertir









        
        # Escalar los valores de profundidad para utilizar todo el rango de uint8
        # Eliminar valores atípicos o ajustar según sea necesario
        # depth_min, depth_max = np.percentile(depth, [5, 95])  # Usa percentiles para ignorar valores extremos
        # depth_scaled = np.clip((depth - depth_min) / (depth_max - depth_min), 0, 1)
        
        # depth_scaled = (depth_scaled * 255).astype(np.uint8)
        
        # Redimensionar para obtener una imagen cuadrada (esto es solo un ejemplo simple)
        size = int(np.sqrt(len(depth_scaled)))
        depth_image = np.zeros((size, size), dtype=np.uint8)
        
        # Llenar la imagen con los valores de profundidad escalados
        depth_image[:len(depth_scaled)] = np.reshape(depth_scaled[:size*size], (size, size))
        
        return depth_image


    # def generate_depth_image(self, points):
    #     # Suponiendo que Z es la profundidad
    #     depth = points[:, 2]
    #     depth_normalized = (depth - np.min(depth)) / (np.max(depth) - np.min(depth))
    #     depth_normalized = (depth_normalized * 255).astype(np.uint8)
    #     size = int(np.sqrt(depth_normalized.shape[0]))P
    #     depth_image = np.reshape(depth_normalized[:size*size], (size, size))
    #     return depth_image

    def init_video_writer(self, width, height):
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or 'XVID'
        self.video_writer = cv2.VideoWriter('/home/data/polytunnel_dataset/point_cloud_video/output_video.mp4', fourcc, 20.0, (width, height), False)

    def destroy_node(self):
        print("Releasing VideoWrite")
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    point_cloud_to_pcd_converter = PointCloudToPCDConverter()
    print("PointCloud to Video Converter Node Running...")
    rclpy.spin(point_cloud_to_pcd_converter)
    point_cloud_to_pcd_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







