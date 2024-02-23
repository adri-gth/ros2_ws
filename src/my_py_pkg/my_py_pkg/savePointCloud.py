#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import os
import open3d as o3d
from sensor_msgs_py import point_cloud2  # Asegúrate de tener esta dependencia

class PointCloudSaver(Node):
    def __init__(self):
        super().__init__('point_cloud_saver')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/front_camera/points',
            self.point_cloud_callback,
            10)
        self.save_directory = '/home/data/polytunnel_dataset/point_clouds'
        os.makedirs(self.save_directory, exist_ok=True)
        self.point_cloud_count = 0

    def point_cloud_callback(self, msg):
        print("Received a point cloud!")
        # Convertir PointCloud2 a Open3D PointCloud
        points_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not points_list:
            self.get_logger().error('Received an empty point cloud.')
            return
        np_points = np.array(points_list, dtype=np.float32)  # Usar float32 para compatibilidad
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)
        
        # Guardar la nube de puntos en un archivo .pcd
        file_path = os.path.join(self.save_directory, f'point_cloud_{self.point_cloud_count:04d}.pcd')
        o3d.io.write_point_cloud(file_path, pcd)
        
        self.point_cloud_count += 1

def main(args=None):
    rclpy.init(args=args)
    point_cloud_saver = PointCloudSaver()
    print("Save point cloud Node Running... ")
    rclpy.spin(point_cloud_saver)
    
    point_cloud_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
