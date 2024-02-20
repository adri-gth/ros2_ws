#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np



class PointCloudToPCDConverter(Node):
    def __init__(self):
        super().__init__('point_cloud_to_pcd_converter')
        self.subscription = self.create_subscription(PointCloud2, '/front_camera/points', self.points_callback, 10)

    def points_callback(self, msg):
        # Convertir PointCloud2 a un array de Numpy
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        np_points = np.array(points_list, dtype=np.float64)

        # Crear una nube de puntos con Open3D
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np_points)

        # Guardar la nube de puntos en un archivo .pcd
        o3d.io.write_point_cloud("/home/data/polytunnel_dataset/point_clouds/output.pcd", pcd)
        self.get_logger().info("PointCloud saved to '/home/data/polytunnel_dataset/point_clouds/output.pcd'")

def main(args=None):
    rclpy.init(args=args)
    point_cloud_to_pcd_converter = PointCloudToPCDConverter()
    rclpy.spin(point_cloud_to_pcd_converter)
    point_cloud_to_pcd_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







