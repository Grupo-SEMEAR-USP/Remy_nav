#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry.laser_geometry import LaserProjection
import tf2_ros

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')
        self.lp = LaserProjection()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.publisher = self.create_publisher(PointCloud2, '/cloud', 10)

    def scan_callback(self, scan_msg):
        try:
            cloud_msg = self.lp.projectLaser(scan_msg)
            cloud_msg.header.frame_id = scan_msg.header.frame_id
            self.publisher.publish(cloud_msg)
        except Exception as e:
            self.get_logger().warn(f"Erro ao converter: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
