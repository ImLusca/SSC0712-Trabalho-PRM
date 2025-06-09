#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np

class Slammer(Node):
    def __init__(self):
        super().__init__('slammer')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_subscription = self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)


    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info('Received a new map!')

        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.get_logger().info(f'Map Resolution: {resolution} m/pixel')
        self.get_logger().info(f'Map Dimensions: {width}x{height} pixels')
        self.get_logger().info(f'Map Origin: ({origin_x}, {origin_y})')

        map_data = np.array(msg.data).reshape(height, width)

        # map_data[y_pixel, x_pixel]

        center_x_pixel = width // 2
        center_y_pixel = height // 2
        center_value = map_data[center_y_pixel, center_x_pixel]
        self.get_logger().info(f'Value at center ({center_x_pixel},{center_y_pixel}): {center_value}')

        free_space_count = np.sum(map_data == 0)
        occupied_space_count = np.sum(map_data == 100)
        unknown_space_count = np.sum(map_data == -1)
        self.get_logger().info(f'Free space: {free_space_count} pixels')
        self.get_logger().info(f'Occupied space: {occupied_space_count} pixels')
        self.get_logger().info(f'Unknown space: {unknown_space_count} pixels')


def main(args=None):
    rclpy.init(args=args)
    map_reader = Slammer()
    rclpy.spin(map_reader)
    map_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    def move_robot(self):
        pass

    def main(args=None):
        rclpy.init(args=args)
        node = Slammer()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()     

    if __name__ == '__main__':
        main()
