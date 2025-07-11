#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros
import numpy as np
import random


class FakeLaserRandomBoxes(Node):
    def __init__(self):
        super().__init__('fake_laser_random_boxes')

        # ROS Publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        # Broadcast static TF between base_link and laser_frame
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.send_static_tf()

        # Laser parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        # Box field configuration
        self.num_boxes = 10         # number of obstacles
        self.box_size = 0.6         # box width (square)
        self.map_half_extent = 4.0  # +/- range in x and y

        random.seed(42)  # Make reproducible (optional)
        self.boxes = self.generate_random_boxes()

        self.get_logger().info("Fake laser with random boxes initialized.")

    def send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2  # height of sensor

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def generate_random_boxes(self):
        boxes = []
        for _ in range(self.num_boxes):
            x = random.uniform(-self.map_half_extent, self.map_half_extent)
            y = random.uniform(-self.map_half_extent, self.map_half_extent)
            boxes.append((x, y))
        return boxes

    def ray_box_intersection(self, px, py, dx, dy, box_center):
        half = self.box_size / 2
        cx, cy = box_center

        # Define the 4 box edges as line segments
        corners = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half)
        ]

        edges = [
            (corners[0], corners[1]),  # bottom
            (corners[1], corners[2]),  # right
            (corners[2], corners[3]),  # top
            (corners[3], corners[0])   # left
        ]

        closest_dist = None
        for (x1, y1), (x2, y2) in edges:
            denom = (x2 - x1) * dy - (y2 - y1) * dx
            if denom == 0:
                continue  # parallel

            t = ((px - x1) * dy - (py - y1) * dx) / denom
            u = ((x1 - px) * (y2 - y1) - (y1 - py) * (x2 - x1)) / denom

            if 0 <= t <= 1 and u >= 0:
                if closest_dist is None or u < closest_dist:
                    closest_dist = u

        return closest_dist

    def publish_scan(self):
        msg = LaserScan()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = 'laser_frame'

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        px, py = 0.0, 0.0
        ranges = []

        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            dx = math.cos(angle)
            dy = math.sin(angle)

            min_dist = self.range_max

            for box in self.boxes:
                dist = self.ray_box_intersection(px, py, dx, dy, box)
                if dist is not None and self.range_min <= dist <= min_dist:
                    min_dist = dist

            # Add Gaussian noise
            noisy_dist = min(self.range_max, max(self.range_min, min_dist + np.random.normal(0, 0.01)))
            ranges.append(noisy_dist)

        msg.ranges = ranges
        msg.intensities = [0.0] * self.num_readings

        self.publisher.publish(msg)
        self.get_logger().info("Published scan with random boxes.")


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserRandomBoxes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
