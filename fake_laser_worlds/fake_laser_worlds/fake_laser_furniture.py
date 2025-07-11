#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros
import numpy as np


class FakeLaserRoomWithFurniture(Node):
    def __init__(self):
        super().__init__('fake_laser_room_with_furniture')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.send_static_tf()

        # Laser settings
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        # Room boundaries (rectangle)
        self.room_min_x = -4.0
        self.room_max_x = 4.0
        self.room_min_y = -3.0
        self.room_max_y = 3.0

        # Furniture: list of boxes [(center_x, center_y, size)]
        self.furniture = [
            (1.0, 1.0, 0.6),   # Table
            (-1.5, 0.5, 0.4),  # Chair
            (-2.0, -1.5, 0.8), # Sofa
            (2.5, -1.0, 0.5)   # Cabinet
        ]

        self.get_logger().info("Fake laser for open room with furniture started.")

    def send_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def ray_intersect_segment(self, px, py, dx, dy, x1, y1, x2, y2):
        denom = (x2 - x1) * dy - (y2 - y1) * dx
        if denom == 0:
            return None
        t = ((px - x1) * dy - (py - y1) * dx) / denom
        u = ((x1 - px) * (y2 - y1) - (y1 - py) * (x2 - x1)) / denom
        if 0 <= t <= 1 and u >= 0:
            return u
        return None

    def box_edges(self, cx, cy, size):
        half = size / 2.0
        corners = [
            (cx - half, cy - half),
            (cx + half, cy - half),
            (cx + half, cy + half),
            (cx - half, cy + half)
        ]
        return [
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[3]),
            (corners[3], corners[0])
        ]

    def room_walls(self):
        x1, x2 = self.room_min_x, self.room_max_x
        y1, y2 = self.room_min_y, self.room_max_y
        return [
            ((x1, y1), (x2, y1)),  # bottom
            ((x2, y1), (x2, y2)),  # right
            ((x2, y2), (x1, y2)),  # top
            ((x1, y2), (x1, y1))   # left
        ]

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

        # Combine all segments
        segments = self.room_walls()
        for obj in self.furniture:
            segments += self.box_edges(*obj)

        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            dx = math.cos(angle)
            dy = math.sin(angle)

            min_dist = self.range_max

            for (x1, y1), (x2, y2) in segments:
                dist = self.ray_intersect_segment(px, py, dx, dy, x1, y1, x2, y2)
                if dist is not None and self.range_min <= dist <= min_dist:
                    min_dist = dist

            noisy = min(self.range_max, max(self.range_min, min_dist + np.random.normal(0, 0.01)))
            ranges.append(noisy)

        msg.ranges = ranges
        msg.intensities = [0.0] * self.num_readings
        self.publisher.publish(msg)

        self.get_logger().info("Published laser scan with room and furniture.")


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserRoomWithFurniture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
