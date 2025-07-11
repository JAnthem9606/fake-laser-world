#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import tf2_ros


class OneWallLaserPublisher(Node):
    def __init__(self):
        super().__init__('one_wall_laser_publisher')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_one_wall_scan)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.1
        self.range_max = 7.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.wall_y = 5.0  # Only one wall at y = +5

        self.get_logger().info("One-wall laser publisher initialized.")

    def broadcast_static_tf(self):
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

    def publish_one_wall_scan(self):
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

        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            dx = math.cos(angle)
            dy = math.sin(angle)

            # Check if ray points toward the front wall (positive y-direction)
            if dy > 0:
                distance = self.wall_y / dy
                x_hit = distance * dx

                # Limit hit within the wall's width, here assumed to be infinite
                if abs(x_hit) <= self.range_max:
                    noise = np.random.normal(0, 0.01)
                    noisy_distance = min(self.range_max, max(self.range_min, distance + noise))
                    ranges.append(noisy_distance)
                    continue

            # No hit — return max range
            ranges.append(self.range_max)

        msg.ranges = ranges
        msg.intensities = [0.0] * self.num_readings

        self.publisher.publish(msg)
        self.get_logger().info('Published one-wall laser scan.')


def main(args=None):
    rclpy.init(args=args)
    node = OneWallLaserPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
