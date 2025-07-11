#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import tf2_ros
import time


class FakeLaserPublisher(Node):
    def __init__(self):
        super().__init__('fake_laser_with_tf_publisher')

        # LaserScan publisher
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_scan)  # 10 Hz

        # Static TF broadcaster: base_link -> laser_frame
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        # Laser scan configuration
        self.angle_min = -math.pi / 2
        self.angle_max = math.pi / 2
        self.angle_increment = math.radians(1.0)  # 1 degree
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.get_logger().info("Fake laser with TF broadcaster initialized.")

    def broadcast_static_tf(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        # Position of laser relative to base_link (can be adjusted)
        t.transform.translation.x = 0.2
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2

        # No rotation
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def publish_fake_scan(self):
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

        # Simulated sine-wave distances
        ranges = 5.0 + 0.5 * np.sin(np.linspace(0, 2 * np.pi, self.num_readings))
        msg.ranges = ranges.tolist()
        msg.intensities = [0.0] * self.num_readings

        self.publisher.publish(msg)
        self.get_logger().info('Published fake laser scan.')


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
