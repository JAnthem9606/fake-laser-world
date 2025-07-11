#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros
import numpy as np


class FakeLaserCorridor(Node):
    def __init__(self):
        super().__init__('fake_laser_corridor')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.send_static_tf()

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(0.5)
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.wall_distance = 1.5  # meters to left/right walls
        self.front_wall_distance = 5.0  # distance to front wall (dead-end)

        self.get_logger().info("Corridor fake laser scan node started.")

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

        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            dx = math.cos(angle)
            dy = math.sin(angle)

            distance = self.range_max

            # Left wall: 90 ± 20 deg
            if 70 <= math.degrees(angle) <= 110:
                if dx != 0:
                    distance = abs(self.wall_distance / dx)

            # Right wall: -90 ± 20 deg
            elif -110 <= math.degrees(angle) <= -70:
                if dx != 0:
                    distance = abs(self.wall_distance / dx)

            # Front wall: -15 to +15 deg
            elif -15 <= math.degrees(angle) <= 15:
                if dy != 0:
                    distance = abs(self.front_wall_distance / dy)

            # Add noise
            noise = np.random.normal(0, 0.01)
            noisy_distance = min(self.range_max, max(self.range_min, distance + noise))
            ranges.append(noisy_distance)

        msg.ranges = ranges
        msg.intensities = [0.0] * self.num_readings

        self.publisher.publish(msg)
        self.get_logger().info('Published fake laser corridor scan.')


def main(args=None):
    rclpy.init(args=args)
    node = FakeLaserCorridor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
