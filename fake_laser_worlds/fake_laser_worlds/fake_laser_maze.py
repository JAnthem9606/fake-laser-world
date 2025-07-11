#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros
import random

class MazeFakeLaser(Node):
    def __init__(self):
        super().__init__('maze_fake_laser')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.send_static_tf()

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.get_logger().info('Maze fake laser scan node with multiple walls started.')

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
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            deg = math.degrees(angle)

            # Outer walls all around at 5m
            range_val = 5.0

            # Internal wall 1: 45° to 135° at 2m
            if 45 <= deg <= 135:
                range_val = 2.0
            # Internal wall 2: -135° to -90° at 3m
            elif -135 <= deg <= -90:
                range_val = 3.0
            # Internal wall 3: -45° to 0° at 1.5m
            elif -45 <= deg <= 0:
                range_val = 1.5

            ranges.append(range_val+random.gauss(0, 0.01))

        scan.ranges = ranges
        scan.intensities = [0.0] * self.num_readings

        self.publisher.publish(scan)
        self.get_logger().info('Publishing maze laser scan with multiple walls.')


def main(args=None):
    rclpy.init(args=args)
    node = MazeFakeLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
