#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros


class EllipseFakeLaser(Node):
    def __init__(self):
        super().__init__('ellipse_fake_laser')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.send_static_tf()

        # Ellipse parameters (semi-major and semi-minor axes)
        self.a = 5.0  # along x-axis
        self.b = 4.0  # along y-axis

        # LaserScan parameters
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        # Precompute ellipse distances for all angles
        self.precomputed_ranges = self.compute_ellipse_ranges()

        self.get_logger().info('Ellipse fake laser scan node started.')

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

    def compute_ellipse_ranges(self):
        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            # Formula for radius of ellipse in polar coordinates:
            # r(θ) = (a*b) / sqrt((b*cos(θ))^2 + (a*sin(θ))^2)
            numerator = self.a * self.b
            denominator = math.sqrt((self.b * math.cos(angle))**2 + (self.a * math.sin(angle))**2)
            r = numerator / denominator

            # Clamp to sensor min/max range
            if r < self.range_min:
                r = self.range_min
            elif r > self.range_max:
                r = self.range_max

            ranges.append(r)
        return ranges

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

        scan.ranges = self.precomputed_ranges
        scan.intensities = [0.0] * self.num_readings

        self.publisher.publish(scan)
        self.get_logger().info('Publishing static ellipse laser scan.')


def main(args=None):
    rclpy.init(args=args)
    node = EllipseFakeLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
