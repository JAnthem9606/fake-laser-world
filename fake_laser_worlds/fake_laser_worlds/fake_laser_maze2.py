#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf2_ros
import random
import time

class EnhancedMazeFakeLaser(Node):
    def __init__(self):
        super().__init__('enhanced_maze_fake_laser')

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

        self.start_time = time.time()

        self.get_logger().info('Enhanced Maze Fake Laser Node Started.')

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

        current_time = time.time() - self.start_time
        moving_obj_angle = math.radians(-30 + (math.sin(current_time) * 30))  # Oscillates between -30° to +30°
        moving_obj_index = int((moving_obj_angle - self.angle_min) / self.angle_increment)

        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            deg = math.degrees(angle)

            # Default: outer corridor walls
            range_val = 6.0

            # Simulated front wall (from -15° to 15°)
            if -15 <= deg <= 15:
                range_val = 2.5

            # Simulated left wall (90° +/- 15°)
            elif 75 <= deg <= 105:
                range_val = 1.5

            # Simulated right wall (-90° +/- 15°)
            elif -105 <= deg <= -75:
                range_val = 1.5

            # Simulated rear wall (180° and -180°)
            elif deg >= 165 or deg <= -165:
                range_val = 3.5

            # Simulate a moving object in front
            if abs(i - moving_obj_index) <= 2:
                range_val = 1.0  # Closer distance

            noise = random.gauss(0, 0.02)
            ranges.append(max(self.range_min, min(range_val + noise, self.range_max)))

        scan.ranges = ranges
        scan.intensities = [0.0] * self.num_readings

        self.publisher.publish(scan)
        self.get_logger().info('Publishing enhanced maze laser scan with dynamic obstacle.')

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedMazeFakeLaser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
