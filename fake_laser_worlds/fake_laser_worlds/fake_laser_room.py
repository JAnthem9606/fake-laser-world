#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import tf2_ros


class SquareRoomLaserPublisher(Node):
    def __init__(self):
        super().__init__('square_room_laser_publisher')

        # Publisher and timer
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_square_scan)

        # TF: base_link -> laser_frame
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.broadcast_static_tf()

        # Laser parameters
        self.angle_min = -math.pi  # 360-degree LIDAR
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)  # 1 degree steps
        self.range_min = 0.1
        self.range_max = 10.0
        self.num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)

        self.room_half_size = 5.0  # Square from -5 to +5 in both x and y

        self.get_logger().info("Square room laser initialized.")

    def broadcast_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'laser_frame'

        # Sensor is at the center
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.2  # Slightly above ground

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def publish_square_scan(self):
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

        # Simulate laser hits on square room walls
        ranges = []
        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # normalize

            # Raycast to square wall
            dx = math.cos(angle)
            dy = math.sin(angle)

            # Avoid division by zero
            t_vals = []
            if dx != 0:
                t_vals.append(self.room_half_size / abs(dx))
            if dy != 0:
                t_vals.append(self.room_half_size / abs(dy))

            distance = min(t_vals)

            # Add small random noise
            noise = np.random.normal(0, 0.01)
            noisy_distance = min(self.range_max, max(self.range_min, distance + noise))
            ranges.append(noisy_distance)

        msg.ranges = ranges
        msg.intensities = [0.0] * self.num_readings

        self.publisher.publish(msg)
        self.get_logger().info('Published square room laser scan.')


def main(args=None):
    rclpy.init(args=args)
    node = SquareRoomLaserPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
