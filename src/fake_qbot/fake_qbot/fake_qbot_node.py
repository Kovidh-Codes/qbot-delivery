#!/usr/bin/env python3
"""
fake_qbot_node.py — Offline QBot simulator with obstacle injection.

Subscribes:
    /cmd_vel  (geometry_msgs/Twist)

Publishes:
    /odom  (nav_msgs/Odometry)
    /scan  (sensor_msgs/LaserScan)

Inject a fake obstacle in front:
    ros2 param set /fake_qbot front_obstacle_distance 0.3
Clear it:
    ros2 param set /fake_qbot front_obstacle_distance 5.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class FakeQBot(Node):
    def __init__(self):
        super().__init__('fake_qbot')

        self.declare_parameter('front_obstacle_distance', 5.0)

        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.vx = 0.0; self.wz = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry,  '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.dt = 0.05
        self.create_timer(self.dt, self.update)

        self.scan_default_range = 5.0
        self.scan_num_rays = 360

        self.get_logger().info('Fake QBot started. Listening on /cmd_vel')

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update(self):
        self.theta += self.wz * self.dt
        self.x     += self.vx * math.cos(self.theta) * self.dt
        self.y     += self.vx * math.sin(self.theta) * self.dt
        self.publish_odom()
        self.publish_scan()

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        msg.twist.twist.linear.x  = self.vx
        msg.twist.twist.angular.z = self.wz
        self.odom_pub.publish(msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = -math.pi
        msg.angle_max =  math.pi
        msg.angle_increment = (2 * math.pi) / self.scan_num_rays
        msg.range_min = 0.1
        msg.range_max = 10.0

        ranges = [self.scan_default_range] * self.scan_num_rays
        front_dist = self.get_parameter('front_obstacle_distance').value
        # Inject obstacle in front cone (~60 degrees)
        front = self.scan_num_rays // 2
        window = self.scan_num_rays // 12
        for i in range(front - window, front + window):
            ranges[i] = front_dist
        msg.ranges = ranges
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeQBot()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
