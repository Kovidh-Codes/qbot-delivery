#!/usr/bin/env python3
"""Compute /odom from /qbot_speed_feedback by integrating velocity over time.
Needed on QBots where the driver_interface doesn't publish /odom directly.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


class OdomFromSpeed(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_t = None
        self.create_subscription(TwistStamped, '/qbot_speed_feedback', self.cb, 50)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.get_logger().info('Odom publisher started. Integrating /qbot_speed_feedback -> /odom')

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        if dt <= 0 or dt > 1.0:  # ignore bad timestamps
            self.last_t = t
            return
        self.last_t = t

        v = msg.twist.linear.x
        w = msg.twist.angular.z

        # Integrate
        self.theta += w * dt
        # Normalize
        while self.theta > math.pi:  self.theta -= 2 * math.pi
        while self.theta < -math.pi: self.theta += 2 * math.pi
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist = msg.twist
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = OdomFromSpeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
