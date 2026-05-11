#!/usr/bin/env python3
"""Save current /odom position as a named waypoint.

Usage:
    python3 save_wp.py <name>
    python3 save_wp.py home
    python3 save_wp.py teacher_desk
"""
import json
import os
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

WP_FILE = os.path.expanduser('~/qbot-delivery/waypoints.json')


class WaypointSaver(Node):
    def __init__(self, name):
        super().__init__('save_waypoint')
        self.name = name
        self.done = False
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        print(f'Recording "{name}"... waiting for /odom')

    def cb(self, msg):
        if self.done:
            return
        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)
        data = {}
        if os.path.exists(WP_FILE):
            try:
                with open(WP_FILE) as f:
                    data = json.load(f)
            except Exception:
                data = {}
        data[self.name] = {'x': x, 'y': y}
        os.makedirs(os.path.dirname(WP_FILE), exist_ok=True)
        with open(WP_FILE, 'w') as f:
            json.dump(data, f, indent=2)
        print(f'\n[OK] SAVED "{self.name}" at ({x}, {y})\n')
        self.done = True


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 save_wp.py <name>')
        sys.exit(1)
    rclpy.init()
    node = WaypointSaver(sys.argv[1])
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
