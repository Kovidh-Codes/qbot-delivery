#!/usr/bin/env python3
import sys, json, os, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
WP = os.path.expanduser('~/qbot-delivery/waypoints.json')

class S(Node):
    def __init__(self, name):
        super().__init__('save_wp')
        self.name = name
        self.done = False
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        print(f'Recording "{name}"... waiting for odom')
    def cb(self, msg):
        if self.done: return
        x = round(msg.pose.pose.position.x, 3)
        y = round(msg.pose.pose.position.y, 3)
        wp = {}
        if os.path.exists(WP):
            with open(WP) as f: wp = json.load(f)
        wp[self.name] = {'x': x, 'y': y}
        with open(WP, 'w') as f: json.dump(wp, f, indent=2)
        print(f'\n✅ SAVED "{self.name}" at ({x}, {y})\n')
        self.done = True

if len(sys.argv) < 2: print('Usage: python3 save_wp.py <name>'); sys.exit(1)
rclpy.init()
n = S(sys.argv[1])
while rclpy.ok() and not n.done: rclpy.spin_once(n, timeout_sec=0.1)
rclpy.shutdown()
