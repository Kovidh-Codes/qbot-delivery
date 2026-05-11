#!/usr/bin/env python3
"""Send the robot to a named waypoint by publishing to /destination.

Usage:
    python3 goto_wp.py <name>
    python3 goto_wp.py teacher_desk

List available waypoints:
    python3 goto_wp.py
"""
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Point

WP_FILE = os.path.expanduser('~/qbot-delivery/waypoints.json')


def load_waypoints():
    if not os.path.exists(WP_FILE):
        return None
    try:
        with open(WP_FILE) as f:
            return json.load(f)
    except Exception:
        return None


def main():
    wp = load_waypoints()
    if not wp:
        print(f'No waypoints saved yet. Run save_wp.py first.')
        sys.exit(1)

    if len(sys.argv) < 2:
        print('Available waypoints:')
        for name, coords in wp.items():
            print(f'  {name}: ({coords["x"]}, {coords["y"]})')
        print('\nUsage: python3 goto_wp.py <name>')
        sys.exit(0)

    name = sys.argv[1]
    if name not in wp:
        print(f'Unknown waypoint "{name}". Available: {list(wp.keys())}')
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node('goto_waypoint')
    pub = node.create_publisher(Point, '/destination', 10)
    time.sleep(1.0)  # let publisher register
    msg = Point(x=float(wp[name]['x']), y=float(wp[name]['y']), z=0.0)
    pub.publish(msg)
    print(f'[OK] Sent "{name}" -> ({msg.x}, {msg.y})')
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
