#!/usr/bin/env python3
import sys, json, os, time, rclpy
from geometry_msgs.msg import Point
WP = os.path.expanduser('~/qbot-delivery/waypoints.json')

if len(sys.argv) < 2:
    print('Usage: python3 goto_wp.py <name>')
    if os.path.exists(WP):
        with open(WP) as f: print('Available:', list(json.load(f).keys()))
    sys.exit(1)
name = sys.argv[1]
with open(WP) as f: wp = json.load(f)
if name not in wp:
    print(f'Unknown: {name}. Have: {list(wp.keys())}')
    sys.exit(1)
rclpy.init()
n = rclpy.create_node('goto')
p = n.create_publisher(Point, '/destination', 10)
time.sleep(1)
m = Point(x=float(wp[name]['x']), y=float(wp[name]['y']), z=0.0)
p.publish(m)
print(f'✅ Sent "{name}" -> ({m.x}, {m.y})')
time.sleep(0.5)
rclpy.shutdown()
