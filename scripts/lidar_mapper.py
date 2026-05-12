#!/usr/bin/env python3
"""
Simple 2D occupancy grid mapper from /scan + /odom.

Builds a map as the bot drives. Saves to PNG every 2 seconds.
View the map: open ~/qbot-delivery/map.png

Usage:
    python3 lidar_mapper.py
"""
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# --- Map configuration -----------------------------------------------
GRID_SIZE = 400        # cells per side (400 x 400)
RESOLUTION = 0.05      # meters per cell (5 cm) -> total map = 20m x 20m
ORIGIN_X = GRID_SIZE // 2  # bot starts at center
ORIGIN_Y = GRID_SIZE // 2

# Cell state values
UNKNOWN  = 128   # gray
FREE     = 255   # white
OCCUPIED = 0     # black

OUTPUT_PATH = os.path.expanduser('~/qbot-delivery/map.png')


class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        self.grid = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=np.uint8)
        self.bot_x = 0.0
        self.bot_y = 0.0
        self.bot_theta = 0.0
        self.scan_count = 0

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_timer(2.0, self.save_map)

        self.get_logger().info(
            f'Mapper started. Map: {GRID_SIZE}x{GRID_SIZE} @ {RESOLUTION}m/cell. '
            f'Saves to {OUTPUT_PATH}'
        )

    def odom_cb(self, msg):
        self.bot_x = msg.pose.pose.position.x
        self.bot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.bot_theta = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))

    def world_to_grid(self, x, y):
        gx = int(ORIGIN_X + x / RESOLUTION)
        gy = int(ORIGIN_Y - y / RESOLUTION)  # flip Y for image coords
        return gx, gy

    def scan_cb(self, msg):
        if msg.ranges is None or len(msg.ranges) == 0:
            return

        bot_gx, bot_gy = self.world_to_grid(self.bot_x, self.bot_y)
        if not (0 <= bot_gx < GRID_SIZE and 0 <= bot_gy < GRID_SIZE):
            return

        angle = msg.angle_min
        ainc  = msg.angle_increment
        rmin  = msg.range_min
        rmax  = msg.range_max

        for r in msg.ranges:
            if r < rmin or r > rmax or math.isinf(r) or math.isnan(r):
                angle += ainc
                continue
            # World coords of the laser hit point
            wx = self.bot_x + r * math.cos(self.bot_theta + angle)
            wy = self.bot_y + r * math.sin(self.bot_theta + angle)
            ex, ey = self.world_to_grid(wx, wy)

            # Mark free space along the ray (Bresenham line)
            self.mark_free(bot_gx, bot_gy, ex, ey)
            # Mark the end point as occupied
            if 0 <= ex < GRID_SIZE and 0 <= ey < GRID_SIZE:
                self.grid[ey, ex] = OCCUPIED

            angle += ainc

        self.scan_count += 1

    def mark_free(self, x0, y0, x1, y1):
        """Bresenham line: mark all cells between (x0,y0) and (x1,y1) as FREE."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        # Don't paint the very last cell (that's the obstacle)
        while (x, y) != (x1, y1):
            if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
                # Only mark UNKNOWN cells as FREE (don't erase obstacles)
                if self.grid[y, x] == UNKNOWN:
                    self.grid[y, x] = FREE
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def save_map(self):
        try:
            # Add a small dot at robot position
            img = self.grid.copy()
            bot_gx, bot_gy = self.world_to_grid(self.bot_x, self.bot_y)
            if 0 <= bot_gx < GRID_SIZE and 0 <= bot_gy < GRID_SIZE:
                # Draw small cross at bot position
                for dx in range(-2, 3):
                    for dy in range(-2, 3):
                        nx, ny = bot_gx + dx, bot_gy + dy
                        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
                            img[ny, nx] = 64  # dark gray
            # Save as PGM (no PIL needed)
            with open(OUTPUT_PATH.replace('.png', '.pgm'), 'wb') as f:
                f.write(f'P5\n{GRID_SIZE} {GRID_SIZE}\n255\n'.encode())
                f.write(img.tobytes())
            # Also try PNG if PIL available
            try:
                from PIL import Image
                Image.fromarray(img).save(OUTPUT_PATH)
            except ImportError:
                pass
            self.get_logger().info(
                f'Map saved (scans: {self.scan_count}, bot@({self.bot_x:.2f},{self.bot_y:.2f}))'
            )
        except Exception as e:
            self.get_logger().warn(f'Save failed: {e}')


def main():
    rclpy.init()
    node = LidarMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
