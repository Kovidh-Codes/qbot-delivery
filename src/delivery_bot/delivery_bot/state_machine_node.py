#!/usr/bin/env python3
"""Delivery Bot state machine.
States: WAITING -> ORDER_RECEIVED -> TRAVELING -> DELIVERED -> RETURNING -> RETURNED
TRAVELING: drives forward to delivery
RETURNING: drives REVERSE straight back to home (no U-turn)
"""
import math, json
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger


class State(Enum):
    WAITING        = 'WAITING'
    ORDER_RECEIVED = 'ORDER_RECEIVED'
    TRAVELING      = 'TRAVELING'
    DELIVERED      = 'DELIVERED'
    RETURNING      = 'RETURNING'
    RETURNED       = 'RETURNED'


class StateMachine(Node):
    def __init__(self):
        super().__init__('delivery_bot_state_machine')
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('steering_gain', 0.8)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('watchdog_timeout', 20.0)
        self.declare_parameter('waypoints_file', '/home/nvidia/qbot-delivery/waypoints.json')

        self.linear_speed     = self.get_parameter('linear_speed').value
        self.angular_speed    = self.get_parameter('angular_speed').value
        self.steering_gain    = self.get_parameter('steering_gain').value
        self.goal_tolerance   = self.get_parameter('goal_tolerance').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.waypoints_file   = self.get_parameter('waypoints_file').value

        self.home    = self._load_home()
        self.state   = State.WAITING
        self.pos     = (0.0, 0.0)
        self.heading = 0.0
        self.target  = None
        self._enter_time = self.get_clock().now()
        self._wd_last_dist = float('inf')
        self._wd_last_time = self.get_clock().now()

        self.cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Point,    '/destination', self.dest_cb, 10)
        self.create_service(Trigger, '/reload_waypoints', self.reload_cb)
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"State machine ready. State={self.state.value} home=({self.home[0]:.2f},{self.home[1]:.2f}) "
            f"lin={self.linear_speed} ang={self.angular_speed} tol={self.goal_tolerance}"
        )

    def _load_home(self):
        try:
            with open(self.waypoints_file) as f:
                data = json.load(f)
            h = data.get('home', {'x': 0.0, 'y': 0.0})
            self.get_logger().info(f"Loaded home: ({h['x']:.2f}, {h['y']:.2f})")
            return (h['x'], h['y'])
        except Exception as e:
            self.get_logger().warn(f"No waypoints ({e}), using (0,0)")
            return (0.0, 0.0)

    def reload_cb(self, req, resp):
        self.home = self._load_home()
        resp.success = True
        resp.message = f"Home: {self.home}"
        return resp

    def odom_cb(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.heading = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def dest_cb(self, msg):
        if self.state in (State.WAITING, State.RETURNED):
            self.target = (msg.x, msg.y)
            self.get_logger().info(f"Order received: ({msg.x:.2f}, {msg.y:.2f})")
            self._transition(State.ORDER_RECEIVED)

    def _transition(self, new):
        self.get_logger().info(f"  {self.state.value} -> {new.value}")
        self.state = new
        self._enter_time   = self.get_clock().now()
        self._wd_last_dist = float('inf')
        self._wd_last_time = self.get_clock().now()
        m = String(); m.data = new.value
        self.state_pub.publish(m)

    def _t_in_state(self):
        return (self.get_clock().now() - self._enter_time).nanoseconds / 1e9

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _drive_forward(self, target):
        """Drive forward toward target with mild steering correction."""
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        err = math.atan2(dy, dx) - self.heading
        while err > math.pi:  err -= 2*math.pi
        while err < -math.pi: err += 2*math.pi
        cmd = Twist()
        cmd.linear.x  = self.linear_speed
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, err * self.steering_gain))
        self.cmd_pub.publish(cmd)
        return dist

    def _drive_reverse(self, target):
        """Drive REVERSE straight back. No turning, no U-turn."""
        dist = math.hypot(target[0] - self.pos[0], target[1] - self.pos[1])
        cmd = Twist()
        cmd.linear.x = -self.linear_speed * 0.7  # slower in reverse for safety
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        return dist

    def _watchdog(self, dist):
        if dist < self._wd_last_dist - 0.05:
            self._wd_last_dist = dist
            self._wd_last_time = self.get_clock().now()
        elif (self.get_clock().now() - self._wd_last_time).nanoseconds / 1e9 > self.watchdog_timeout:
            self.get_logger().warn(f"Watchdog: no progress {self.watchdog_timeout}s, dist={dist:.2f}")
            self._wd_last_time = self.get_clock().now()

    def tick(self):
        if self.state == State.WAITING:
            self._stop()
        elif self.state == State.ORDER_RECEIVED:
            self._stop()
            if self._t_in_state() > 2.0:
                self._transition(State.TRAVELING)
        elif self.state == State.TRAVELING:
            dist = self._drive_forward(self.target)
            self._watchdog(dist)
            if dist < self.goal_tolerance:
                self._stop()
                self._transition(State.DELIVERED)
        elif self.state == State.DELIVERED:
            self._stop()
            if self._t_in_state() > 3.0:
                self._transition(State.RETURNING)
        elif self.state == State.RETURNING:
            dist = self._drive_reverse(self.home)
            self._watchdog(dist)
            if dist < self.goal_tolerance:
                self._stop()
                self._transition(State.RETURNED)
        elif self.state == State.RETURNED:
            self._stop()
            if self._t_in_state() > 2.0:
                self._transition(State.WAITING)


def main():
    rclpy.init()
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
