#!/usr/bin/env python3
"""
intent_communication_node.py — Visual + audio + signal feedback per robot state.

Subscribes:
    /robot_state  (std_msgs/String)

Publishes:
    /intent/color     (std_msgs/ColorRGBA)        — RGB color for dashboard
    /intent/display   (std_msgs/String)           — short text for display screens
    /voice/announce   (std_msgs/String)           — full sentence for voice TTS
    /qbot_led_strip   (std_msgs/Float64MultiArray)— RGB to physical QBot LED strip
                                                    [leftR, leftG, leftB, rightR, rightG, rightB]

Optional onboard speech via `espeak` (set parameter enable_local_voice=true).
"""

import os
import subprocess
import shutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA, Float64MultiArray


# State -> (R, G, B, A, display_text, voice_sentence, ansi, marker)
STATE_INTENT = {
    'WAITING':         (1.0, 1.0, 1.0, 1.0,
                        'READY',
                        'Ready for delivery.',
                        '\033[97m', 'O'),
    'SET_DESTINATION': (1.0, 0.8, 0.0, 1.0,
                        'PREPARING',
                        'Destination received. Preparing to move.',
                        '\033[93m', '*'),
    'SIGNAL_MOVE':     (1.0, 0.8, 0.0, 1.0,
                        'ABOUT TO MOVE',
                        'Caution. I am about to move.',
                        '\033[93m', '!'),
    'MOVING':          (0.0, 1.0, 0.0, 1.0,
                        'MOVING',
                        'Moving to destination.',
                        '\033[92m', '>'),
    'YIELD_STOP':      (1.0, 0.0, 0.0, 1.0,
                        'YIELDING',
                        'Obstacle detected. Yielding. Please step aside.',
                        '\033[91m', 'X'),
    'DELIVERY_READY':  (0.0, 0.5, 1.0, 1.0,
                        'DELIVERY READY',
                        'I have arrived. Ready to deliver.',
                        '\033[94m', '#'),
    'DELIVER':         (1.0, 0.0, 1.0, 1.0,
                        'DELIVERING',
                        'Delivery in progress. Please take your item.',
                        '\033[95m', '$'),
    'RETURN_HOME':     (0.0, 1.0, 0.5, 1.0,
                        'RETURNING HOME',
                        'Delivery complete. Returning to base.',
                        '\033[96m', '<'),
    'ERROR':           (1.0, 0.0, 0.0, 1.0,
                        'ERROR',
                        'An error has occurred.',
                        '\033[91m', '!'),
}

ANSI_RESET = '\033[0m'


class IntentCommunication(Node):
    def __init__(self):
        super().__init__('intent_communication')

        # Topic parameters
        self.declare_parameter('state_topic',     '/robot_state')
        self.declare_parameter('color_topic',     '/intent/color')
        self.declare_parameter('display_topic',   '/intent/display')
        self.declare_parameter('voice_topic',     '/voice/announce')
        self.declare_parameter('led_strip_topic', '/qbot_led_strip')

        # Behavior parameters
        self.declare_parameter('enable_led',          True)
        self.declare_parameter('enable_local_voice',  False)  # set True if espeak is installed on bot

        state_topic     = self.get_parameter('state_topic').value
        color_topic     = self.get_parameter('color_topic').value
        display_topic   = self.get_parameter('display_topic').value
        voice_topic     = self.get_parameter('voice_topic').value
        led_strip_topic = self.get_parameter('led_strip_topic').value
        self.enable_led         = self.get_parameter('enable_led').value
        self.enable_local_voice = self.get_parameter('enable_local_voice').value

        self.create_subscription(String, state_topic, self.state_callback, 10)
        self.color_pub   = self.create_publisher(ColorRGBA,         color_topic,     10)
        self.display_pub = self.create_publisher(String,            display_topic,   10)
        self.voice_pub   = self.create_publisher(String,            voice_topic,     10)
        self.led_pub     = self.create_publisher(Float64MultiArray, led_strip_topic, 10)

        # Verify espeak exists if local voice requested
        self.espeak_path = shutil.which('espeak') if self.enable_local_voice else None
        if self.enable_local_voice and not self.espeak_path:
            self.get_logger().warn('enable_local_voice=true but espeak not found. Skipping local TTS.')
            self.enable_local_voice = False

        # Republish LED at 2Hz so strip stays lit
        self.create_timer(0.5, self._republish_leds)
        self._current_led = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.last_state = None

        self.get_logger().info(
            f'Intent Communication ready. LED:{self.enable_led} Voice:{self.enable_local_voice}')

    def state_callback(self, msg):
        state = msg.data
        if state == self.last_state:
            return
        self.last_state = state

        intent = STATE_INTENT.get(state)
        if intent is None:
            self.get_logger().warn(f'Unknown state: {state}')
            return

        r, g, b, a, display_text, voice_sentence, ansi, marker = intent

        # Publish color
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        self.color_pub.publish(c)

        # Publish short display
        disp = String()
        disp.data = display_text
        self.display_pub.publish(disp)

        # Publish voice sentence (dashboard listens, plus optional local TTS)
        v = String()
        v.data = voice_sentence
        self.voice_pub.publish(v)

        # Physical LED strip
        if self.enable_led:
            self._current_led = [r, g, b, r, g, b]
            led_msg = Float64MultiArray()
            led_msg.data = self._current_led
            self.led_pub.publish(led_msg)

        # Local TTS (non-blocking)
        if self.enable_local_voice and self.espeak_path:
            try:
                subprocess.Popen(
                    [self.espeak_path, '-s', '155', '-a', '180', voice_sentence],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
            except Exception as e:
                self.get_logger().warn(f'espeak failed: {e}')

        # Terminal banner
        print()
        print(f'  {ansi}+----------------------------------------+{ANSI_RESET}')
        print(f'  {ansi}|  [{marker}]  STATE: {state:<22s} |{ANSI_RESET}')
        print(f'  {ansi}|       -> {display_text:<29s} |{ANSI_RESET}')
        print(f'  {ansi}+----------------------------------------+{ANSI_RESET}')
        print()

    def _republish_leds(self):
        if not self.enable_led or self.last_state is None:
            return
        led_msg = Float64MultiArray()
        led_msg.data = self._current_led
        self.led_pub.publish(led_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IntentCommunication()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
