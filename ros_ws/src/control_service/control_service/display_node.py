#!/usr/bin/env python3
"""
encoder_visualizer_node.py

Subscribes to a topic publishing raw encoder pulse counts and displays the
encoder position as a rotating indicator on a circular dial using pygame.

Assumptions (override via ROS parameters if they don't match your setup):
  - Topic: "sensor_data", message type: std_msgs/msg/Int32
  - Counts per revolution (CPR): 100

Run:
    python3 encoder_visualizer_node.py --ros-args -p topic:=sensor_data -p counts_per_rev:=360

Dependencies:
    pip install pygame
"""

import math
import sys
import threading

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

WINDOW_SIZE = 500
CENTER = (WINDOW_SIZE // 2, WINDOW_SIZE // 2)
RADIUS = 200
BG_COLOR = (20, 20, 24)
DIAL_COLOR = (80, 80, 90)
INDICATOR_COLOR = (0, 200, 120)
TEXT_COLOR = (230, 230, 230)


class EncoderVisualizerNode(Node):
    def __init__(self):
        super().__init__('encoder_visualizer_node')

        self.declare_parameter('topic', 'sensor_data')
        self.declare_parameter('counts_per_rev', 100)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.counts_per_rev = (
            self.get_parameter('counts_per_rev').get_parameter_value().integer_value
        )
        if self.counts_per_rev <= 0:
            self.counts_per_rev = 100  # guard against a bad param value

        self._lock = threading.Lock()
        self._count = 0

        self.subscription = self.create_subscription(
            Int32,
            topic,
            self._on_sensor_data,
            10,
        )
        self.get_logger().info(
            f"Listening on '{topic}' (std_msgs/Int32), counts_per_rev={self.counts_per_rev}"
        )

    def _on_sensor_data(self, msg: Int32):
        with self._lock:
            self._count = msg.data

    def get_count(self) -> int:
        with self._lock:
            return self._count


def run_ros_spin(node: Node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = EncoderVisualizerNode()

    ros_thread = threading.Thread(target=run_ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Encoder Position")
    font = pygame.font.SysFont(None, 28)
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        count = node.get_count()
        angle_deg = (count % node.counts_per_rev) / node.counts_per_rev * 360.0
        angle_rad = math.radians(angle_deg - 90)  # -90 so count 0 points up

        screen.fill(BG_COLOR)

        # dial outline
        pygame.draw.circle(screen, DIAL_COLOR, CENTER, RADIUS, width=4)

        # tick marks every 30 degrees
        for tick_deg in range(0, 360, 30):
            tick_rad = math.radians(tick_deg - 90)
            inner = (
                CENTER[0] + (RADIUS - 15) * math.cos(tick_rad),
                CENTER[1] + (RADIUS - 15) * math.sin(tick_rad),
            )
            outer = (
                CENTER[0] + RADIUS * math.cos(tick_rad),
                CENTER[1] + RADIUS * math.sin(tick_rad),
            )
            pygame.draw.line(screen, DIAL_COLOR, inner, outer, 2)

        # position indicator: line from center to current angle
        tip = (
            CENTER[0] + RADIUS * math.cos(angle_rad),
            CENTER[1] + RADIUS * math.sin(angle_rad),
        )
        pygame.draw.line(screen, INDICATOR_COLOR, CENTER, tip, 5)
        pygame.draw.circle(screen, INDICATOR_COLOR, CENTER, 8)

        # readout
        text = font.render(f"count: {count}   angle: {angle_deg:.1f} deg", True, TEXT_COLOR)
        screen.blit(text, (10, WINDOW_SIZE - 30))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
