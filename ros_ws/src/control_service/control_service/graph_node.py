#!/usr/bin/env python3
"""
encoder_monitor_node.py — ROS2 Jazzy node that subscribes to the encoder
position and RPM topics published by the ESP32 micro-ROS firmware
(main.c), prints the latest values, and plots their history live in
two matplotlib graphs.

Topics (must match main.c):
    angle_data  (std_msgs/Int32)    — raw pulse count / position
    rpm_data     (std_msgs/Float32)  — computed RPM
"""

import threading
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32, Float32

HISTORY_LEN = 200          # number of points kept on each graph
POSITION_TOPIC = "angle_data"
RPM_TOPIC = "rpm_data"


class EncoderMonitor(Node):
    def __init__(self):
        super().__init__("encoder_monitor")

        # micro-ROS default publishers are typically best-effort; match that
        # here or messages may silently never arrive.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.lock = threading.Lock()
        self.position_hist = deque(maxlen=HISTORY_LEN)
        self.rpm_hist = deque(maxlen=HISTORY_LEN)

        self.create_subscription(Int32, POSITION_TOPIC, self._position_cb, qos)
        self.create_subscription(Float32, RPM_TOPIC, self._rpm_cb, qos)

        self.get_logger().info(
            f"Listening on '{POSITION_TOPIC}' (Int32) and '{RPM_TOPIC}' (Float32)"
        )

    def _position_cb(self, msg: Int32):
        with self.lock:
            self.position_hist.append(msg.data)
        print(f"Position: {msg.data} ticks")

    def _rpm_cb(self, msg: Float32):
        with self.lock:
            self.rpm_hist.append(msg.data)
        print(f"RPM: {msg.data:.2f}")


def _ros_spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderMonitor()

    # Spin ROS in a background thread so matplotlib's event loop can own
    # the main thread (required on most platforms/backends).
    spin_thread = threading.Thread(target=_ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    fig, (ax_pos, ax_rpm) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    fig.suptitle("Encoder Live Monitor")

    ax_pos.set_ylabel("Position (ticks)")
    ax_pos.grid(True, alpha=0.3)
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_xlabel("Sample #")
    ax_rpm.grid(True, alpha=0.3)

    (line_pos,) = ax_pos.plot([], [], color="tab:blue")
    (line_rpm,) = ax_rpm.plot([], [], color="tab:orange")

    def update(_frame):
        with node.lock:
            pos_data = list(node.position_hist)
            rpm_data = list(node.rpm_hist)

        line_pos.set_data(range(len(pos_data)), pos_data)
        line_rpm.set_data(range(len(rpm_data)), rpm_data)

        for ax, data in ((ax_pos, pos_data), (ax_rpm, rpm_data)):
            if data:
                ax.relim()
                ax.autoscale_view()

        return line_pos, line_rpm

    ani = animation.FuncAnimation(fig, update, interval=200, blit=False)

    try:
        plt.show()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
