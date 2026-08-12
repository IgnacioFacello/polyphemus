#!/usr/bin/env python3
"""
encoder_monitor_node.py — ROS2 Jazzy node that subscribes to the encoder
position and RPM topics published by the ESP32 micro-ROS firmware
(main.c), prints the latest values, plots their history live in two
matplotlib graphs, and periodically writes the full (unbounded) history
to a JSON file on disk.

Topics (must match main.c):
    angle_data  (std_msgs/Float32)    — raw pulse count / position
    rpm_data     (std_msgs/Float32)  — computed RPM

Parameters:
    history_file      (string, default "encoder_history.json")
                       Path to the JSON file the history is written to.
    save_interval_sec (double, default 5.0)
                       How often the JSON file is rewritten while running.
"""

import json
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32

HISTORY_LEN = 200          # number of points kept on each graph
POSITION_TOPIC = "angle_data"
RPM_TOPIC = "rpm_data"


class EncoderMonitor(Node):
    def __init__(self):
        super().__init__("encoder_monitor")

        self.declare_parameter("history_file", "encoder_history.json")
        self.declare_parameter("save_interval_sec", 5.0)
        self.history_file = self.get_parameter("history_file").value
        save_interval = self.get_parameter("save_interval_sec").value

        # micro-ROS default publishers are typically best-effort; match that
        # here or messages may silently never arrive.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.lock = threading.Lock()

        # Bounded history used only for the live plots.
        self.position_hist = deque(maxlen=HISTORY_LEN)
        self.rpm_hist = deque(maxlen=HISTORY_LEN)

        # Unbounded, timestamped history written out to JSON.
        self._start_time = time.time()
        self.position_log = []   # [{"t": elapsed_sec, "value": ticks}, ...]
        self.rpm_log = []        # [{"t": elapsed_sec, "value": rpm}, ...]

        self.create_subscription(Float32, POSITION_TOPIC, self._position_cb, qos)
        self.create_subscription(Float32, RPM_TOPIC, self._rpm_cb, qos)

        # Periodically flush history to disk so you don't lose everything
        # if the process is killed instead of shut down cleanly.
        self.create_timer(save_interval, self._save_history)

        self.get_logger().info(
            f"Listening on '{POSITION_TOPIC}' (Float32) and '{RPM_TOPIC}' (Float32); "
            f"writing history to '{self.history_file}' every {save_interval:.1f}s"
        )

    def _position_cb(self, msg: Float32):
        with self.lock:
            self.position_hist.append(msg.data)
            self.position_log.append(
                {"t": round(time.time() - self._start_time, 3), "value": msg.data}
            )
        print(f"Position: {msg.data} ticks")

    def _rpm_cb(self, msg: Float32):
        with self.lock:
            self.rpm_hist.append(msg.data)
            self.rpm_log.append(
                {"t": round(time.time() - self._start_time, 3), "value": msg.data}
            )
        print(f"RPM: {msg.data:.2f}")

    def _save_history(self):
        with self.lock:
            payload = {
                "position": list(self.position_log),
                "rpm": list(self.rpm_log),
            }
        try:
            with open(self.history_file, "w") as f:
                json.dump(payload, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f"Failed to write history file: {e}")


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
        node._save_history()   # capture whatever happened since the last periodic save
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
