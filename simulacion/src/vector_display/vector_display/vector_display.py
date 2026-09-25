#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib
matplotlib.use('TkAgg')  # or 'Qt5Agg' if you have PyQt5 installed
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import numpy as np
from threading import Lock


class VectorDisplay(Node):
    def __init__(self):
        super().__init__('vector_display')

        # --- State shared with callbacks ---
        self.lock = Lock()
        self.input_data = None    # [x, y, z, alpha, beta, phi, N]
        self.rotated_data = None  # [x1, y1, z1, x2, y2, z2]

        # --- Subscribers ---
        self.sub_input = self.create_subscription(
            Float32MultiArray, 'rotation_input', self.input_callback, 10)
        self.sub_rotated = self.create_subscription(
            Float32MultiArray, 'vector_rotado', self.rotated_callback, 10)

        # --- Figure setup ---
        plt.ion()
        self.fig = plt.figure(figsize=(11, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.suptitle('Vector Rotation Display', fontsize=13)

        # Text box for numeric values
        self.text_artist = self.fig.text(
            0.02, 0.02, '', fontsize=9, family='monospace',
            verticalalignment='bottom')

        self.fig.canvas.mpl_connect('close_event', self._on_close)
        self._closed = False

        # --- Update the plot periodically ---
        self.timer = self.create_timer(0.1, self.update_plot)

        self.get_logger().info('VectorDisplay node started. Waiting for data...')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def input_callback(self, msg):
        with self.lock:
            self.get_logger().info("Input data recieved")
            self.input_data = list(msg.data)

    def rotated_callback(self, msg):
        with self.lock:
            self.get_logger().info("Output data recieved")
            self.rotated_data = list(msg.data)

    def _on_close(self, _event):
        self._closed = True

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------
    def update_plot(self):
        if self._closed:
            return

        with self.lock:
            inp = self.input_data
            rot = self.rotated_data

        if inp is None or rot is None:
            return

        # Defensive parsing ------------------------------------------------
        try:
            x, y, z = inp[0], inp[1], inp[2]
            alpha, beta, phi = inp[3], inp[4], inp[5]
            N = int(inp[6])
        except (IndexError, ValueError):
            self.get_logger().warn('rotation_input has unexpected format')
            return

        try:
            x1, y1, z1 = rot[0], rot[1], rot[2]   # exact rotation
            x2, y2, z2 = rot[3], rot[4], rot[5]   # micro rotation
        except IndexError:
            self.get_logger().warn('vector_rotado has unexpected format')
            return

        self.get_logger().info("Updating Plot")

        # Redraw ----------------------------------------------------------
        self.ax.clear()

        # Arrows from the origin
        self.ax.quiver(0, 0, 0, x, y, z,
                       color='royalblue', arrow_length_ratio=0.12,
                       linewidth=2, label='Original  v')
        self.ax.quiver(0, 0, 0, x1, y1, z1,
                       color='forestgreen', arrow_length_ratio=0.12,
                       linewidth=2, label='Exact rotation')
        self.ax.quiver(0, 0, 0, x2, y2, z2,
                       color='crimson', arrow_length_ratio=0.12,
                       linewidth=2, label='Micro rotations')

        # Auto-scale (keeps a cube aspect)
        coords = [x, y, z, x1, y1, z1, x2, y2, z2, 0.0]
        max_val = max(1.0, max(abs(c) for c in coords))
        self.ax.set_xlim(-max_val, max_val)
        self.ax.set_ylim(-max_val, max_val)
        self.ax.set_zlim(-max_val, max_val)
        self.ax.set_box_aspect((1, 1, 1))

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title(
            f'alpha = {alpha:.2f}°, beta = {beta:.2f}°, '
            f'phi = {phi:.2f}°, N = {N}')
        self.ax.legend(loc='upper right', fontsize=9)
        self.ax.grid(True)

        # Numeric text panel ---------------------------------------------
        err = np.linalg.norm(
            np.array([x1, y1, z1]) - np.array([x2, y2, z2]))

        text = (
            f"Input vector      : ({x: .4f}, {y: .4f}, {z: .4f})\n"
            f"Exact rotation    : ({x1: .4f}, {y1: .4f}, {z1: .4f})\n"
            f"Micro rotations   : ({x2: .4f}, {y2: .4f}, {z2: .4f})\n"
            f"|difference|      : {err:.6e}\n"
            f"Angles (deg)      : alpha={alpha:.3f}, beta={beta:.3f}, phi={phi:.3f}\n"
            f"Micro steps N     : {N}"
        )
        self.text_artist.set_text(text)

        # Refresh ---------------------------------------------------------
        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception:
            # Figure was closed by the user
            self._closed = True


def main(args=None):
    rclpy.init(args=args)
    node = VectorDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
