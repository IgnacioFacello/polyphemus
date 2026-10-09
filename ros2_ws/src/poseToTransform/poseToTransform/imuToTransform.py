#!/usr/bin/env python3
"""
Subscribes to a sensor_msgs/Imu that already contains the orientation
quaternion (computed on the ESP32) and broadcasts it as the TF
  odom -> imu_link
so RViz can rotate a model attached to imu_link.

Optional (integrate_position:=true): a *demo-grade* position estimate so the
model also moves. Linear acceleration is rotated into the world frame,
gravity is removed, and it is integrated with leaky damping + a deadband.
IMU-only position always drifts; the damping makes the model drift back to
the origin instead of running away. Use a real odometry source for accurate
translation.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

G = 9.80665


def rotate(q, v):
    """Rotate vector v by unit quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.declare_parameter('input_topic', '/imu_dmp')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'imu_link')
        self.declare_parameter('restamp', True)  # use PC clock instead of MCU stamp

        # Demo position estimate (off by default)
        self.declare_parameter('integrate_position', False)
        self.declare_parameter('gravity_removed', False)  # True if accel has no gravity
        self.declare_parameter('accel_deadband', 0.08)    # m/s^2
        self.declare_parameter('vel_decay', 2.0)          # 1/s, velocity damping
        self.declare_parameter('pos_decay', 0.3)          # 1/s, pull back to origin

        p = self.get_parameter
        self.parent = p('parent_frame').value
        self.child = p('child_frame').value
        self.restamp = p('restamp').value
        self.integrate = p('integrate_position').value
        self.gravity_removed = p('gravity_removed').value
        self.deadband = p('accel_deadband').value
        self.vel_decay = p('vel_decay').value
        self.pos_decay = p('pos_decay').value

        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.last_t = None

        self.br = TransformBroadcaster(self)
        topic = p('input_topic').value
        self.create_subscription(Imu, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'{topic} -> TF {self.parent} -> {self.child} '
            f'(position integration: {self.integrate})')

    def cb(self, msg: Imu):
        o = msg.orientation
        n = math.sqrt(o.x ** 2 + o.y ** 2 + o.z ** 2 + o.w ** 2)
        if n < 1e-6:
            self.get_logger().warn('Received a zero quaternion, ignoring',
                                   throttle_duration_sec=2.0)
            return
        q = (o.x / n, o.y / n, o.z / n, o.w / n)

        stamp = (self.get_clock().now().to_msg() if self.restamp
                 else msg.header.stamp)
        t = stamp.sec + stamp.nanosec * 1e-9

        if self.integrate:
            self.integrate_step(q, msg, t)
        self.last_t = t

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.parent
        tf.child_frame_id = self.child
        tf.transform.translation.x = self.pos[0]
        tf.transform.translation.y = self.pos[1]
        tf.transform.translation.z = self.pos[2]
        tf.transform.rotation.x, tf.transform.rotation.y = q[0], q[1]
        tf.transform.rotation.z, tf.transform.rotation.w = q[2], q[3]
        self.br.sendTransform(tf)

    def integrate_step(self, q, msg, t):
        if self.last_t is None:
            return
        dt = t - self.last_t
        if dt <= 0.0 or dt > 0.5:
            return

        a = msg.linear_acceleration
        aw = list(rotate(q, (a.x, a.y, a.z)))  # body -> world
        if not self.gravity_removed:
            aw[2] -= G

        mag = math.sqrt(sum(c * c for c in aw))
        if mag < self.deadband:
            aw = [0.0, 0.0, 0.0]
            vd = self.vel_decay * 5.0  # stronger damping when (almost) still
        else:
            vd = self.vel_decay

        kv = math.exp(-vd * dt)
        kp = math.exp(-self.pos_decay * dt)
        for i in range(3):
            self.vel[i] = (self.vel[i] + aw[i] * dt) * kv
            self.pos[i] = (self.pos[i] + self.vel[i] * dt) * kp


def main():
    rclpy.init()
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
