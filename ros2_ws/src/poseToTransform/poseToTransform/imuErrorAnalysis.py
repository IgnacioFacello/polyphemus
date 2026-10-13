#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


def quaternion_error_deg(q1, q2):
    """
    Calcula la diferencia angular entre dos cuaterniones.

    q = (x, y, z, w)

    error = 2 * acos(|q1 dot q2|)
    """

    n1 = math.sqrt(
        q1.x * q1.x +
        q1.y * q1.y +
        q1.z * q1.z +
        q1.w * q1.w
    )

    n2 = math.sqrt(
        q2.x * q2.x +
        q2.y * q2.y +
        q2.z * q2.z +
        q2.w * q2.w
    )

    if n1 < 1e-6 or n2 < 1e-6:
        return None

    dot = (
        (q1.x / n1) * (q2.x / n2) +
        (q1.y / n1) * (q2.y / n2) +
        (q1.z / n1) * (q2.z / n2) +
        (q1.w / n1) * (q2.w / n2)
    )

    # q y -q representan la misma orientación
    dot = abs(dot)

    # Protección numérica para acos()
    dot = min(1.0, max(-1.0, dot))

    error_rad = 2.0 * math.acos(dot)

    return math.degrees(error_rad)


class ImuErrorAnalysis(Node):

    def __init__(self):
        super().__init__('imu_error_analysis')

        self.declare_parameter('dmp_topic', '/imu_dmp')
        self.declare_parameter('gyro_topic', '/imu_gyro')
        self.declare_parameter('filter_topic', '/imu_diy')

        dmp_topic = self.get_parameter('dmp_topic').value
        gyro_topic = self.get_parameter('gyro_topic').value
        filter_topic = self.get_parameter('filter_topic').value

        self.q_dmp = None

        self.error_gyro_pub = self.create_publisher(
            Float32,
            '/error_gyro_deg',
            10
        )

        self.error_filter_pub = self.create_publisher(
            Float32,
            '/error_filter_deg',
            10
        )

        self.create_subscription(
            Imu,
            dmp_topic,
            self.dmp_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Imu,
            gyro_topic,
            self.gyro_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Imu,
            filter_topic,
            self.filter_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Comparando {dmp_topic} con '
            f'{gyro_topic} y {filter_topic}'
        )

    def dmp_callback(self, msg):
        self.q_dmp = msg.orientation

    def gyro_callback(self, msg):
        if self.q_dmp is None:
            return

        error = quaternion_error_deg(
            self.q_dmp,
            msg.orientation
        )

        if error is None:
            return

        out = Float32()
        out.data = float(error)

        self.error_gyro_pub.publish(out)

    def filter_callback(self, msg):
        if self.q_dmp is None:
            return

        error = quaternion_error_deg(
            self.q_dmp,
            msg.orientation
        )

        if error is None:
            return

        out = Float32()
        out.data = float(error)

        self.error_filter_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = ImuErrorAnalysis()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
