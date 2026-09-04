#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState


class SensorToJointState(Node):
    def __init__(self):
        super().__init__('sensor_to_joint_state')

        self.encoder = 0.0
        self.potentiometer = 0.0

        self.create_subscription(
            Float32,
            '/polyphemus/encoder',
            self.encoder_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/polyphemus/potentiometer',
            self.potentiometer_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.create_timer(0.1, self.publish_joint_state)

    def encoder_callback(self, message):
        self.encoder = message.data

    def potentiometer_callback(self, message):
        self.potentiometer = message.data

    def publish_joint_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [
            'body_u_joint',
            'camera_lens_joint'
        ]
        message.position = [
            self.encoder,
            self.potentiometer
        ]

        self.publisher.publish(message)


def main():
    rclpy.init()
    node = SensorToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()