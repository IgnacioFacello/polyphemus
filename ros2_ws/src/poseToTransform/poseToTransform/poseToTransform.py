import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu_diy', self.cb, 10)

    def cb(self, msg):
        print("Message Received")
        t = TransformStamped()
        t.header = msg.header                 # frame_id = parent, e.g. 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    pose_node = PoseToTF()
    rclpy.spin(pose_node)
    pose_node.destroy_node()
    rclpy.shutdown()
