"""
Usage (micro-ROS agent already running and the ESP32 publishing an Imu with orientation):
  ros2 launch imu_tf_rviz.launch.py imu_topic:=/esp32/imu
  ros2 launch imu_tf_rviz.launch.py imu_topic:=/esp32/imu integrate_position:=true

Starts:
  - imu_tf_broadcaster.py   (Imu orientation -> TF odom -> imu_link)
  - robot_state_publisher   (URDF: a box attached to imu_link)
  - rviz2                   (fixed frame = odom)

Keep this file in the same folder as imu_tf_broadcaster.py.
Do NOT run imu_filter_madgwick at the same time: it would publish the same TF.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))

BLUE_URDF = """
<robot name="imu_box">
  <link name="imu_dmp_link">
    <visual>
      <geometry><box size="0.20 0.10 0.02"/></geometry>
      <material name="blue"><color rgba="0.1 0.3 0.9 1.0"/></material>
    </visual>
  </link>
</robot>
"""

RED_URDF = """
<robot name="imu_box">
  <link name="imu_diy_link">
    <visual>
      <geometry><box size="0.20 0.10 0.02"/></geometry>
      <material name="red"><color rgba="0.9 0.3 0.1 1.0"/></material>
    </visual>
  </link>
</robot>
"""


def generate_launch_description():
    imu_topic = LaunchConfiguration('imu_topic')
    integrate = LaunchConfiguration('integrate_position')

    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu_dmp'),
        DeclareLaunchArgument('integrate_position', default_value='false'),

        Node(
            package='poseToTransform',
            executable='imuToTransform_node',
            parameters=[{
                'imu_topic': "imu_dmp",
                'child_frame': "imu_dmp_link",
                'integrate': integrate,
            }]
        ),

        Node(
            package='poseToTransform',
            executable='imuToTransform_node',
            parameters=[{
                'imu_topic': "imu_diy",
                'child_frame': "imu_diy_link",
                'integrate': integrate,
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': BLUE_URDF,
                'frame_prefix': "dmp"
            }],
            remappings=[(
                'robot_description', 'robot_description_dmp'
        )]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': RED_URDF,
                'frame_prefix': "diy"
            }],
            remappings=[(
                'robot_description', 'robot_description_diy'
            )]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-f', 'odom']),
    ])
