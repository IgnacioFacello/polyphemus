import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_package')  # <-- your package name
    layout_file = os.path.join(pkg_share, 'config', 'plotjuggler_layout.xml')

    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=['--layout', layout_file],
    )

    return LaunchDescription([plotjuggler])
