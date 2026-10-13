from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    layout_arg = DeclareLaunchArgument(
        'layout',
        default_value='dmp',
        choices=['dmp', 'gyro'],  # <-- your two layout names (no .xml)
        description='PlotJuggler layout to load from the config folder',
    )

    layout_file = PathJoinSubstitution([
        FindPackageShare('camera_bringup'),
        'config',
        ['diy_', LaunchConfiguration('layout'), '__error_plot', '.xml'],
    ])

    plotjuggler = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=[
            '--layout', layout_file,
            '--start_streamer', 'ROS2 Topic Subscriber'
        ],
    )

    return LaunchDescription([layout_arg, plotjuggler])
