from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='positioner_tf2_frame_publisher',
            executable='conveyor_encoder',
            name='conveyor'
        ),
    ])