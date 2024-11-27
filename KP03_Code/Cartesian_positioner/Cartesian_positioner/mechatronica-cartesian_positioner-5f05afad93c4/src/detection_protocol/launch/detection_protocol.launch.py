from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection_protocol',
            executable='detections',
            name='publisher_detections',
        ),

        Node(
            package='detection_protocol',
            executable='speed',
            name='publisher_speed',
        ),

        Node(
            package='detection_protocol',
            executable='filtering',
            name='filtering',
        ),

        Node(
            package='detection_protocol',
            executable='weeding_sequence',
            name='weeding_sequence',
        )
    ])
