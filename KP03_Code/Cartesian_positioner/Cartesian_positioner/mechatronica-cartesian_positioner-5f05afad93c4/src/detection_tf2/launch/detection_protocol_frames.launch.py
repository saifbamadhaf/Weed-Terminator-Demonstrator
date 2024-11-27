from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection_tf2',
            executable='static_tool_broadcaster',
            name='static_tool_broadcaster',
        ),

        Node(
            package='detection_tf2',
            executable='static_camera_broadcaster',
            name='static_camera_broadcaster',
        )
        # Node(
        #     package='learning_tf2_py',
        #     executable='dynamic_frame_tf2_broadcaster',
        #     name='weed_broadcaster',
        #)
    ])

    