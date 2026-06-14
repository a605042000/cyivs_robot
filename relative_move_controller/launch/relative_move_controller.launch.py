from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='relative_move_controller',
            executable='relative_move_controller',
            name='relative_move_controller',
            output='screen',
        )
    ])
