from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    fishbot_bringup_dir = get_package_share_directory('fishbot_bringup')

    fishbot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [fishbot_bringup_dir, '/launch/bringup.launch.py']
        )
    )

    relative_move_controller_node = Node(
        package='relative_move_controller',
        executable='relative_move_controller',
        output='screen',
    )

    return LaunchDescription([
        fishbot_bringup_launch,
        relative_move_controller_node,
    ])