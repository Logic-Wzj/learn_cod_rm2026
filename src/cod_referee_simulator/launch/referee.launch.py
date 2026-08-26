import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cod_referee_simulator')
    params_path = os.path.join(pkg_dir, 'config', 'referee_params.yaml')

    referee_simulator_node = Node(
        package='cod_referee_simulator',
        executable='referee_simulator_node',
        name='referee_simulator_node',
        output='screen',
        parameters=[params_path]
    )

    referee_keyboard = Node(
        package='cod_referee_simulator',
        executable='referee_keyboard',
        name='referee_keyboard',
        output='screen',
        prefix='gnome-terminal -- '
    )

    return LaunchDescription([referee_simulator_node, referee_keyboard])
