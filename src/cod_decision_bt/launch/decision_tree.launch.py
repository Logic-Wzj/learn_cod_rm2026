import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('cod_decision_bt')
    params_path = os.path.join(pkg_dir, 'config', 'decision_tree_params.yaml')
    bt_xml = os.path.join(pkg_dir, 'behavior_trees', 'decision_tree.xml')

    decision_tree_node = Node(
        package='cod_decision_bt',
        executable='decision_tree_node',
        name='decision_tree_node',
        output='screen',
        parameters=[params_path, {'bt_xml': bt_xml}]
    )

    return LaunchDescription([decision_tree_node])
