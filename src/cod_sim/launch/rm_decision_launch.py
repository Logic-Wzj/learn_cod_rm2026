# 一键启动：仿真 + 裁判系统 + 高层决策行为树
# 用法：ros2 launch cod_sim rm_decision_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    cod_sim_dir = get_package_share_directory('cod_sim')
    referee_dir = get_package_share_directory('cod_referee_simulator')
    decision_dir = get_package_share_directory('cod_decision_bt')

    sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cod_sim_dir, 'launch', 'fzsd_sim_launch.py')),
    )

    referee_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(referee_dir, 'launch', 'referee.launch.py')),
    )

    decision_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(decision_dir, 'launch', 'decision_tree.launch.py')),
    )

    return LaunchDescription([sim_cmd, referee_cmd, decision_cmd])
