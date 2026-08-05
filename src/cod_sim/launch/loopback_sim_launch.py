# loopback_sim_launch.py — 无硬件 loopback 仿真
#
# 用途：不接雷达/底盘/深度相机，跑通整个 Nav2 栈（MPPI 控制器 + 地图定位）。
# 原理：
#   - fake_odom 订阅底盘速度(aft_cmd_vel)积分出 odom，替代 small_point_lio
#   - 定位用固定静态 TF map->odom（本工程无 AMCL，真机也是固定 TF）
#   - 跳过 realsense / cod_serial / small_point_lio / cpp_lidar_filter
# 启动：ros2 launch cod_sim loopback_sim_launch.py
# 测试：在 RViz 用 "2D Goal Pose" 发目标点，观察 MPPI 轨迹
# 注意：静态 map->odom 决定了机器人在地图里的初始位姿（原点 + yaw -0.5），
#       若初始位姿在地图障碍物内，MPPI 会因 lethal 代价无法规划，需调整下面参数。

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bring_up_dir = get_package_share_directory('cod_bringup')

    rviz_config_file = os.path.join(bring_up_dir, 'rviz', 'cod_nav.rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file', default_value=os.path.join(bring_up_dir, 'params', 'singlenav2_params.yaml'))
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value=os.path.join(bring_up_dir, 'maps', 'rmul2026.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    map_yaml = LaunchConfiguration('map_yaml')

    load_nodes = GroupAction(
        actions=[
            # 1. 固定定位 TF（与 singlenav_launch 一致，可调）
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=[
                    "--x", "0.0", "--y", "0.0", "--z", "0.05",
                    "--roll", "0.0", "--pitch", "0.0", "--yaw", "-0.5",
                    "--frame-id", "map", "--child-frame-id", "odom",
                ],
            ),
            # 2. 假里程计：aft_cmd_vel -> odom + TF(odom->base_link)
            Node(
                package="cod_sim",
                executable="fake_odom",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # 3. 伪坐标系：base_link -> base_link_fake（Nav2 的 robot_base_frame）
            Node(
                package="fake_vel_transform",
                executable="fake_vel_transform_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # 4. 导航栈（controller_server 内加载 MPPI + goal_approach_controller）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bring_up_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': "false",
                    'autostart': "true",
                    'params_file': nav2_params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'container_name': 'nav2_container'}.items()
            ),
            # 5. 定位：map_server 加载地图（无 AMCL，定位 = 固定 TF）
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bring_up_dir, 'launch', 'localization_launch.py')),
                launch_arguments={
                    'use_sim_time': "false",
                    'autostart': "true",
                    'params_file': nav2_params_file,
                    'map': map_yaml,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'container_name': 'nav2_container'}.items()
            ),
            # 6. RViz
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen',
            ),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_nav2_params_file,
        declare_map_yaml,
        load_nodes,
    ])
