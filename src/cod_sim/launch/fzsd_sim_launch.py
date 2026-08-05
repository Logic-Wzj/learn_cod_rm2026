# fzsd_sim_launch.py — 用 fzsd2025 的 Gazebo 仿真世界跑 cod 的 MPPI 导航栈
#
# 架构（最短链路，全部为已验证组件）：
#   - 身体：fzsd2025 的 rmu_gazebo_simulator（世界 + 哨兵机器人 + ros_gz_bridge + 底盘控制）
#   - 里程计：Gazebo ground-truth（chassis_odometry_gt -> odometry + TF odom->gimbal_yaw，零漂移）
#   - 定位：静态 TF map->odom（机器人出生位姿，与 rmul_2025 地图对齐）
#   - 障碍：rplidar_a2 的 /scan 喂 costmap ObstacleLayer
#   - 脑子：cod 的 navigation_launch（goal_approach_controller + MPPI，无 namespace，
#     参数文件直接匹配，不经过 fzsd 的容器/namespace 机制）
#   - 速度链：nav2 -> /cmd_vel -> fake_vel_transform(转发+旋转变换) -> /red_standard_robot1/cmd_vel
#     -> chassis_controller -> gz 麦轮底盘
#
# 前置：source /home/wzj/fzsd2025/install/setup.zsh 和 cod 工作空间的 setup.zsh
# 启动：ros2 launch cod_sim fzsd_sim_launch.py
#
# 注意：
#   - init_x/init_y/init_yaw 必须与 gz_world.yaml 中机器人出生位姿一致（默认 rmul_2025 red_standard_robot1）
#   - 换世界 = 改 fzsd 的 rmu_gazebo_simulator/config/gz_world.yaml + 传新地图和出生位姿
#   - 首次启动 Gazebo 加载较慢（1~2 分钟），需要 GPU

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cod_bringup_dir = get_package_share_directory('cod_bringup')
    cod_sim_dir = get_package_share_directory('cod_sim')
    fzsd_simulator_dir = get_package_share_directory('rmu_gazebo_simulator')
    fzsd_bringup_dir = get_package_share_directory('pb2025_nav_bringup')

    # 参数
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value='rmul_2025',
        description="仿真世界（需与 fzsd gz_world.yaml 一致）")
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='red_standard_robot1',
        description="仿真机器人命名空间（话题前缀）")
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(cod_sim_dir, 'config', 'gt_sim_params.yaml'),
        description="导航参数（cod 版改造：gimbal_yaw 帧 + scan 障碍 + 仿真时钟）")
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(fzsd_bringup_dir, 'map', 'simulation', 'testslam.yaml'))
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True')
    # map->odom 静态 TF：gt_odom 的 odom 系 = gazebo world 系，而 world 坐标即地图坐标
    # （出生位姿已体现在 gazebo 中），故默认全 0
    declare_init_x_cmd = DeclareLaunchArgument('init_x', default_value='0.0')
    declare_init_y_cmd = DeclareLaunchArgument('init_y', default_value='0.0')
    declare_init_yaw_cmd = DeclareLaunchArgument('init_yaw', default_value='0.0')

    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map_yaml')
    use_rviz = LaunchConfiguration('use_rviz')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_yaw = LaunchConfiguration('init_yaw')

    # 1. Gazebo 世界 + 哨兵机器人 + 话题桥 + 底盘控制（fzsd 的"身体"）
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fzsd_simulator_dir, 'launch', 'bringup_sim.launch.py')),
    )

    # 2. Gazebo ground-truth 里程计 -> odometry + TF(odom->gimbal_yaw)
    gt_odom_cmd = Node(
        package='cod_sim',
        executable='gt_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_odom_topic': '/red_standard_robot1/chassis_odometry_gt',
            'output_odom_topic': 'odometry',
            'frame_id': 'odom',
            'child_frame_id': 'gimbal_yaw',
        }],
    )

    # 3. 静态定位 TF map->odom（出生位姿，与地图对齐）
    static_tf_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "--x", init_x, "--y", init_y, "--z", "0.28",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", init_yaw,
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    # 4. 地图（rmul_2025 场地图）供全局规划
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}],
    )
    lifecycle_localization_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # 5. cod 的导航栈（controller = goal_approach_controller + MPPI，无 namespace）
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cod_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'params_file': params_file,
            'use_composition': 'False',
            'use_respawn': 'False',
            'container_name': 'nav2_container',
        }.items(),
    )

    # 6. cmd_vel 转发：nav2 输出 /cmd_vel -> /<namespace>/cmd_vel（chassis_controller 订阅）
    #    顺便做 yaw 旋转变换（gimbal 静止时等效恒等）
    vel_forward_cmd = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        name='fake_vel_transform',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': 'odometry',
            'robot_base_frame': 'gimbal_yaw',
            'fake_robot_base_frame': 'gimbal_yaw_fake',
            'input_cmd_vel_topic': 'cmd_vel',
            'output_cmd_vel_topic': ['/', namespace, '/cmd_vel'],
        }],
    )

    # 7. RViz（cod 的配置，无 namespace，话题全匹配）
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(cod_bringup_dir, 'rviz', 'cod_nav.rviz')],
        output='screen',
    )

    return LaunchDescription([
        declare_world_cmd,
        declare_namespace_cmd,
        declare_params_file_cmd,
        declare_map_yaml_cmd,
        declare_use_rviz_cmd,
        declare_init_x_cmd,
        declare_init_y_cmd,
        declare_init_yaw_cmd,
        gazebo_cmd,
        gt_odom_cmd,
        static_tf_cmd,
        map_server_cmd,
        lifecycle_localization_cmd,
        nav_cmd,
        vel_forward_cmd,
        rviz_cmd,
    ])
