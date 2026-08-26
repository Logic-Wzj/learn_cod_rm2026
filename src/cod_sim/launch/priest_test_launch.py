# priest_test_launch.py — PRIEST 控制器仿真闭环测试
#
# 只起：Gazebo 世界 + 机器人 + 底盘控制（fzsd 身体）
#       + gt_odom（里程计） + priest_bridge（PRIEST 控制器）
# 不起 MPPI 导航栈（避免抢 cmd_vel）。
#
# 测试：PRIEST 应驱动机器人从出生点走向 target。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cod_sim_dir = get_package_share_directory('cod_sim')
    cod_bringup_dir = get_package_share_directory('cod_bringup')

    declare_target_x = DeclareLaunchArgument('target_x', default_value='-2.0')
    declare_target_y = DeclareLaunchArgument('target_y', default_value='0.0')
    declare_v_max = DeclareLaunchArgument('v_max', default_value='2.5')  # 对齐 MPPI vx_max/vy_max=2.5

    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    v_max = LaunchConfiguration('v_max')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cod_sim_dir, 'launch', 'fzsd_gazebo_launch.py')),
    )

    gt_odom_cmd = Node(
        package='cod_sim',
        executable='gt_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_odom_topic': '/red_standard_robot1/chassis_odometry_gt',
            'output_odom_topic': 'odometry',
            'frame_id': 'odom',
            'child_frame_id': 'base_footprint',  # RSP 的根帧，避免 gimbal_yaw 双父冲突
        }],
    )

    # 全局 robot_description 参数（rviz RobotModel 默认读它）
    urdf_global_cmd = Node(
        package='cod_sim',
        executable='urdf_global',
        output='screen',
        parameters=[{'use_sim_time': True, 'color': 'red'}],
    )

    # map->odom 静态 TF（0 偏移），让 cod_nav.rviz 的 fixed frame(map) 可用
    map_to_odom_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=[
            "--x", "5.0", "--y", "-3.0", "--z", "0.0",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
    )

    # map_server：加载先验地图供 rviz 显示 + 全局规划
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': '/home/wzj/fzsd2025/install/pb2025_nav_bringup/share/pb2025_nav_bringup/map/simulation/testslam.yaml',
        }],
    )
    # 生命周期顺序修复：map_server 先激活出图，global_costmap 的静态层配置需要地图。
    # （nav2 lifecycle manager 默认"先配置全部再激活全部"，costmap 配置时 map_server 未激活
    #   就没图可订阅 → costmap 配置卡死 → planner 永不激活。）
    # 方案：map_server 用独立 manager 立即激活；costmap+planner 用另一 manager 延迟 8s 启动，
    # 确保 map_server 已激活、/map 已发布后再配置 costmap。
    lifecycle_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 15.0,  # 启动重载时 map_server 心跳可能延迟 >4s，被误判 DOWN 停用；加大超时
        }],
    )
    # planner_server 配置（含内置 costmap 加载地图）较慢，nav2 lifecycle manager 的
    # change_state 服务响应 5s 超时会失败。改用 ExecuteProcess：等待 planner 就绪后配置+激活。
    activate_planner_cmd = ExecuteProcess(
        cmd=['bash', '-c',
             "for i in $(seq 1 30); do "
             "  if ros2 lifecycle get /planner_server >/dev/null 2>&1; then break; fi; "
             "  echo \"wait planner ($i/30)...\"; sleep 2; "
             "done; "
             "st=$(ros2 lifecycle get /planner_server 2>/dev/null); "
             "if echo \"$st\" | grep -q 'unconfigured'; then "
             "  echo 'configuring planner...'; ros2 lifecycle set /planner_server configure; "
             "fi; "
             "echo 'activating planner...'; ros2 lifecycle set /planner_server activate"],
        output='screen',
    )

    # 全局规划：planner_server（内置 global_costmap，标准 nav2 结构），PRIEST 沿全局路径走
    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(cod_sim_dir, 'config', 'priest_planner_params.yaml')],
    )

    # 场地点云地图（testslam.pcd → /map_cloud），周期发布（pcl_ros 的一次性发布 rviz 收不到）
    pcd_cmd = Node(
        package='cod_sim',
        executable='pcd_publisher',
        name='pcd_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pcd_file': '/home/wzj/fzsd2025/install/pb2025_nav_bringup/share/pb2025_nav_bringup/pcd/simulation/testslam.pcd',
            'frame_id': 'map',
            'topic': 'map_cloud',
            'stride': 20,
            'publish_period': 2.0,
        }],
    )

    # RViz（cod_bringup 的 cod_nav.rviz，含 RobotModel 显示项）
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(cod_sim_dir, 'rviz', 'priest_nav.rviz')],
        # 开 use_sim_time：数据（点云/TF）用仿真时钟，rviz 默认 wall time 会时间戳不匹配
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    priest_cmd = Node(
        package='cod_sim',
        executable='priest_bridge',
        output='screen',
        # JAX 显存按需分配已在 priest_bridge.py 内通过 os.environ 处理
        parameters=[{
            'use_sim_time': True,
            'odom_topic': 'odometry',
            'cmd_vel_topic': '/priest_cmd',  # -> fake_vel_transform 输入（复刻 MPPI 链路）
            'target_x': target_x,
            'target_y': target_y,
            'v_max': v_max,
            'control_frequency': 5.0,
            'weight_track': 5.0,  # 大幅增强 waypoint 跟踪（横向跟随）
            'obs_range': 15.0,  # PRIEST 用 livox 点云避障（参考 MPPI：避障在局部控制器做）
        }],
    )

    # fake_vel_transform：nav2 链路里它负责把速度转到 base 系，MPPI 经它成功导航
    fake_vel_cmd = Node(
        package='fake_vel_transform',
        executable='fake_vel_transform_node',
        name='fake_vel_transform',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': 'odometry',
            'robot_base_frame': 'gimbal_yaw',
            'fake_robot_base_frame': 'gimbal_yaw_fake',
            'input_cmd_vel_topic': '/priest_cmd',
            'output_cmd_vel_topic': '/red_standard_robot1/cmd_vel',
        }],
    )

    return LaunchDescription([
        declare_target_x,
        declare_target_y,
        declare_v_max,
        gazebo_cmd,
        gt_odom_cmd,
        urdf_global_cmd,
        map_to_odom_cmd,
        map_server_cmd,
        lifecycle_map_cmd,
        planner_server_cmd,
        activate_planner_cmd,
        pcd_cmd,
        priest_cmd,
        fake_vel_cmd,
        rviz_cmd,
    ])
