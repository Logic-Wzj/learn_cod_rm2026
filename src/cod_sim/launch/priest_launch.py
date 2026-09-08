# 启动 PRIEST 局部规划器：Gazebo + 地图 + PRIEST，无初始目标，用 rviz 2D Goal 设目标
# 用法：ros2 launch cod_sim priest_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cod_sim_dir = get_package_share_directory('cod_sim')

    declare_v_max = DeclareLaunchArgument('v_max', default_value='2.5')

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
            'child_frame_id': 'base_footprint',
        }],
    )

    urdf_global_cmd = Node(
        package='cod_sim',
        executable='urdf_global',
        output='screen',
        parameters=[{'use_sim_time': True, 'color': 'red'}],
    )

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

    # map_server 先激活，costmap 配置需要地图
    lifecycle_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 15.0,
        }],
    )

    # planner_server 配置较慢，等待就绪后手动配置+激活
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

    planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[os.path.join(cod_sim_dir, 'config', 'priest_planner_params.yaml')],
    )

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

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(cod_sim_dir, 'rviz', 'priest_nav.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    priest_cmd = Node(
        package='cod_sim',
        executable='priest_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': 'odometry',
            'cmd_vel_topic': '/priest_cmd',
            'v_max': v_max,
            'control_frequency': 15.0,
            'weight_track': 5.0,
            'obs_range': 15.0,
        }],
    )

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
