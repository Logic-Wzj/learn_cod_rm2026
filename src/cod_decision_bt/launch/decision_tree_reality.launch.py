# decision_tree_reality.launch.py — 实车"上层决策"一键 launch
#
# 前提：实车导航栈已在跑（提供 bt_navigator，action 在 reality yaml 的
#       navigate_server_name 指向的位置，默认 /red_standard_robot1/navigate_to_pose）。
#       导航栈（含定位/地图/底盘 cmd_vel）由车队自己的 bringup 起，这里不重复起。
#
# 本 launch 负责：
#   1. decision_tree_node   —— 高层决策树（reality XML + reality params，真实时钟）
#   2. 假裁判 referee_simulator_node —— 订阅 /referee/cmd，发 /referee/game_status
#      + /referee/robot_status（cod 类型、全局话题，与决策树条件订阅一致）
#   3. patrol_collector_node —— 收集 RViz Publish Point 的巡航点，喂给 Patrol 分支
#   4. （可选）rviz —— 可视化 / 用 Publish Point 点巡航点
#
# 所有可调参数（补给点、巡航点、话题名…）在 config/reality/decision_tree_reality.yaml，
# 改那份文件即可，不用动本 launch。
#
# 用法（先确保实车 nav 起来）：
#   ros2 launch cod_decision_bt decision_tree_reality.launch.py
#   有屏幕想一起开 RViz：
#   ros2 launch cod_decision_bt decision_tree_reality.launch.py start_rviz:=true
#   无显示环境（SSH 上板子）把键盘终端关掉，自己另开 ssh 终端跑键盘：
#   ros2 launch cod_decision_bt decision_tree_reality.launch.py start_keyboard:=false
#   （另开终端）ros2 run cod_referee_simulator referee_keyboard
#
# 按键：3=比赛开始 / 5=扣血 / 4=结束 / 8=重置 / q=退出（referee_keyboard 窗口内按）
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    decision_dir = get_package_share_directory('cod_decision_bt')
    referee_dir = get_package_share_directory('cod_referee_simulator')

    reality_params = os.path.join(
        decision_dir, 'config', 'reality', 'decision_tree_reality.yaml')
    # 实车决策树 XML（服务器名/坐标都引用 yaml 里的 ${key}）
    bt_xml = os.path.join(decision_dir, 'behavior_trees', 'decision_tree_reality.xml')

    declare_fake_referee = DeclareLaunchArgument(
        'use_fake_referee', default_value='true',
        description='启动假裁判 referee_simulator_node（发 cod 类型 /referee/*）')
    declare_keyboard = DeclareLaunchArgument(
        'start_keyboard', default_value='true',
        description='在 gnome-terminal 里开 referee_keyboard（无显示环境设 false，另开终端跑）')
    declare_collector = DeclareLaunchArgument(
        'start_collector', default_value='true',
        description='启动巡航点收集器 patrol_collector_node（RViz Publish Point 点巡航点）')
    declare_rviz = DeclareLaunchArgument(
        'start_rviz', default_value='false',
        description='启动 RViz（一般由导航那边开，避免重复）')

    use_fake_referee = LaunchConfiguration('use_fake_referee')
    start_keyboard = LaunchConfiguration('start_keyboard')
    start_collector = LaunchConfiguration('start_collector')
    start_rviz = LaunchConfiguration('start_rviz')

    # 1. 决策树节点（核心）
    decision_tree_node = Node(
        package='cod_decision_bt',
        executable='decision_tree_node',
        name='decision_tree_node',
        output='screen',
        parameters=[
            reality_params,
            {'bt_xml': bt_xml},
            {'use_sim_time': False},
        ],
    )

    # 2. 假裁判：收 /referee/cmd(键盘) -> 发 /referee/game_status + /referee/robot_status
    #    话题全局、cod 类型，和决策树条件订阅完全一致；真实时钟。
    referee_simulator = Node(
        package='cod_referee_simulator',
        executable='referee_simulator_node',
        name='referee_simulator_node',
        output='screen',
        condition=IfCondition(use_fake_referee),
        parameters=[{'use_sim_time': False}],
    )

    # 2b. 键盘（交互式，独立终端）；在 launch 外跑也一样
    referee_keyboard = Node(
        package='cod_referee_simulator',
        executable='referee_keyboard',
        name='referee_keyboard',
        output='screen',
        condition=IfCondition(start_keyboard),
        prefix='gnome-terminal -- ',
    )

    # 3. 巡航点收集器：RViz Publish Point -> /patrol_waypoints -> 决策树 Patrol
    collector_node = Node(
        package='cod_decision_bt',
        executable='patrol_collector_node',
        name='patrol_collector_node',
        output='screen',
        condition=IfCondition(start_collector),
    )

    # 4. 可选 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(start_rviz),
        arguments=['-d', os.path.join(
            get_package_share_directory('cod_bringup'), 'rviz', 'cod_nav.rviz')],
        output='screen',
    )

    return LaunchDescription([
        declare_fake_referee,
        declare_keyboard,
        declare_collector,
        declare_rviz,
        decision_tree_node,
        referee_simulator,
        referee_keyboard,
        collector_node,
        rviz_node,
    ])
