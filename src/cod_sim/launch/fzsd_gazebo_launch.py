# fzsd_gazebo_launch.py — 起 fzsd2025 的 Gazebo 世界 + 机器人（cod_sim 版 spawn）
#
# 为什么不用 fzsd 的 bringup_sim.launch.py：
#   fzsd 的 spawn_robots 用 create 传位姿时 `-x -5.0` 分开传，gflags 把负数
#   `-5.0` 当成未知 flag，位姿退回默认 (0,0)。本 launch 复制其逻辑，create
#   改用 `-x=-5.0` 等号格式。fzsd 工作区一个字节都不改（其他项目还要用）。
#
# 世界 + 机器人 + 话题桥 + 底盘控制，与 bringup_sim 等效（不含 referee 裁判系统）。

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from xmacro.xmacro4sdf import XMLMacro4sdf

from sdformat_tools.urdf_generator import UrdfGenerator


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_fzsd2025_robot_description = get_package_share_directory("fzsd2025_robot_description")

    robot_xmacro_path = os.path.join(
        pkg_fzsd2025_robot_description, "resource", "xmacro", "fzsd2025_sentry_robot.sdf.xmacro")
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # 读出生位姿
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = LaunchDescription()

    # 1. Gazebo 世界（fzsd 的 gazebo.launch.py，含 /clock 桥）
    #    必须传 world_sdf_path/ign_config_path，否则 gazebo.launch.py 默认加载
    #    rmul_2024 世界（和 gz_world.yaml 的 rmul_2025 位姿不匹配）
    world_sdf_path = os.path.join(pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf")
    ign_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "ign_config_path": ign_config_path,
        }.items(),
    ))

    # 2. 每个机器人：spawn + 底盘控制 + 状态发布 + 话题桥
    for robot in robots:
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string", robot_xml,
                "-name", robot["name"],
                "-allow_renaming", "true",
                # gflags：负数位姿必须用 `-x=值` 等号格式
                "-x=" + robot["x_pose"],
                "-y=" + robot["y_pose"],
                "-z=" + robot["z_pose"],
                "-Y=" + robot["yaw"],
            ],
        )

        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        # red 的 RSP 完全发全局 TF（模型显示：base_footprint 及内部帧在全局树）；
        # blue 保持 namespace（避免帧冲突）。nav2 定位用 gt_odom 的 odom->base_footprint
        # （配合 gt_odom child=base_footprint，全局树为 map->odom->base_footprint->... 单一链）
        rsp_remap = [] if robot["name"] == "red_standard_robot1" else remappings
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            remappings=rsp_remap,
            parameters=[{"use_sim_time": True, "robot_description": robot_urdf_xml}],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        set_performer_service = ExecuteProcess(
            cmd=[
                "ign", "service", "-s", "/world/default/level/set_performer",
                "--reqtype", "ignition.msgs.StringMsg",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "2000",
                "--req", f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        ld.add_action(spawn_robot)
        ld.add_action(robot_base)
        ld.add_action(robot_state_publisher)
        ld.add_action(robot_ign_bridge)
        ld.add_action(set_performer_service)

    return ld
