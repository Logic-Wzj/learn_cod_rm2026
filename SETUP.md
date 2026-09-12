# 环境搭建 / 一键配置

本项目**自足**：把仿真真正需要的 fzsd 部分已 vendored 进 `src/fzsd_vendor/`，所以**不需要外部 fzsd2025 工作区**就能编、能跑仿真。

## 目标环境
- **Ubuntu 22.04 + ROS 2 Humble**（本项目按 humble 编写；其它发行版不保证）
- 有显示（跑 Gazebo/RViz）

## 一键配置
```bash
# 先把 ROS 2 Humble 装好（apt），然后：
cd <本仓库>
./setup_env.sh
```
脚本会做：装系统依赖（gz Fortress、ros_gz、nav2、pcl/eigen、xmacro…）→ `rosdep install` → `colcon build`。
只装依赖不编译：`./setup_env.sh --deps-only`

## 手动步骤（等价于脚本）
```bash
source /opt/ros/humble/setup.bash
sudo apt install -y python3-pip libeigen3-dev libpcl-dev ignition-fortress \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
    ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-ament-cmake-clang-tidy
pip3 install xmacro==1.2.1
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## 跑仿真
```bash
source install/setup.bash
ros2 launch cod_sim rm_decision_launch.py     # 仿真 + 裁判 + 决策树
```
（另开终端）`ros2 run cod_referee_simulator referee_keyboard` 按键 3 开赛 / 5 扣血 / 4 结束。

## 关于 `src/fzsd_vendor/`（vendored 的第三方代码）
| 目录 | 内容 | 说明 |
|---|---|---|
| `fzsd2025_gazebo_simulator/` | rmu_gazebo_simulator + rmoss_gz_* / rmoss_core / sdformat_tools / rmoss_interfaces | 仿真世界 + 哨兵底盘 + 桥 |
| `fzsd2025_robot_description/` | 机器人 SDF/xmacro 模型 + mesh | spawn 用 |
| `pb2025_nav_bringup/` | **只保留 `map/`**（testslam 仿真地图） | 已去掉 2G 的 `pcd/` |
| `livox_ros_driver2/` | livox 雷达驱动 + 自带 **Livox-SDK2**（含 amd64/arm64 预编译库） | 无需外装 SDK；`small_point_lio` 会自动启用 Livox 支持 |
| `nav2_command_handler/` | 命令处理节点（rmu 依赖） | 小，随包带上 |
| `vision_interfaces/` | 视觉消息接口（rmu 依赖） | 小，随包带上 |

> 注：vendored 的 `pb2025_nav_bringup` 只用于它的 `map/`，其 package.xml 里**实车专用**依赖（point_lio / terrain_analysis / small_gicp_relocalization / pb_teleop_twist_joy / pb2025_sentry_nav）已裁剪，避免 rosdep 报未解析；它的实车 launch 本仓库不跑。

- 这些是从 **fzsd2025** 工程拷来的（非本团队原创），只为让仿真自足。
- **刷新方式**（上游更新时）：
  ```bash
  rsync -a --exclude '.git' --exclude build --exclude install --exclude log \
      ~/fzsd2025/src/fzsd2025_gazebo_simulator/ src/fzsd_vendor/fzsd2025_gazebo_simulator/
  rsync -a --exclude '.git' --exclude build --exclude install --exclude log \
      ~/fzsd2025/src/fzsd2025_robot_description/ src/fzsd_vendor/fzsd2025_robot_description/
  rsync -a --exclude 'pcd' --exclude '.git' \
      ~/fzsd2025/src/fzsd2025_sentry_nav/pb2025_nav_bringup/ src/fzsd_vendor/pb2025_nav_bringup/
  rsync -a --exclude '.git' --exclude build --exclude install --exclude log \
      ~/fzsd2025/src/fzsd2025_sentry_nav/livox_ros_driver2/ src/fzsd_vendor/livox_ros_driver2/
  ```

## 说明
- 仿真里的导航/决策/MPPI 全是本仓库的（cod_bringup / cod_decision_bt / …），vendored 的只是"身体"（世界/模型/地图）。
- 雷达驱动 `livox_ros_driver2` 已 vendored 进 `src/fzsd_vendor/`（自带 SDK，编译即可用），`small_point_lio` 会自动启用 Livox (CustomMsg) 支持。**仿真**本身仍用 Gazebo 的 `gpu_lidar` 模拟雷达，不依赖这个驱动。
- 实车（fzsd 哨兵）另需 fzsd 的实车 bringup（雷达/LIO/定位），那部分是车上的事，不在本仓库。
