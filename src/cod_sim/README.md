# cod_sim — COD 导航仿真包

本包为 cod_-rm2026_-navigation 提供两种仿真方式，让导航栈（MPPI + goal_approach_controller）无需真机即可验证。

## 1. Loopback 仿真（轻量，无 Gazebo）

用假里程计 + 固定静态 TF 跑通导航栈，不依赖雷达/底盘/相机。

```bash
ros2 launch cod_sim loopback_sim_launch.py
# 可选：换地图
ros2 launch cod_sim loopback_sim_launch.py map_yaml:=/path/to/map.yaml
```

- `fake_odom` 节点订阅 `aft_cmd_vel` 积分出 odom + TF(odom->base_link)，替代 small_point_lio
- 定位 = 静态 TF map->odom（本工程真机也是固定 TF，无 AMCL）
- 初始位姿由 launch 中 `map_to_odom` 静态 TF 决定，需落在无障碍区域

## 2. fzsd2025 Gazebo 仿真（完整，需 GPU）

复用上一届工作空间 `/home/wzj/fzsd2025` 的 Gazebo(Ignition) 仿真世界（rmul_2024/2025、rmuc_2024/2025 场地 + 哨兵机器人 + 传感器），跑 cod 的 MPPI 导航栈。

**架构（换脑）：**

```
Gazebo 世界 + 哨兵机器人 (fzsd2025 的 rmu_gazebo_simulator)
  ├─ ros_gz_bridge: /<ns>/livox/lidar, /<ns>/livox/imu, /<ns>/chassis_odometry_gt
  ├─ ign_sim_pointcloud_tool: 仿真点云 → livox 风格 (velodyne_points)
  ├─ point_lio: 点云+IMU → odometry + TF(odom->gimbal_yaw)
  ├─ small_gicp_relocalization: prior_pcd 重定位 → TF(map->odom)
  └─ 导航栈 (pb2025_nav_bringup 的 launch 结构):
       controller = cod 的 goal_approach_controller + MPPI  ← 换脑点
       velocity_smoother: cmd_vel_controller → cmd_vel_nav2_result
       fake_vel_transform: cmd_vel_nav2_result → cmd_vel
       → chassis_controller (rmua19_robot_base) → gz 麦轮底盘
```

**启动（需同时 source 两个工作空间）：**

```bash
source /opt/ros/humble/setup.zsh
source /home/wzj/fzsd2025/install/setup.zsh
source /home/wzj/cod_-rm2026_-navigation/install/setup.zsh
ros2 launch cod_sim fzsd_sim_launch.py
```

- 首次启动 Gazebo 加载 1~2 分钟（需要 GPU）
- RViz 用 "2D Goal Pose" 发目标点；也可命令行发 `NavigateToPose` action
- 换世界：修改 fzsd 的 `rmu_gazebo_simulator/config/gz_world.yaml` 中 `world:` 字段（launch 的 world 参数需一致），地图/prior_pcd 自动跟随
- 裁判系统组件 `pose_bridge` 可能崩溃（exit -11），与导航无关，可忽略

**关键文件：**

- `launch/fzsd_sim_launch.py` — 整合 launch（fzsd 身体 + cod 脑）
- `config/fzsd_sim_params.yaml` — 整合参数：fzsd 仿真全套参数（point_lio/small_gicp/fake_vel_transform 等）+ cod 的 FollowPath 段（goal_approach_controller + MPPI，取自 singlenav2_params.yaml）
- 重新生成整合参数的方法：见下方脚本逻辑（读 fzsd simulation yaml，替换 controller_server 的 FollowPath 段为 cod 版，yaml.dump 输出）

**已知注意点：**

- launch 的布尔参数必须用大写 `True`/`False`（fzsd 的 launch 用 PythonExpression eval）
- 若 launch 中途崩溃会留下孤儿进程（容器/gazebo），排查前先清理：
  `pkill -f "ign gazebo"; pkill -f "rmua19_robot_base"; pkill -f "component_container"; pkill -f "pointlio_mapping"`
- 同名 `nav2_container` 残留进程会导致组件加载失败，务必先清理再重启
