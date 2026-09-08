#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# priest_bridge.py — PRIEST 控制器 ROS2 桥节点
#
# 复用 PRIEST 的纯 JAX 算法核心（cod_sim/priest_algo/mpc_expert.py），
# 替代 MPPI 作为局部控制器。
#
# 订阅:  odometry (odom)         -> 机器人位置/速度（world 系）
#        /goal_pose              -> rviz 2D Goal（设目标，无目标时停车等待）
#        livox/lidar (PointCloud2) -> 障碍（避障输入）
# 发布:  cmd_vel                 -> 底盘速度（body 系，PRIEST 输出 world 系需旋转）
# 参数:  v_max                   -> 速度上限

import math
import os
import struct

# JAX 显存按需分配，避免和 gazebo 渲染竞争 GPU 显存（cuSolver handle 创建失败）
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2, JointState
import numpy as np
import jax.numpy as jnp
from jax import random
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose

from cod_sim.priest_algo import mpc_expert


class PriestBridge(Node):
    def __init__(self):
        super().__init__('priest_bridge')

        self.declare_parameter('odom_topic', 'odometry')
        self.declare_parameter('cmd_vel_topic', '/priest_cmd')  # 输出到 fake_vel_transform 输入
        self.declare_parameter('target_x', -2.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('v_max', 2.5)
        self.declare_parameter('v_min', 0.02)
        self.declare_parameter('a_max', 3.0)  # 加速更快，短时间域内平均速度更高
        self.declare_parameter('control_frequency', 15.0)  # 单次计算 ~53ms，支持到 ~18Hz
        self.declare_parameter('cloud_topic', '/red_standard_robot1/livox/lidar')
        self.declare_parameter('obs_range', 15.0)
        self.declare_parameter('weight_track', 0.001)  # waypoint 跟踪权重（太小则横向不跟随）
        self.declare_parameter('weight_smoothness', 1.0)
        self.declare_parameter('v_des', 2.5)  # PRIEST 目标速度（对齐 MPPI vx_max/vy_max=2.5）
        self.declare_parameter('speed_floor', 0.5)  # 最低输出速度：PRIEST 输出偏小时放大到该值（保方向，clamp v_max）
        self.declare_parameter('stop_radius', 0.4)  # 到目标该距离内停车

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.has_target = False  # 无初始目标，等 rviz 2D Goal

        # 机器人状态（world/odom 系）
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0

        # 障碍（odom 系，PRIEST 障碍输入）
        self.x_obs = jnp.ones(420) * 100
        self.y_obs = jnp.ones(420) * 100

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 全局路径（odom 系）
        self.path_odom = None
        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # 算法对象
        self.prob = None
        self.key = random.PRNGKey(0)
        self.initialized = False

        # gimbal yaw（chassis_controller 用它旋转 cmd_vel 到底盘系，需先转到 gimbal 系）
        self.gimbal_yaw = 0.0
        self.create_subscription(JointState, '/red_standard_robot1/joint_states', self.js_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # 发布 PRIEST 最优轨迹（rviz 可视化，诊断算法/链路问题）
        self.path_pub = self.create_publisher(Path, '/priest_path', 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self.cloud_cb, 10)
        self.timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(
            f'PriestBridge: odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'频率={self.control_frequency}Hz')

    # ---------- 回调 ----------

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def goal_cb(self, msg: PoseStamped):
        """rviz 2D Goal Pose（map 系）-> 转 odom 系目标 + 请求全局路径"""
        # PRIEST 的 waypoint 用 odom 系（self.x/self.y 是 odom 系），
        # rviz goal 是 map 系，需用 TF map->odom 转换，否则方向偏移
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'map', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            self.target_x = msg.pose.position.x + trans.transform.translation.x
            self.target_y = msg.pose.position.y + trans.transform.translation.y
        except Exception:
            self.target_x = msg.pose.position.x
            self.target_y = msg.pose.position.y
        # 清掉旧路径：新目标路径获取失败时若沿用旧 path_odom，PRIEST 会沿旧路径走错方向
        self.path_odom = None
        self.has_target = True
        self.get_logger().info(f'PRIEST 新目标(odom): ({self.target_x:.2f}, {self.target_y:.2f})')
        self._request_global_path(msg)

    def _request_global_path(self, goal_msg: PoseStamped):
        """调 planner_server 的 compute_path_to_pose 得到全局路径"""
        if not self.path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('全局路径：action server 不可用')
            return
        req = ComputePathToPose.Goal()
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'全局路径：TF map->odom 查询失败: {e}')
            return
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = self.x + trans.transform.translation.x
        start.pose.position.y = self.y + trans.transform.translation.y
        start.pose.orientation.w = 1.0
        req.start = start
        req.goal = goal_msg
        self.get_logger().info(
            f'请求全局路径: start=({start.pose.position.x:.2f},{start.pose.position.y:.2f}) '
            f'goal=({goal_msg.pose.position.x:.2f},{goal_msg.pose.position.y:.2f})')
        future = self.path_client.send_goal_async(req)
        future.add_done_callback(self._path_goal_done)

    def _path_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('全局路径：goal 被拒')
            return
        self.get_logger().info('全局路径：goal 已接受')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._path_result_done)

    def _path_result_done(self, future):
        path_msg = future.result().result.path
        if not path_msg.poses:
            self.get_logger().warn('全局路径：结果为 0 点')
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', 'map', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return
        pts = []
        for p in path_msg.poses:
            pts.append((p.pose.position.x + trans.transform.translation.x,
                        p.pose.position.y + trans.transform.translation.y))
        self.path_odom = pts
        self.get_logger().info(f'已获取全局路径（{len(pts)} 点）')

    def js_cb(self, msg: JointState):
        """读取 gimbal yaw（chassis_controller 旋转 cmd_vel 用）"""
        for i, name in enumerate(msg.name):
            if name == 'gimbal_yaw_joint':
                self.gimbal_yaw = msg.position[i]

    def _parse_cloud(self, msg: PointCloud2):
        """解析 PointCloud2 的 xyz（点云 frame 系）"""
        x_off = y_off = z_off = None
        for f in msg.fields:
            if f.name == 'x':
                x_off = f.offset
            elif f.name == 'y':
                y_off = f.offset
            elif f.name == 'z':
                z_off = f.offset
        if x_off is None or y_off is None or z_off is None:
            return []
        pts = []
        pt_step = msg.point_step
        data = msg.data
        for i in range(msg.width):
            base = i * pt_step
            pts.append((struct.unpack_from('<f', data, base + x_off)[0],
                        struct.unpack_from('<f', data, base + y_off)[0],
                        struct.unpack_from('<f', data, base + z_off)[0]))
        return pts

    def cloud_cb(self, msg: PointCloud2):
        """点云 -> odom 系障碍点 -> PRIEST 障碍输入"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom', msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            return
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        obs_range = self.get_parameter('obs_range').value
        ox, oy = [], []
        for (x, y, z) in self._parse_cloud(msg):
            wx = x * cos_y - y * sin_y + tx
            wy = x * sin_y + y * cos_y + ty
            if not (math.isfinite(wx) and math.isfinite(wy)):
                continue
            dist = math.hypot(wx, wy)
            # 下限 0.5→0.15：原 0.5m 内障碍被丢弃，车贴近墙(<0.5m)时看不到墙→撞上去卡住
            if 0.15 < dist < obs_range:
                ox.append(wx)
                oy.append(wy)
        stride = max(1, len(ox) // 400) if ox else 1
        ox = ox[::stride][:420]
        oy = oy[::stride][:420]
        ox += [100.0] * (420 - len(ox))
        oy += [100.0] * (420 - len(oy))
        self.x_obs = jnp.asarray(ox)
        self.y_obs = jnp.asarray(oy)

    # ---------- 算法 ----------

    def _init_algorithm(self):
        """初始化 PRIEST 算法并预热（JIT 首次编译 ~15s）"""
        v_max = self.get_parameter('v_max').value
        v_min = self.get_parameter('v_min').value
        a_max = self.get_parameter('a_max').value
        weight_track = self.get_parameter('weight_track').value
        weight_smoothness = self.get_parameter('weight_smoothness').value
        v_des = self.get_parameter('v_des').value
        # 参数调整（2026-08-26 分析 + 论文对照）：
        #   t_fin 10→1.5：局部时间域，消除"填满时间域冲过头"的大绕弯（适配短程目标）
        #   a_obs/b_obs 0.58→2.5：避障椭圆增大（弧线风格下加强避障，避免撞障碍）
        #   num_obs 60→420：用全部 lidar 障碍点
        #   maxiter_cem 13→30：CEM 更多迭代，近障碍更稳定
        #   代价函数：cost_obs 权重 1.0→5.0（mpc_expert.py 内）
        # a_obs/b_obs：2.5 太大（投影推 2.5m 侧绕→CEM 选穿障碍）；0.5~0.7 能避但贴障碍近；
        # 1.0 让障碍禁区更大 → 绕行弧度更大，避障更有力（只影响有障碍时，无障碍轨迹不变）
        self.prob = mpc_expert.batch_crowd_nav(
            1.0, 1.0, v_max, v_min, a_max, 420, 1.5, 100, 110,
            1, 30, weight_smoothness, weight_track, 1000, v_des)
        self.get_logger().info('PriestBridge: 预热（JIT 编译，约 15s）...')
        self._compute_step(0.0, 0.0, 0.0, 0.0)
        self.initialized = True
        self.get_logger().info('PriestBridge: 预热完成，开始控制')

    def _compute_step(self, x, y, vx, vy):
        """一次控制计算，返回 (vx_world, vy_world)"""
        target_x = self.target_x
        target_y = self.target_y

        # waypoint：优先用全局路径；否则当前->目标直线
        if self.path_odom is not None and len(self.path_odom) > 2:
            x_wp, y_wp = self._path_waypoints(x, y)
        else:
            x_wp = jnp.linspace(x, target_x, 1000)
            y_wp = jnp.linspace(y, target_y, 1000)
        # 轨迹初始速度 = min(v_des, 到目标距离/t_fin) 沿 waypoint 方向。
        # 起点速度0 → 直线需瞬间加速→高平滑代价→代价偏爱弧线；v_des 起步 → 轨迹变直。
        # 但近目标时 v_des 会过冲绕目标振荡，故用 min 限速（目标近自动减速）。
        v_des = self.get_parameter('v_des').value
        dir_x = float(x_wp[1] - x_wp[0])
        dir_y = float(y_wp[1] - y_wp[0])
        d0 = math.hypot(dir_x, dir_y)
        dist_to_end = math.hypot(float(x_wp[-1]) - x, float(y_wp[-1]) - y)
        t_fin = 1.5  # 与 batch_crowd_nav 的 t_fin 一致
        # 初速度按到目标距离限速（min），轨迹内建自然减速；近终点轨迹长度已缩短→保持直
        v_init = min(v_des, dist_to_end / t_fin)
        if d0 > 1e-6:
            vx0, vy0 = v_init * dir_x / d0, v_init * dir_y / d0
        else:
            vx0, vy0 = 0.0, 0.0
        initial_state = jnp.hstack((x, y, vx0, vy0, 0.0, 0.0))

        try:
            arc_len, arc_vec, x_diff, y_diff = self.prob.path_spline(x_wp, y_wp)
            # 用 v_des 参数（原硬编码 0.5 → 前瞻仅 0.75m，速度被锁在 ~0.5；v_des=2.5 → 前瞻 3.75m）
            x_gp, y_gp = self.prob.compute_warm_traj(
                initial_state, v_des, x_wp, y_wp, arc_vec, x_diff, y_diff)
            x_obs_t, y_obs_t, x_obs_tp, y_obs_tp = self.prob.compute_obs_traj_prediction(
                self.x_obs.flatten(), self.y_obs.flatten(),
                jnp.zeros(420), jnp.zeros(420), x, y)
            sol_x, sol_y, xg, yg, xdg, ydg, xddg, yddg, cm, cc, x_fin, y_fin = self.prob.compute_traj_guess(
                initial_state, x_obs_t, y_obs_t, v_des, x_wp, y_wp, arc_vec, x_gp, y_gp, x_diff, y_diff)
            lam_x = jnp.zeros((110, self.prob.nvar))
            lam_y = jnp.zeros((110, self.prob.nvar))
            result = self.prob.compute_cem(
                self.key, initial_state, x_fin, y_fin, lam_x, lam_y,
                x_obs_t, y_obs_t, x_obs_tp, y_obs_tp,
                sol_x, sol_y, xg, yg, xdg, ydg, xddg, yddg, x_wp, y_wp, arc_vec, cm, cc)
            if result is None:
                self.get_logger().warn('compute_cem 返回 None')
                return None, None
            x_el, y_el, x_s, y_s, c_x, c_y, x_b, y_b, xdb, ydb, xg_p, yg_p = result
            self._publish_path(x_b, y_b)  # 发布最优轨迹供 rviz 显示
            vx_c, vy_c, ax_c, ay_c = self.prob.compute_controls(
                c_x, c_y, jnp.array(1.0 / self.control_frequency))
            # v_max 截断（num_sample 调大后输出可能超限，防止失控）
            mag = math.hypot(float(vx_c), float(vy_c))
            v_max = self.get_parameter('v_max').value
            if mag > v_max > 0:
                s = v_max / mag
                vx_c, vy_c = vx_c * s, vy_c * s
            return float(vx_c), float(vy_c)
        except Exception as e:
            self.get_logger().warn(f'PRIEST 计算异常: {e}')
            return None, None

    def _publish_path(self, xs, ys):
        """发布 PRIEST 最优轨迹（odom 系 -> map 系）到 /priest_path"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception:
            return
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        n = min(len(xs), len(ys), 200)
        for i in range(0, n, 5):  # 降采样显示
            x = float(xs[i]) + trans.transform.translation.x
            y = float(ys[i]) + trans.transform.translation.y
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _path_waypoints(self, x, y):
        """从全局路径取机器人前方的一段，插值到 1000 点作为 PRIEST waypoint"""
        pts = self.path_odom
        best = 0
        best_d = float('inf')
        for i, (px, py) in enumerate(pts):
            d = (px - x) ** 2 + (py - y) ** 2
            if d < best_d:
                best_d = d
                best = i
        # 从离机器人最近点开始（原 best-5 会包含身后点，初始方向反 → 轨迹反走+折线）
        seg = pts[best:]
        if len(seg) < 2:
            seg = pts
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        n = len(seg)
        if n == 1:
            return jnp.linspace(x, seg[0][0], 1000), jnp.linspace(y, seg[0][1], 1000)
        arcs = [0.0]
        for i in range(1, n):
            arcs.append(arcs[-1] + math.hypot(xs[i] - xs[i-1], ys[i] - ys[i-1]))
        total = arcs[-1]
        if total < 1e-6:
            return jnp.linspace(x, xs[-1], 1000), jnp.linspace(y, ys[-1], 1000)
        t = np.linspace(0.0, 1.0, 1000)
        x_interp = np.interp(t * total, arcs, xs)
        y_interp = np.interp(t * total, arcs, ys)
        return jnp.asarray(x_interp), jnp.asarray(y_interp)

    # ---------- 控制循环 ----------

    def control_loop(self):
        if not self.initialized:
            try:
                self._init_algorithm()
            except Exception as e:
                self.get_logger().error(f'算法初始化失败: {e}')
                self.initialized = True
            return

        # 无目标：停车等 rviz 2D Goal
        if not self.has_target:
            cmd = Twist()
            cmd.linear.x = 0.0; cmd.linear.y = 0.0; cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # 有目标但全局路径未就绪：停车等路径（避免直冲全局目标/切弯穿障碍）
        if self.path_odom is None or len(self.path_odom) < 2:
            cmd = Twist()
            cmd.linear.x = 0.0; cmd.linear.y = 0.0; cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # 到达目标：停止（避免局部轨迹到终点乱晃）
        stop_radius = self.get_parameter('stop_radius').value
        if math.hypot(self.x - self.target_x, self.y - self.target_y) < stop_radius:
            cmd = Twist()
            cmd.linear.x = 0.0; cmd.linear.y = 0.0; cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        # 贴障碍（<0.35m）：推离最近障碍，避免接触卡死（局部轨迹贴障碍时避不掉）
        min_d = 1e9; min_i = -1
        for i in range(420):
            d = math.hypot(float(self.x_obs[i]) - self.x, float(self.y_obs[i]) - self.y)
            if d < min_d:
                min_d = d; min_i = i
        if min_d < 0.35:
            dx = self.x - float(self.x_obs[min_i])
            dy = self.y - float(self.y_obs[min_i])
            dd = math.hypot(dx, dy)
            if dd > 1e-6:
                evx, evy = 0.4 * dx / dd, 0.4 * dy / dd
                gy = self.gimbal_yaw
                cmd = Twist()
                cmd.linear.x = evx * math.cos(gy) + evy * math.sin(gy)
                cmd.linear.y = -evx * math.sin(gy) + evy * math.cos(gy)
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return

        vx_w, vy_w = self._compute_step(self.x, self.y, self.vx, self.vy)
        if vx_w is None:
            return

        # 诊断：打印 PRIEST 输出（world 系）+ yaw + 目标
        if self.get_clock().now().nanoseconds % 3000000000 == 0:
            self.get_logger().info(
                f'pos=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.2f} '
                f'goal=({self.target_x:.2f},{self.target_y:.2f}) '
                f'PRIEST(world)=({vx_w:.3f},{vy_w:.3f})')

        # 对齐 MPPI 输出系：MPPI 输出是 gimbal_fake 系（chassis 用它旋转到 base 系后车才正确）。
        # PRIEST 输出是 world 系，需先转 +gimbal_yaw 到 gimbal_fake 系，再经 fake_vel_transform + chassis
        gy = self.gimbal_yaw
        cos_g, sin_g = math.cos(gy), math.sin(gy)
        cmd = Twist()
        cmd.linear.x = vx_w * cos_g + vy_w * sin_g
        cmd.linear.y = -vx_w * sin_g + vy_w * cos_g
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PriestBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
