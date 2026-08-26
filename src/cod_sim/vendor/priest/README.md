# PRIEST 算法源码（vendored）

来源：https://github.com/fatemeh-rastgar/PRIEST（RA-L 2024 论文）

**PRIEST: Projection Guided Sampling-Based Optimization for Autonomous Navigation**
- arXiv: https://ar5iv.labs.arxiv.org/html/2309.08235

## 说明

- 只收了**算法核心**（`src/` 的 Python 文件 + `config/` + `params/`），
  原始仓库的 meshes/worlds/urdf（demo 模型、161M）未收录。
- 原始代码是 **ROS1 + Python + JAX**，本目录仅作算法参考/复用，
  不参与 colcon 构建。
- 计划：写一个 ROS2 Python 桥节点（cod_sim 内），订阅全局路径/odom/scan，
  调用 `planner_holonomic.py`（全向模型）输出 cmd_vel，替代 MPPI。

## 关键文件

- `src/planner_holonomic.py` —— 全向（holonomic）规划器，麦轮可用
- `src/planner_cem_dynamic.py` / `src/plnner_nonholonomic.py` —— 非全向/动态版本
- `src/mpc_*.py` —— 论文的 MPC 相关变体
- `params/costmap_common_params.yaml` —— 参数示例

## 依赖

- Python 3 + numpy + **jax**（GPU 加速，`pip install "jax[cuda]"`）
- 运行时需要 CUDA toolkit（当前机器 nvcc 未装，待装）
