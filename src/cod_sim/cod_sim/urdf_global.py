#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# urdf_global.py — 把机器人 URDF 设为全局 /robot_description 参数
#
# rviz 的 RobotModel 默认从全局 /robot_description 参数读 URDF，
# 但 robot_state_publisher 的参数在 /red_standard_robot1 命名空间下，
# rviz 读不到导致模型标红。本节点用 xmacro 重新生成 URDF 并设为全局参数。

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from xmacro.xmacro4sdf import XMLMacro4sdf
from sdformat_tools.urdf_generator import UrdfGenerator
from std_msgs.msg import String


class UrdfGlobal(Node):
    def __init__(self):
        super().__init__('urdf_global')
        self.declare_parameter('color', 'red')
        color = self.get_parameter('color').value

        pkg = get_package_share_directory('fzsd2025_robot_description')
        xmacro_path = os.path.join(
            pkg, 'resource', 'xmacro', 'fzsd2025_sentry_robot.sdf.xmacro')

        xm = XMLMacro4sdf()
        xm.set_xml_file(xmacro_path)
        xm.generate({'global_initial_color': color})
        ug = UrdfGenerator()
        ug.parse_from_sdf_string(xm.to_string())
        urdf = ug.to_string()

        # 全局参数 robot_description（节点私有，rviz 读不到，仅存档用）
        self.declare_parameter('robot_description', urdf)

        # 发布 /robot_description 话题：rviz RobotModel 订阅要求 TRANSIENT_LOCAL，
        # 用 transient local + 周期重发（保证 rviz 晚订阅也能收到）
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, 'robot_description', qos)
        self._urdf_msg = String()
        self._urdf_msg.data = urdf
        self.pub.publish(self._urdf_msg)
        self.create_timer(2.0, self._republish)
        self.get_logger().info(
            f'UrdfGlobal: 已发布 /robot_description 话题（{len(urdf)} 字节，transient local）')

    def _republish(self):
        self.pub.publish(self._urdf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UrdfGlobal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
