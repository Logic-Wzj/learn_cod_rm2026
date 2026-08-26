#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# pcd_publisher.py — 场地点云地图发布器（读 PCD + 降采样 + 周期发布）
#
# rviz 需要持续收到点云话题（transient local + 周期重发），pcl_ros 的
# pcd_to_pointcloud 发布一次就退出，导致 rviz 收不到。本节点读 PCD、
# 降采样（1870万点太大）后周期发布。

import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

FIELDS = ['x', 'y', 'z', 'intensity', 'normal_x', 'normal_y', 'normal_z', 'curvature']
PT_SIZE = 32  # 8 个 float32


def read_pcd_binary(path, stride=20):
    """读 binary PCD，降采样返回 xyz 列表"""
    with open(path, 'rb') as f:
        header = b''
        while True:
            line = f.readline()
            header += line
            if line.startswith(b'DATA'):
                break
        data = f.read()
    points = []
    n = len(data)
    for i in range(0, n, PT_SIZE * stride):
        if i + 12 > n:
            break
        x, y, z = struct.unpack_from('fff', data, i)
        points.append((x, y, z))
    return points


class PcdPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', 'map_cloud')
        self.declare_parameter('stride', 20)
        self.declare_parameter('publish_period', 2.0)

        pcd_file = self.get_parameter('pcd_file').value
        frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        stride = self.get_parameter('stride').value
        period = self.get_parameter('publish_period').value

        self.get_logger().info(f'PcdPublisher: 读取 {pcd_file}（降采样 stride={stride}）...')
        self._points = read_pcd_binary(pcd_file, stride)
        self.get_logger().info(f'PcdPublisher: 读取 {len(self._points)} 点')

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(PointCloud2, topic, qos)
        self.create_timer(period, self._publish)
        self._publish()
        self.get_logger().info(f'PcdPublisher: 发布到 {topic}（transient local, 每 {period}s）')

    def _publish(self):
        msg = PointCloud2()
        msg.header = Header(frame_id=self.get_parameter('frame_id').value)
        msg.height = 1
        msg.width = len(self._points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.is_dense = True
        # 打包 xyz
        buf = bytearray()
        for (x, y, z) in self._points:
            buf += struct.pack('fff', x, y, z)
        msg.data = bytes(buf)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PcdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
