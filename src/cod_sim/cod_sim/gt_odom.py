#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# gt_odom.py — 用 Gazebo ground-truth 里程计替代 LIO 链路
#
# 订阅仿真底盘的 odometry（chassis_odometry_gt，world 系），
# 转发为 nav2 需要的 odom 话题 + TF(odom -> gimbal_yaw)。
# 定位 = 静态 TF map->odom（launch 里发布，取机器人出生位姿）+ 本节点的 odom。
# 无漂移，适合仿真验证 MPPI。

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GtOdom(Node):
    def __init__(self):
        super().__init__('gt_odom')

        self.declare_parameter('input_odom_topic', 'chassis_odometry_gt')
        self.declare_parameter('output_odom_topic', 'odometry')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'gimbal_yaw')
        input_topic = self.get_parameter('input_odom_topic').value
        output_topic = self.get_parameter('output_odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Odometry, input_topic, self.odom_cb, 10)
        self.get_logger().info(
            f'GtOdom started: sub {input_topic} -> pub {output_topic}, '
            f'TF {self.frame_id} -> {self.child_frame_id}')

    def odom_cb(self, msg: Odometry):
        # 转发消息，改 frame
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.child_frame_id = self.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.odom_pub.publish(out)

        # TF odom -> child
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = o
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GtOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
