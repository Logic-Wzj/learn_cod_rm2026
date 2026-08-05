#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# fake_odom.py — loopback 仿真用假里程计
#
# 订阅底盘速度指令（aft_cmd_vel，base_link 系），积分出位置，
# 发布 odom 话题 + TF(odom->base_link)，替代 small_point_lio 的里程计。
# 同时发布 /Odometry（本工程参数中 fake_vel_transform 订阅的话题名）。
#
# 帧链：map -> odom (静态, launch 中发布) -> base_link (本节点) -> base_link_fake (fake_vel_transform)

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.declare_parameter('cmd_vel_topic', 'aft_cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        # 本工程 fake_vel_transform 默认订阅 "Odometry"（大写），只读 yaw
        self.odom_pub_alt = self.create_publisher(Odometry, 'Odometry', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.sub = self.create_subscription(
            Twist, cmd_vel_topic, self.cmd_vel_cb, 10)

        self.last_time = None
        self.timer = self.create_timer(0.02, self.publish_odom)  # 50Hz
        self.get_logger().info(
            f'FakeOdom started: sub {cmd_vel_topic} -> pub {odom_topic} + Odometry, '
            f'TF {self.frame_id} -> {self.child_frame_id}')

    def cmd_vel_cb(self, msg: Twist):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0 or dt > 0.5:  # 丢弃异常间隔
            return

        # 底盘系速度积分到世界系（odom）
        self.yaw += msg.angular.z * dt
        vx_w = msg.linear.x * math.cos(self.yaw) - msg.linear.y * math.sin(self.yaw)
        vy_w = msg.linear.x * math.sin(self.yaw) + msg.linear.y * math.cos(self.yaw)
        self.x += vx_w * dt
        self.y += vy_w * dt

    def publish_odom(self):
        now = self.get_clock().now()
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.odom_pub.publish(msg)
        self.odom_pub_alt.publish(msg)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
