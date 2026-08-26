#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# tf_forward.py — 把命名空间下的机器人 TF 转发到全局 /tf
#
# 场景：robot_state_publisher 在 /red_standard_robot1 命名空间下发布 TF
# （话题 /red_standard_robot1/tf），rviz 订阅全局 /tf 看不到，导致模型不显示。
# 本节点订阅命名空间下的 TF 并转发到全局。

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage


class TfForward(Node):
    def __init__(self):
        super().__init__('tf_forward')
        self.declare_parameter('ns_tf_topic', '/red_standard_robot1/tf')
        self.declare_parameter('ns_tf_static_topic', '/red_standard_robot1/tf_static')
        self.declare_parameter('out_tf_topic', '/tf')
        self.declare_parameter('out_tf_static_topic', '/tf_static')
        ns_tf = self.get_parameter('ns_tf_topic').value
        ns_static = self.get_parameter('ns_tf_static_topic').value
        out_tf = self.get_parameter('out_tf_topic').value
        out_static = self.get_parameter('out_tf_static_topic').value

        self.tf_pub = self.create_publisher(TFMessage, out_tf, 10)
        # tf_static 需要 transient_local durability，否则 rviz 收不到静态 TF
        static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_pub = self.create_publisher(TFMessage, out_static, static_qos)
        self.create_subscription(TFMessage, ns_tf, self.tf_cb, 10)
        self.create_subscription(TFMessage, ns_static, self.static_cb, static_qos)
        self.get_logger().info(f'TfForward: {ns_tf} -> {out_tf}, {ns_static} -> {out_static}')

    def tf_cb(self, msg: TFMessage):
        self.tf_pub.publish(msg)

    def static_cb(self, msg: TFMessage):
        self.static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
