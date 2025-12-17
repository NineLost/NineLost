#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray


class EncoderOdom(Node):
    """
    Nhận /wheel_ticks (FL, FR, RL, RR) từ STM32,
    tính odometry (x, y, theta) và publish /odom_encoder (KHÔNG TF).
    """

    def __init__(self):
        super().__init__('encoder_odom')

        # Tham số cơ khí
        self.R = self.declare_parameter('wheel_radius', 0.07).get_parameter_value().double_value
        self.L = self.declare_parameter('base_width', 0.50).get_parameter_value().double_value
        self.TPR = self.declare_parameter('ticks_per_rev', 6864.0).get_parameter_value().double_value

        # Odom state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_left = None
        self.last_right = None
        self.last_stamp = None

        # Publisher ONLY odom, KHÔNG TF
        self.pub = self.create_publisher(Odometry, '/odom_encoder', 20)

        # Subscribe ticks từ STM32
        self.sub = self.create_subscription(
            Int32MultiArray, '/wheel_ticks', self.cb_ticks, 20
        )

        self.get_logger().info("EncoderOdom started")

    def cb_ticks(self, msg: Int32MultiArray):
        # Giả định: [FL, FR, RL, RR]
        if len(msg.data) < 4:
            self.get_logger().warn(f"wheel_ticks length={len(msg.data)} < 4")
            return

        fl, fr, rl, rr = msg.data
        left = (fl + rl) / 2.0
        right = (fr + rr) / 2.0

        stamp = self.get_clock().now()

        if self.last_left is None:
            self.last_left = left
            self.last_right = right
            self.last_stamp = stamp
            return

        dt = (stamp - self.last_stamp).nanoseconds / 1e9
        if dt <= 0.0:
            return

        dL = left - self.last_left
        dR = right - self.last_right

        self.last_left = left
        self.last_right = right
        self.last_stamp = stamp

        # meters per tick
        meters_per_tick = (2.0 * math.pi * self.R) / self.TPR
        dL_m = dL * meters_per_tick
        dR_m = dR * meters_per_tick

        d_center = (dL_m + dR_m) / 2.0
        d_th = (dR_m - dL_m) / self.L

        self.th += d_th
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)

        vx = d_center / dt
        wz = d_th / dt

        self.publish_odom(stamp, vx, wz)

    def publish_odom(self, stamp, vx, wz):
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        # pose được hiểu là trong hệ "odom"
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

