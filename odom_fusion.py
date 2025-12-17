#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomFusion(Node):
    """
    Fuse:
      - /odom_encoder (x, y, vx, wz)
      - /odom_imu     (yaw)
    → xuất /odom + TF odom -> base_link
    """

    def __init__(self):
        super().__init__("odom_fusion")

        self.encoder_sub = self.create_subscription(
            Odometry, "/odom_encoder", self.cb_encoder, 20
        )
        self.imu_sub = self.create_subscription(
            Odometry, "/odom_imu", self.cb_imu, 20
        )

        self.pub = self.create_publisher(Odometry, "/odom", 20)
        self.tf_pub = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_encoder = None
        self.last_imu_yaw = 0.0

        # Timer debug: 0.5s gửi TF 1 lần để đảm bảo TF tồn tại
        self.debug_timer = self.create_timer(0.5, self.debug_timer_cb)

        self.get_logger().info("Odom Fusion started: encoder + imu")

    def cb_encoder(self, msg: Odometry):
        self.last_encoder = msg

    def cb_imu(self, msg: Odometry):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        self.last_imu_yaw = yaw
        self.update_odom()

    def update_odom(self):
        if self.last_encoder is None:
            return

        enc = self.last_encoder
        stamp = enc.header.stamp

        # X, Y từ encoder
        self.x = enc.pose.pose.position.x
        self.y = enc.pose.pose.position.y

        # Yaw từ IMU
        self.th = self.last_imu_yaw

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"

        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist = enc.twist

        self.pub.publish(od)

        # TF từ dữ liệu thật
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_pub.sendTransform(t)

    def debug_timer_cb(self):
        """
        Timer debug: nếu vì lý do nào đó update_odom chưa chạy,
        ta vẫn gửi một TF (x,y,theta hiện tại – mặc định 0,0,0).
        Mục đích: đảm bảo frame 'odom' & 'base_link' tồn tại trong TF tree.
        """
        now = self.get_clock().now().to_msg()

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

