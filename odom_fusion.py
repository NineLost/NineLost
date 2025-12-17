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
      - /odom_encoder : x, y, vx, wz
      - /odom_imu     : yaw
    Publish:
      - /odom
      - TF odom -> base_link  (LUÔN TỒN TẠI)
    """

    def __init__(self):
        super().__init__("odom_fusion")

        # --- Subscribers ---
        self.sub_encoder = self.create_subscription(
            Odometry, "/odom_encoder", self.cb_encoder, 20
        )
        self.sub_imu = self.create_subscription(
            Odometry, "/odom_imu", self.cb_imu, 20
        )

        # --- Publisher ---
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)

        # --- TF ---
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- State ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.wz = 0.0

        self.have_encoder = False
        self.have_imu = False

        # --- TF timer (QUAN TRỌNG) ---
        self.tf_timer = self.create_timer(0.05, self.publish_tf)

        self.get_logger().info("OdomFusion READY (standard odom)")

    # ================= CALLBACKS =================

    def cb_encoder(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z
        self.have_encoder = True

        if self.have_imu:
            self.publish_odom(msg.header.stamp)

    def cb_imu(self, msg: Odometry):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = 2.0 * math.atan2(qz, qw)
        self.have_imu = True

    # ================= PUBLISHERS =================

    def publish_odom(self, stamp):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

    def publish_tf(self):
        """
        TF LUÔN ĐƯỢC PUBLISH
        → đảm bảo frame 'odom' luôn tồn tại
        """
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


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


