#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuOdom(Node):
    """
    Publish yaw từ IMU → /odom_imu
    KHÔNG tạo odom
    KHÔNG publish TF
    """

    def __init__(self):
        super().__init__('imu_odom')

        self.sub = self.create_subscription(
            Imu, '/imu/data', self.cb_imu, 50
        )
        self.pub = self.create_publisher(Odometry, '/odom_imu', 20)

        self.get_logger().info("IMU Odom READY (yaw only)")

    def cb_imu(self, msg: Imu):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w

        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy*qy + qz*qz)
        )

        od = Odometry()
        od.header.stamp = self.get_clock().now().to_msg()
        od.pose.pose.orientation.z = math.sin(yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(yaw / 2.0)
        od.twist.twist.angular.z = msg.angular_velocity.z

        self.pub.publish(od)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    try:
        rclpy.spin(node)   # ⬅⬅⬅ BẮT BUỘC
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
