#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class WheelTFNode(Node):
    def __init__(self):
        super().__init__('wheel_tf_node')

        self.static_broadcaster = StaticTransformBroadcaster(self)

        transforms = []

        def make_tf(child, x, y, z=0.0):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            return t

        # Vị trí (±0.25, ±0.24)
        transforms.append(make_tf("wheel_fl_mount",  0.25,  0.24))
        transforms.append(make_tf("wheel_fr_mount",  0.25, -0.24))
        transforms.append(make_tf("wheel_rl_mount", -0.25,  0.24))
        transforms.append(make_tf("wheel_rr_mount", -0.25, -0.24))

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Wheel TF Node started (static transforms).")

def main(args=None):
    rclpy.init(args=args)
    node = WheelTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
