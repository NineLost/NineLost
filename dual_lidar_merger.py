import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class DualLidarMerger(Node):
    def __init__(self):
        super().__init__('dual_lidar_merger')

        # Topic input
        front_topic = self.declare_parameter(
            'front_topic', '/scan_front'
        ).get_parameter_value().string_value

        rear_topic = self.declare_parameter(
            'rear_topic', '/scan_rear'
        ).get_parameter_value().string_value

        # Frame + topic output
        self.output_frame = self.declare_parameter(
            'output_frame', 'base_link'
        ).get_parameter_value().string_value

        self.output_topic = self.declare_parameter(
            'output_topic', '/scan_merged'
        ).get_parameter_value().string_value

        # ============================================================
        # Vị trí 2 lidar trong base_link (DEFAULT KHỚP VỚI TF)
        # ============================================================

        # FRONT (giống TF FRONT: (0.43, 0.29), yaw = pi)
        self.front_tx = self.declare_parameter(
            'front_tx', 0.43
        ).get_parameter_value().double_value

        self.front_ty = self.declare_parameter(
            'front_ty', 0.29
        ).get_parameter_value().double_value

        self.front_yaw = self.declare_parameter(
            'front_yaw', 3.14159
        ).get_parameter_value().double_value

        # REAR (giống TF REAR: (-0.43, -0.29), yaw = 0)
        self.rear_tx = self.declare_parameter(
            'rear_tx', -0.43
        ).get_parameter_value().double_value

        self.rear_ty = self.declare_parameter(
            'rear_ty', -0.29
        ).get_parameter_value().double_value

        self.rear_yaw = self.declare_parameter(
            'rear_yaw', 0.0
        ).get_parameter_value().double_value

        # Offset xoay cho toàn bộ scan_merged (nếu cần tinh chỉnh sau)
        self.merged_yaw_offset = self.declare_parameter(
            'merged_yaw_offset', 0.0
        ).get_parameter_value().double_value

        # ==============================================================

        # Kích thước hình chữ nhật thân robot
        self.body_x_min = self.declare_parameter(
            'body_x_min', -0.395
        ).get_parameter_value().double_value

        self.body_x_max = self.declare_parameter(
            'body_x_max', 0.395
        ).get_parameter_value().double_value

        self.body_y_min = self.declare_parameter(
            'body_y_min', -0.265
        ).get_parameter_value().double_value

        self.body_y_max = self.declare_parameter(
            'body_y_max', 0.265
        ).get_parameter_value().double_value

        # Subscriber
        self.sub_front = self.create_subscription(
            LaserScan, front_topic, self.front_callback, 10
        )
        self.sub_rear = self.create_subscription(
            LaserScan, rear_topic, self.rear_callback, 10
        )

        # Publisher
        self.pub_merged = self.create_publisher(LaserScan, self.output_topic, 10)

        # Lưu scan mới nhất
        self.scan_front = None
        self.scan_rear = None

        self.get_logger().info(
            f'Merger listening on {front_topic} and {rear_topic}, '
            f'publishing {self.output_topic}'
        )
        self.get_logger().info(
            f'Robot body rectangle: x[{self.body_x_min}, {self.body_x_max}], '
            f'y[{self.body_y_min}, {self.body_y_max}]'
        )
        self.get_logger().info(
            f'Front lidar: tx={self.front_tx}, ty={self.front_ty}, yaw={self.front_yaw}'
        )
        self.get_logger().info(
            f'Rear lidar:  tx={self.rear_tx}, ty={self.rear_ty}, yaw={self.rear_yaw}'
        )
        self.get_logger().info(
            f'Merged yaw offset: {self.merged_yaw_offset} rad'
        )

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def front_callback(self, msg: LaserScan):
        self.scan_front = msg
        if self.scan_rear is not None:
            self.merge_and_publish()

    def rear_callback(self, msg: LaserScan):
        self.scan_rear = msg
        if self.scan_front is not None:
            self.merge_and_publish()

    # ------------------------------------------------------------------ #
    # Core logic
    # ------------------------------------------------------------------ #
    def transform_point(self, r, angle, tx, ty, yaw):
        """
        Biến 1 điểm (r, angle) trong frame sensor
        sang toạ độ (x, y) trong base_link.
        """
        x_s = r * math.cos(angle)
        y_s = r * math.sin(angle)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        x_b = tx + cos_yaw * x_s - sin_yaw * y_s
        y_b = ty + sin_yaw * x_s + cos_yaw * y_s

        return x_b, y_b

    def point_inside_body(self, x, y):
        return (self.body_x_min <= x <= self.body_x_max and
                self.body_y_min <= y <= self.body_y_max)

    def merge_and_publish(self):
        front = self.scan_front
        rear = self.scan_rear

        if front is None or rear is None:
            return

        # Toàn bộ vòng 360°
        angle_min = -math.pi
        angle_max = math.pi

        angle_increment = (
            front.angle_increment if front.angle_increment > 0.0 else 0.0058
        )

        num_bins = int(round((angle_max - angle_min) / angle_increment)) + 1
        merged_ranges = [float('inf')] * num_bins

        range_min = min(front.range_min, rear.range_min)
        range_max = max(front.range_max, rear.range_max)

        def accumulate_scan(scan, tx, ty, yaw):
            for i, r in enumerate(scan.ranges):
                if not math.isfinite(r):
                    continue
                if r < scan.range_min or r > scan.range_max:
                    continue

                angle_sensor = scan.angle_min + i * scan.angle_increment
                x_b, y_b = self.transform_point(r, angle_sensor, tx, ty, yaw)

                r_b = math.hypot(x_b, y_b)
                if r_b < range_min or r_b > range_max:
                    continue

                if self.point_inside_body(x_b, y_b):
                    continue

                # Góc trong base_link
                angle_b = math.atan2(y_b, x_b)

                # Có thể bù thêm offset toàn cục nếu cần
                angle_b += self.merged_yaw_offset

                if not (angle_min <= angle_b <= angle_max):
                    continue

                idx = int(round((angle_b - angle_min) / angle_increment))
                if 0 <= idx < num_bins and r_b < merged_ranges[idx]:
                    merged_ranges[idx] = r_b

        # Tích luỹ từ front + rear
        accumulate_scan(front, self.front_tx, self.front_ty, self.front_yaw)
        accumulate_scan(rear, self.rear_tx, self.rear_ty, self.rear_yaw)

        # Xuất LaserScan merged
        out = LaserScan()
        out.header.frame_id = self.output_frame

        if (front.header.stamp.sec > rear.header.stamp.sec) or (
            front.header.stamp.sec == rear.header.stamp.sec and
            front.header.stamp.nanosec >= rear.header.stamp.nanosec
        ):
            out.header.stamp = front.header.stamp
        else:
            out.header.stamp = rear.header.stamp

        out.angle_min = angle_min
        out.angle_max = angle_max
        out.angle_increment = angle_increment
        out.range_min = range_min
        out.range_max = range_max
        out.ranges = merged_ranges

        self.pub_merged.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DualLidarMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
