#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import time
import threading


class DieuKhienNode(Node):
    def __init__(self):
        super().__init__('dieukhien_node')

        # ===============================
        # Parameters
        # ===============================
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('hz', 30.0)
        self.declare_parameter('cmd_timeout', 0.3)  # seconds

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.hz = self.get_parameter('hz').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.dt = 1.0 / self.hz

        # ===============================
        # Velocity state
        # ===============================
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = time.time()

        # ===============================
        # Open serial
        # ===============================
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.01
            )
            time.sleep(2.0)  # STM32 reset
            self.get_logger().info(
                f"Serial opened {self.port} @ {self.baudrate}"
            )
        except Exception as e:
            self.get_logger().fatal(f"Cannot open serial: {e}")
            raise SystemExit

        # ===============================
        # ROS interfaces
        # ===============================
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ===============================
        # Control loop timer
        # ===============================
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info("DieuKhienNode started")

    # ==================================================
    # Receive cmd_vel
    # ==================================================
    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.last_cmd_time = time.time()

    # ==================================================
    # Main control loop
    # ==================================================
    def control_loop(self):
        now = time.time()

        # Watchdog: nếu mất cmd_vel -> stop mềm
        if now - self.last_cmd_time > self.cmd_timeout:
            self.vx *= 0.9
            self.vy *= 0.9
            self.wz *= 0.9

            if abs(self.vx) < 1e-3: self.vx = 0.0
            if abs(self.vy) < 1e-3: self.vy = 0.0
            if abs(self.wz) < 1e-3: self.wz = 0.0

        # Send UART
        try:
            msg = f"V {self.vx:.3f} {self.vy:.3f} {self.wz:.3f}\n"
            self.ser.write(msg.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    # ==================================================
    # Shutdown
    # ==================================================
    def destroy_node(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DieuKhienNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
