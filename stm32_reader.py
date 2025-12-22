#!/usr/bin/env python3
import time
import serial
import re
import threading
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import JointState

ENC_PAT = re.compile(
    r"ENC\s+"                           # Từ khóa "ENC" + khoảng trắng
    r"ms=(\d+)\s+"                      # ms=<số> + khoảng trắng
    r"T=([-0-9]+),([-0-9]+),([-0-9]+),([-0-9]+)\s+"  # T=<4 số phân cách bởi dấu phẩy>
    r"D=([-0-9]+),([-0-9]+),([-0-9]+),([-0-9]+)"     # D=<4 số phân cách bởi dấu phẩy>
)


class VelocityBridgeVfmt(Node):
    """
    Node cầu nối vận tốc - Giao tiếp với STM32 điều khiển mecanum robot
    File này sẽ bao gồm điều khiển teleop + đọc ticks từ encoder + stm32
    Kiến trúc:
    ---------
    ┌─────────────┐      ┌──────────────┐      ┌─────────────┐
    │   Nav2 /    │ cmd_vel│  Velocity   │  TX  │   STM32     │
    │  Teleop     │──────→│   Bridge    │─────→│  Firmware   │
    └─────────────┘       │             │      └─────────────┘
                          │   (Node)    │  RX         ↓
    ┌─────────────┐ joint │             │←─────  Encoders
    │robot_state_ │ state │             │
    │ publisher   │←──────┴──────────────┘
    └─────────────┘
    
    Luồng dữ liệu:
    -------------
    1. Subscribe /cmd_vel (geometry_msgs/Twist) từ Nav2/teleop
    2. Giới hạn vận tốc trong max_vx, max_vy, max_wz
    3. Gửi "V vx vy wz" tới STM32 qua Serial (50Hz)
    4. Đọc "ENC ..." từ STM32 (thread riêng)
    5. Tính position và velocity từ encoder ticks
    6. Publish sensor_msgs/JointState cho robot_state_publisher
    
    Tham số cấu hình:
    ----------------
    SERIAL & TX:
    - serial_port: Cổng serial STM32 (mặc định /dev/ttyACM0)
    - baud: Baudrate (mặc định 115200)
    - rate_hz: Tần số gửi lệnh (mặc định 50Hz)
    - cmd_timeout_ms: Timeout lệnh (mặc định 200ms)
    - max_vx/vy/wz: Giới hạn vận tốc
    - zero_on_timeout: Gửi zero khi timeout (mặc định False)
    
    ENCODER & JOINT STATE:
    - ticks_per_rev: Xung encoder/vòng (mặc định 6864)
    - wheel_joint_names: Tên các joint bánh xe
    - invert_wheels: Đảo chiều bánh nào [FL, FR, RR, RL]
    
    LOGGING:
    - echo_tx: Hiện log lệnh gửi đi (mặc định True)
    - echo_rx: Hiện log dữ liệu nhận về (mặc định False)
    - tx_log_on_change_only: Chỉ log khi vận tốc thay đổi
    """
    
    def __init__(self):
        super().__init__('velocity_bridge')
        

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        
        # Tần số gửi lệnh (Hz)
        # 50Hz = 20ms/lần - cân bằng giữa độ mượt và tải CPU
        # Cao hơn (100Hz) = mượt hơn nhưng tốn CPU
        # Thấp hơn (20Hz) = giật hơn nhưng nhẹ
        self.declare_parameter('rate_hz', 50.0)
        
        # Timeout lệnh (milliseconds)
        # Nếu >200ms không nhận cmd_vel mới → coi như timeout
        # Nếu zero_on_timeout=True → gửi "V 0 0 0" để dừng robot
        self.declare_parameter('cmd_timeout_ms', 200)
        
        # Giới hạn vận tốc tối đa (m/s và rad/s)
        # Dùng để clamp các lệnh vượt quá khả năng robot
        self.declare_parameter('max_vx', 1.0)   # Vận tốc X tối đa (m/s)
        self.declare_parameter('max_vy', 1.0)   # Vận tốc Y tối đa (m/s)
        self.declare_parameter('max_wz', 2.0)   # Vận tốc góc tối đa (rad/s)
        
        # Watchdog - Gửi zero khi timeout?
        # True: Gửi "V 0 0 0" khi timeout để dừng robot (an toàn hơn)
        # False: Không gửi gì, STM32 tự xử lý timeout
        self.declare_parameter('zero_on_timeout', False)
        
        # Hiển thị log TX/RX?
        self.declare_parameter('echo_tx', True)   # Hiện lệnh gửi đi
        self.declare_parameter('echo_rx', False)  # Hiện dữ liệu nhận về (spam!)

        self.declare_parameter('ticks_per_rev', 6864.0)

        self.declare_parameter('wheel_joint_names', [
            'wheel_fl_joint',
            'wheel_fr_joint', 
            'wheel_rr_joint',
            'wheel_rl_joint'
        ])
        

        self.declare_parameter('invert_wheels', [True, True, True, True])
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = int(self.get_parameter('baud').value)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.timeout_s = float(self.get_parameter('cmd_timeout_ms').value) / 1000.0
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.zero_on_timeout = bool(self.get_parameter('zero_on_timeout').value)
        self.echo_tx = bool(self.get_parameter('echo_tx').value)
        self.echo_rx = bool(self.get_parameter('echo_rx').value)

        self.declare_parameter('tx_log_on_change_only', True)

        self.declare_parameter('tx_log_epsilon_v', 0.01)   # 1cm/s
        self.declare_parameter('tx_log_epsilon_w', 0.02)   # ~1°/s

        self.declare_parameter('tx_keepalive_sec', 0.0)
        
        self.tx_log_on_change_only = bool(self.get_parameter('tx_log_on_change_only').value)
        self.tx_log_eps_v = float(self.get_parameter('tx_log_epsilon_v').value)
        self.tx_log_eps_w = float(self.get_parameter('tx_log_epsilon_w').value)
        self.tx_keepalive_sec = float(self.get_parameter('tx_keepalive_sec').value)
        
        # Biến tracking cho log
        self._last_logged_cmd = None          # Lệnh cuối cùng được log
        self._last_log_wall = time.monotonic()  # Thời điểm log cuối (wall time)
        
        tpr = float(self.get_parameter('ticks_per_rev').value)
        self.rad_per_tick = 2.0 * math.pi / tpr
        
        # Đọc tên joint và cờ đảo chiều
        self.names = [str(x) for x in self.get_parameter('wheel_joint_names').value]
        self.invert = [bool(x) for x in self.get_parameter('invert_wheels').value]
        self._already_sent_zero = False  # Flag: Đã gửi zero chưa?             
        self.get_logger().info(f"Đang mở cổng serial: {port} @ {baud} baud")
        
        try:
            # Mở cổng serial
            # timeout=0.02: Đọc non-blocking, chờ tối đa 20ms
            self.ser = serial.Serial(
                port=port, 
                baudrate=baud, 
                timeout=0.02
            )
            
            # Đợi 200ms cho STM32 reset sau khi mở serial
            # Một số board STM32 tự reset khi DTR toggle
            time.sleep(0.2)
            
            self.get_logger().info("✅ Đã kết nối Serial thành công!")
            
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Không thể mở cổng serial {port}")
            self.get_logger().error(f"   Lỗi: {e}")
            self.get_logger().error(f"   Kiểm tra:")
            self.get_logger().error(f"   1. STM32 đã cắm USB chưa?")
            self.get_logger().error(f"   2. Cổng đúng chưa: ls -l /dev/ttyACM*")
            self.get_logger().error(f"   3. Có quyền truy cập: sudo chmod 666 {port}")
            raise
            
        except Exception as e:
            self.get_logger().error(f"❌ Lỗi không mong đợi: {e}")
            raise

        # =====================================================================
        # PUBLISHERS - Xuất bản dữ liệu
        # =====================================================================
        
        # Xuất bản dòng encoder thô (debug)
        # Topic: /enc/line (std_msgs/String)
        # Nội dung: "ENC ms=12345 T=1,2,3,4 D=5,6,7,8"
        self.enc_line_pub = self.create_publisher(String, 'enc/line', 10)
        
        # Xuất bản encoder total ticks (debug)
        # Topic: /enc/total (std_msgs/Int32MultiArray)
        # Nội dung: [T_FL, T_FR, T_RR, T_RL] - Tổng ticks từ lúc bật nguồn
        self.enc_total_pub = self.create_publisher(Int32MultiArray, 'enc/total', 10)
        
        # Xuất bản encoder delta ticks (debug)
        # Topic: /enc/delta (std_msgs/Int32MultiArray)
        # Nội dung: [D_FL, D_FR, D_RR, D_RL] - Ticks thay đổi từ lần đọc trước
        self.enc_delta_pub = self.create_publisher(Int32MultiArray, 'enc/delta', 10)
        
        # Xuất bản trạng thái khớp bánh xe (QUAN TRỌNG!)
        # Topic: /joint_states (sensor_msgs/JointState)
        # Dùng bởi robot_state_publisher để cập nhật TF tree
        self.js_pub = self.create_publisher(JointState, 'joint_states', 20)

        # =====================================================================
        # SUBSCRIBER & TIMERS
        # =====================================================================
        
        # Thời điểm nhận lệnh cmd_vel cuối cùng
        self.last_cmd_time = self.get_clock().now()
        
        # Lệnh vận tốc hiện tại (vx, vy, wz)
        self.last_cmd = (0.0, 0.0, 0.0)
        
        # Subscribe topic /cmd_vel từ Nav2/teleop
        # QoS: 10 (hàng đợi 10 message)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        
        # Timer gửi lệnh vận tốc định kỳ
        # Period = max(2ms, 1/rate_hz)
        # Ví dụ: rate=50Hz → timer mỗi 20ms
        self.create_timer(max(0.002, 1.0/self.rate), self._tick)

        # =====================================================================
        # THREAD NHẬN DỮ LIỆU (RX)
        # =====================================================================
        
        # Event để dừng thread
        self._stop = threading.Event()
        
        # Tạo thread daemon đọc Serial
        # daemon=True: Thread tự tắt khi chương trình chính tắt
        self._thr = threading.Thread(target=self._rx_loop, daemon=True)
        self._thr.start()

        # =====================================================================
        # BIẾN CHO TÍNH TOÁN VELOCITY TỪ ENCODER
        # =====================================================================
        
        # Timestamp (ms) của lần đọc encoder trước
        # Dùng để tính dt (delta time) cho velocity
        self._last_ms = None
        self._last_T = None
        # =====================================================================
        # LOG THÔNG TIN KHỞI ĐỘNG
        # =====================================================================
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ Velocity Bridge đã khởi động!")
        self.get_logger().info(f"   - Tần số gửi: {self.rate} Hz")
        self.get_logger().info(f"   - Timeout: {self.timeout_s} s")
        self.get_logger().info(f"   - Xung encoder/vòng: {tpr}")
        self.get_logger().info(f"   - Radian/tick: {self.rad_per_tick:.6f}")
        self.get_logger().info(f"   - Watchdog: {'BẬT' if self.zero_on_timeout else 'TẮT'}")
        self.get_logger().info("=" * 60)

    # =========================================================================
    # CLEANUP - Dọn dẹp khi tắt node
    # =========================================================================
    
    def destroy_node(self):
        self.get_logger().info("🛑 Đang tắt Velocity Bridge...")
        
        # Dừng thread RX
        self._stop.set()
        
        try:
            # Đợi thread kết thúc
            if self._thr.is_alive(): 
                self._thr.join(timeout=0.3)
        except Exception: 
            pass
        
        try: 
            # Đóng serial
            if hasattr(self, 'ser') and self.ser.is_open:
                self.ser.close()
                self.get_logger().info("🔌 Đã đóng cổng Serial")
        except Exception: 
            pass
        
        return super().destroy_node()

    # =========================================================================
    # TX - TRUYỀN LỆNH VẬN TỐC
    # =========================================================================
    
    def _on_cmd(self, msg: Twist):
        # Lambda function để giới hạn giá trị trong [lo, hi]
        clamp = lambda v, lo, hi: lo if v < lo else hi if v > hi else v
        
        # Clamp các vận tốc trong giới hạn
        vx = clamp(msg.linear.x,  -self.max_vx, self.max_vx)
        vy = clamp(msg.linear.y,  -self.max_vy, self.max_vy)
        wz = clamp(msg.angular.z, -self.max_wz, self.max_wz)
        
        # Lưu lệnh mới
        self.last_cmd = (vx, vy, wz)
        self.last_cmd_time = self.get_clock().now()
        
        # =====================================================================
        # ✅ SỬA LỖI MAJOR #8: RESET FLAG KHI CÓ LỆNH CHUYỂN ĐỘNG MỚI
        # =====================================================================
        # Khi có lệnh chuyển động (khác 0) → reset flag
        # Điều này cho phép gửi zero lại nếu sau này timeout
        # =====================================================================
        if vx != 0.0 or vy != 0.0 or wz != 0.0:
            self._already_sent_zero = False

    def _send_v(self, vx, vy, wz):
        # Tạo chuỗi lệnh theo protocol "V vx vy wz\n"
        # :.3f = Float với 3 chữ số thập phân
        line = f"V {vx:.3f} {vy:.3f} {wz:.3f}\n"
        
        try:
            # Gửi qua Serial (encode ASCII)
            self.ser.write(line.encode('ascii'))
            
            # ================================================================
            # LOGGING - Kiểm soát log để tránh spam
            # ================================================================
            if self.echo_tx:
                do_log = True  # Mặc định log
                
                # Kiểm tra: Chỉ log khi thay đổi?
                if self.tx_log_on_change_only:
                    if self._last_logged_cmd is None:
                        # Lần đầu tiên → log
                        do_log = True
                    else:
                        # So sánh với lệnh đã log trước
                        lvx, lvy, lwz = self._last_logged_cmd
                        
                        # Tính độ thay đổi lớn nhất
                        dv = max(abs(vx - lvx), abs(vy - lvy))  # Delta vận tốc tuyến tính
                        dw = abs(wz - lwz)                       # Delta vận tốc góc
                        
                        # Log nếu thay đổi > ngưỡng
                        do_log = (dv > self.tx_log_eps_v) or (dw > self.tx_log_eps_w)
                
                # Kiểm tra: Keepalive (log định kỳ dù không đổi)
                if not do_log and self.tx_keepalive_sec > 0.0:
                    now = time.monotonic()
                    if (now - self._last_log_wall) >= self.tx_keepalive_sec:
                        do_log = True
                
                # Thực hiện log nếu cần
                if do_log:
                    self.get_logger().info(f"📤 TX: {line.strip()}")
                    self._last_logged_cmd = (vx, vy, wz)
                    self._last_log_wall = time.monotonic()
                    
        except serial.SerialException as e:
            self.get_logger().warn(f'❌ Lỗi ghi Serial: {e}')
        except Exception as e:
            self.get_logger().warn(f'❌ Lỗi không mong đợi khi gửi: {e}')

    def _tick(self):
        # Tính tuổi của lệnh cuối cùng
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9  # Chuyển ns → s
           
        if self.zero_on_timeout and age > self.timeout_s:
            # Timeout! Không còn nhận cmd_vel mới
            
            # Chỉ gửi zero nếu CHƯA gửi lần nào
            if not self._already_sent_zero:
                # Gửi zero để dừng robot
                self._send_v(0.0, 0.0, 0.0)
                
                # Đánh dấu đã gửi
                self._already_sent_zero = True
                
                # Log cảnh báo
                self.get_logger().info(
                    f"⚠️ Cmd timeout ({age:.2f}s) - đã gửi ZERO một lần"
                )
            
            # Nếu đã gửi zero rồi → KHÔNG làm gì cả
            # (Không gửi lại zero, tránh spam)
            
        else:
            # Có lệnh trong thời hạn → gửi bình thường
            vx, vy, wz = self.last_cmd
            self._send_v(vx, vy, wz)

    # =========================================================================
    # RX - NHẬN DỮ LIỆU ENCODER
    # =========================================================================
    
    def _rx_loop(self):
        # Buffer lưu trữ bytes chưa xử lý
        buf = b''
        
        # Vòng lặp chính của thread
        while not self._stop.is_set():
            try:
                # ============================================================
                # BƯỚC 1: ĐỌC TỪ SERIAL
                # ============================================================
                # Đọc tối đa 256 bytes từ Serial
                # timeout=0.02s (set khi mở Serial)
                chunk = self.ser.read(256)
                buf += chunk
                
                # Nếu buffer chưa có dòng hoàn chỉnh → đợi thêm
                if b'\n' not in buf:
                    time.sleep(0.002)  # Ngủ 2ms để không spam CPU
                    continue
                
                # ============================================================
                # BƯỚC 2: TÁCH DÒNG
                # ============================================================
                # Split buffer thành các dòng
                # parts[-1] là phần dư chưa có '\n'
                parts = buf.split(b'\n')
                buf = parts[-1]  # Giữ lại phần dư
                
                # ============================================================
                # BƯỚC 3: XỬ LÝ TỪNG DÒNG
                # ============================================================
                for raw in parts[:-1]:  # Bỏ phần tử cuối (phần dư)
                    # Decode bytes → string
                    # errors='ignore': Bỏ qua bytes không hợp lệ
                    line = raw.decode(errors='ignore').strip()
                    
                    if not line:  # Dòng trống → skip
                        continue
                    
                    # Log dòng nhận được (nếu echo_rx=True)
                    if self.echo_rx: 
                        self.get_logger().info(f"📥 RX: {line}")
                    
                    # Publish dòng thô (cho debug)
                    self.enc_line_pub.publish(String(data=line))

                    # ========================================================
                    # BƯỚC 4: PARSE ENCODER DATA
                    # ========================================================
                    # Dùng regex để extract các số
                    m = ENC_PAT.match(line)
                    if not m:  # Không khớp pattern → skip
                        continue
                    
                    # Extract timestamp (ms)
                    ms = int(m.group(1))
                    
                    # Extract Total ticks (4 bánh)
                    # Groups 2-5: T values
                    T = [int(m.group(i)) for i in range(2, 6)]
                    # T = [T_FL, T_FR, T_RR, T_RL]
                    
                    # Extract Delta ticks (4 bánh)
                    # Groups 6-9: D values
                    d = [int(m.group(i)) for i in range(6, 10)]
                    # d = [D_FL, D_FR, D_RR, D_RL]
                    
                    # ========================================================
                    # BƯỚC 5: PUBLISH DỮ LIỆU THÔ (DEBUG)
                    # ========================================================
                    # Publish total ticks
                    self.enc_total_pub.publish(Int32MultiArray(data=T))
                    
                    # Publish delta ticks
                    self.enc_delta_pub.publish(Int32MultiArray(data=d))
                    
                    # ========================================================
                    # BƯỚC 6: TÍNH JOINT STATE
                    # ========================================================
                    
                    # ------ Position từ total ticks ------
                    # Công thức: position (rad) = ticks × (2π / ticks_per_rev)
                    pos = [t * self.rad_per_tick for t in T]
                    # pos = [pos_FL, pos_FR, pos_RR, pos_RL] (rad)
                    
                    # ------ Velocity từ DELTA T (chênh lệch total ticks) ------
                    # ✅ SỬA: Dùng Delta T thay vì giá trị D từ STM32
                    if self._last_ms is not None and hasattr(self, '_last_T'):
                        # Tính delta time (s)
                        dt = (ms - self._last_ms) / 1000.0
                        
                        # Kiểm tra dt hợp lệ (0 < dt < 0.5s)
                        if 0.0 < dt < 0.5:
                            # ✅ TÍNH DELTA T: Chênh lệch total ticks giữa 2 lần đo
                            delta_T = [T[i] - self._last_T[i] for i in range(4)]
                            
                            # Công thức: velocity = (delta_T / dt) × (2π / tpr)
                            vel = [dT * self.rad_per_tick / dt for dT in delta_T]
                        else:
                            vel = [0.0] * 4
                    else:
                        vel = [0.0] * 4

                    # Lưu total ticks hiện tại cho lần tính sau
                    self._last_T = T
                    
                    # Cập nhật timestamp cho lần sau
                    self._last_ms = ms
                    
                    # ------ Invert bánh xe nếu cần ------
                    # Dùng list comprehension với điều kiện
                    # Nếu invert[i]=True → đảo dấu
                    pos = [(-p if inv else p) for p, inv in zip(pos, self.invert)]
                    vel = [(-v if inv else v) for v, inv in zip(vel, self.invert)]
                    
                    # ========================================================
                    # BƯỚC 7: PUBLISH JOINT STATE
                    # ========================================================
                    # Tạo message JointState
                    js = JointState()
                    
                    # Header với timestamp
                    js.header.stamp = self.get_clock().now().to_msg()
                    
                    # ✅ SỬA LỖI CRITICAL: js.n → js.name
                    # LỖI CŨ: js.n = self.names  # ❌ Không có thuộc tính .n
                    # SỬA: js.name = self.names  # ✅ Thuộc tính đúng
                    js.name = self.names
                    
                    # Position và velocity
                    js.position = pos  # [rad, rad, rad, rad]
                    js.velocity = vel  # [rad/s, rad/s, rad/s, rad/s]
                    
                    # Publish
                    self.js_pub.publish(js)
                    
            except serial.SerialException as e:
                # Lỗi Serial (mất kết nối, timeout)
                self.get_logger().error(
                    f'❌ Lỗi Serial trong RX: {e}',
                    throttle_duration_sec=5.0
                )
                time.sleep(0.1)
                
            except Exception as e:
                # Lỗi khác (parse, decode, ...)
                self.get_logger().debug(
                    f'⚠️ Lỗi RX (non-critical): {e}',
                    throttle_duration_sec=5.0
                )
                time.sleep(0.01)


def main():
    # Khởi tạo ROS2
    rclpy.init()
    
    try:
        # Tạo node
        node = VelocityBridgeVfmt()
        
        # Chạy node (vòng lặp xử lý callbacks)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # Người dùng nhấn Ctrl+C
        print("\n⚠️ Đã nhận Ctrl+C, đang tắt...")
        
    except Exception as e:
        # Lỗi không mong đợi
        print(f"❌ Lỗi nghiêm trọng: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Cleanup
        if 'node' in locals():
            node.destroy_node()
        
        rclpy.shutdown()
        print("✅ Đã tắt Velocity Bridge")


if __name__ == '__main__':
    main()
