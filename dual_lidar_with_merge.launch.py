from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # 1. LIDAR FRONT (góc trước-trái của robot)
    # =========================================================
    lidar_front = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_front',
        output='screen',
        parameters=[{
            # Đổi /dev/ttyUSB0 <-> /dev/ttyUSB1 nếu dây USB đảo
            'serial_port': '/dev/lidar_front',
            'serial_baudrate': 115200,
            'frame_id': 'laser_front',
            'angle_compensate': True,\
            'scan_mode': 'Standard',
        }],
        remappings=[
            ('scan', 'scan_front'),
        ]
    )

    # =========================================================
    # 2. LIDAR REAR (góc sau-phải của robot) – DELAY 3s
    # =========================================================
    lidar_rear = TimerAction(
        period=3.0,   # đợi 3 giây cho front ổn rồi mới start rear
        actions=[
            Node(
                package='rplidar_ros',
                executable='rplidar_composition',
                name='rplidar_rear',
                output='screen',
                parameters=[{
                    'serial_port': '/dev/lidar_rear',
                    'serial_baudrate': 115200,
                    'frame_id': 'laser_rear',
                    'angle_compensate': True,
                    'scan_mode': 'Standard',
                }],
                remappings=[
                    ('scan', 'scan_rear'),
                ]
            )
        ]
    )

    # =========================================================
    # 3. TF: base_link -> laser_front
    #    Lidar đặt ở góc trước-trái, cách tâm ~0.505 m
    # =========================================================
    tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_front',
        arguments=[
            # x, y, z (m)
            # Tính từ tâm ra góc theo đường chéo:
            # x_front ≈ 0.419, y_front ≈ 0.281
            '0.43', '0.29', '0.05',
            # roll, pitch, yaw (rad) – tạm cho lidar nhìn về phía trước
            '3.14159', '0', '0',
            'base_link', 'laser_front'
        ]
    )

    # =========================================================
    # 4. TF: base_link -> laser_rear
    #    Lidar đặt ở góc sau-phải, đối diện với front
    # =========================================================
    tf_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_rear',
        arguments=[
            # x, y, z (m)
            # Đối xứng qua tâm: x_rear ≈ -0.419, y_rear ≈ -0.281
            '-0.43', '-0.29', '0.05',
            # quay 180 độ quanh trục z: lidar nhìn về phía sau
            '0', '0', '0',
            'base_link', 'laser_rear'
        ]
    )

    # =========================================================
    # 5. NODE MERGE (dual_lidar_tools.dual_lidar_merger)
    #    Merge + cắt mọi điểm nằm trong hình chữ nhật thân robot
    # =========================================================
    merger = TimerAction(
        period=5.0,   # đợi 5 giây cho 2 lidar ổn rồi mới merge
        actions=[
            Node(
                package='dual_lidar_tools',
                executable='dual_lidar_merger',
                name='dual_lidar_merger',
                output='screen',
                parameters=[{
                    # TOPIC INPUT/OUTPUT
                    'front_topic': '/scan_front',
                    'rear_topic': '/scan_rear',
                    'output_topic': '/scan_merged',
                    'output_frame': 'base_link',

                    # VỊ TRÍ 2 LIDAR TRONG base_link – PHẢI TRÙNG VỚI TF
                    'front_tx': 0.5,
                    'front_ty': 0.281,
                    'front_yaw': 3.14159,

                    'rear_tx': -0.5,
                    'rear_ty': -0.281,
                    'rear_yaw': 0.0,

                    # HÌNH CHỮ NHẬT THÂN ROBOT TRONG base_link
                    # L = 0.79 m -> L/2 = 0.395
                    # W = 0.53 m -> W/2 = 0.265
                    'body_x_min': -0.395,
                    'body_x_max':  0.395,
                    'body_y_min': -0.265,
                    'body_y_max':  0.265,
                    
                    'merged_yaw_offset': 0.261,
                }],
            )
        ]
    )

    return LaunchDescription([
        lidar_front,
        lidar_rear,
        tf_front,
        tf_rear,
        merger,
    ])
