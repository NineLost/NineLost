from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_driver = Node(
        package='imu_hwt911',
        executable='hwt911_node',
        name='hwt911',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'baud': 57600,
        }]
    )
    imu_odom = Node(
        package='my_robot_config',
        executable='imu_odom.py',
        name='imu_odom',
        output='screen',
        parameters=[{
            'imu_topic': '/imu/data',
            'odom_topic': '/odom_imu',
        }]
    )
    encoder_odom = Node(
        package='my_robot_config',
        executable='encoder_dep_trai.py',
        name='encoder_odom',
        output='screen',
        parameters=[{
            'wheel_radius': 0.07,
            'lx': 0.235,
            'ly': 0.24,
        }]
    )
    stm32_reader = Node(
        package='my_robot_config',
        executable='stm32_reader.py',
        name='stm32_reader',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 115200,
        }]
   )
    wheel_tf_node = Node(
        package='my_robot_config',
        executable='wheel_tf_node.py',
        name='wheel_tf_node',
        output='screen'
    )
    return LaunchDescription([
        imu_driver,
        imu_odom,
        encoder_odom,
        stm32_reader,
        wheel_tf_node,
    ])
