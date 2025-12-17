from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    imu_driver = Node(
        package='imu_hwt911',
        executable='hwt911_node',
        name='hwt911',
        output='screen',
        parameters=[{
            'port': '/dev/imu',
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
        executable='encoder_odom.py',
        name='encoder_odom',
        output='screen',
        parameters=[{
            'wheel_radius': 0.07,
            'base_width': 0.50,
            'ticks_per_rev': 6864.0,
        }]
    )

    odom_fusion = Node(
        package='my_robot_config',
        executable='odom_fusion.py',
        name='odom_fusion',
        output='screen'
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
        odom_fusion,
        wheel_tf_node,
    ])
