import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('my_robot_config')

    # ======================================================
    # 1) Dual LiDAR merge (gồm front + rear + TF + merger)
    # ======================================================
    dual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'dual_lidar_with_merge.launch.py')
        )
    )

    # ======================================================
    # 2) SLAM TOOLBOX – chỉ khởi động sau khi LiDAR hoạt động
    # ======================================================
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_merged.yaml')

    slam_node = TimerAction(
        period=4.0,  # đợi lidar + merger ổn định để tránh lỗi scan missing
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_params],
                remappings=[
                    ('scan', '/scan_merged'),   # dùng dữ liệu merge
                ]
            )
        ]
    )

    # ======================================================
    # RETURN LAUNCH
    # ======================================================
    return LaunchDescription([
        dual_lidar_launch,
        slam_node,
    ])
