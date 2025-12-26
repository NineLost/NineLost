from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # =========================
    # Launch arguments
    # =========================
    map_yaml = LaunchConfiguration('map')
    amcl_params = LaunchConfiguration('amcl_params')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/lota/ros2_ws/my_map.yaml',
        description='Full path to map yaml file'
    )

    declare_amcl = DeclareLaunchArgument(
        'amcl_params',
        default_value=os.path.join(
            get_package_share_directory('my_robot_config'),
            'config',
            'amcl.yaml'
        ),
        description='AMCL parameter file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    # =========================
    # Robot odometry + IMU + EKF
    # =========================
    encoder_imu_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_config'),
                'launch',
                'encoder_imu_odom.launch.py'
            )
        )
    )

    # =========================
    # Dual LiDAR + merge
    # =========================
    dual_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot_config'),
                'launch',
                'dual_lidar_with_merge.launch.py'
            )
        )
    )

    # =========================
    # Map server (Lifecycle)
    # =========================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml}
        ]
    )

    # =========================
    # AMCL (Lifecycle)
    # =========================
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    # =========================
    # Lifecycle manager
    # =========================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # =========================
    # RViz
    # =========================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        declare_amcl,
        declare_use_sim_time,

        encoder_imu_odom,
        dual_lidar,

        map_server,
        amcl,
        lifecycle_manager,

        rviz
    ])
