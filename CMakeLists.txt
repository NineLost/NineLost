cmake_minimum_required(VERSION 3.8)
project(my_robot_config)

find_package(ament_cmake REQUIRED)

# Nếu sau này cần import rclpy / tf2 / nav_msgs,... ở C++ thì mới cần các dòng này
# find_package(rclpy REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

# 1) Cài launch + config
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}/
)

# 2) Cài các node Python (trong thư mục scripts/)
install(
  PROGRAMS
    scripts/imu_odom.py
    scripts/encoder_odom.py
    scripts/odom_fusion.py
    scripts/wheel_tf_node.py
    scripts/dieukhien_node.py
    scripts/stm32_wheel_ticks.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
