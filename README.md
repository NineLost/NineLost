# các gói cần thiết
sudo apt install ros-humble-desktop
sudo apt install \
  ros-humble-rclpy \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-std-msgs \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf-transformations \
  ros-humble-robot-state-publisher
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-rplidar-ros
sudo apt install teleop_twist_keyboard

pip3 install \
  numpy \
  transforms3d \
  pyserial

sudo usermod -aG dialout $USER
sudo apt install python3-colcon-common-extensions

