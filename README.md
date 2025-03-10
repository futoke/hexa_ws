source install/setup.bash

echo source ~/hexa_ws/install/setup.bash >> ~/.bashrc

colcon build

colcon build --packages-select hexa_servo hexa_description hexa_ik

-------------------------------------------------------------------

---- source install/setup.bash && ros2 launch hexa_description display.launch.py

source install/setup.bash && ros2 run hexa_servo servo

---- source install/setup.bash && ros2 launch ldlidar_node ldlidar_bringup.launch.py params_file:=lidar_params.yaml

source install/setup.bash && ros2 launch ldlidar_node ldlidar_slam.launch.py

source install/setup.bash && ros2 run mpu6050_driver mpu6050_driver

---- source install/setup.bash && ros2 run hexa_odom hexa_odom

---- source install/setup.bash && ros2 launch ros2_laser_scan_matcher scan_matcher.launch.py

source install/setup.bash && ros2 launch nav2_bringup navigation_launch.py

source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py params_file:=mapper_params_online_async.yaml

-------------------------------------------------------------------

source install/setup.bash && ros2 run hexa_fake_slam fake_slam

lidar: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0

servo: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

source install/setup.bash && ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py

source install/setup.bash && ros2 launch ldlidar_stl_ros2 ld19.launch.py

ros2 topic echo /joy

ros2 topic pub /joy sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.5, 0.0, 1.0, 1.0, 0.0, 0.0]}" --once

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once


sudo apt install ros-humble-tf-transformations

sudo apt install ros-humble-slam-toolbox

ros2 pkg list | grep slam_toolbox

source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py params_file:=slam_params.yaml

ros2 run tf2_tools view_frames

ros2 run tf2_ros tf2_echo odom base_link

ros2 topic hz /scan

sudo apt install -y i2c-tools python3-smbus

sudo apt install -y i2c-tools python3-smbus2

pip install filterpy

colcon build --packages-select ldlidar_component --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

set ROS_DOMAIN_ID=5

call C:\dev\ros2_humble\local_setup.bat

export ROS_DOMAIN_ID=5

source ~/.bashrc