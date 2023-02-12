Infracamera Racecar Simulator
=================

INSTALL
1. cd ~/catkin_ws/src
2. git clone "https://github.com/mit-racecar/racecar.git"
3. git clone "https://github.com/ros-drivers/ackermann_msgs.git"
4. git clone "https://github.com/kait-wakita/racecar_gazebo.git"
5. cd ..
6. catkin_make
7. rospack profile

USAGE

1. run gazeo "roslaunch racecar_gazebo racecar_robocar_camera.launch"
2. key control car "rosrun racecar_control keyboard_teleop.py"
