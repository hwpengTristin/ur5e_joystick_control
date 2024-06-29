# How to run the code in this repository
1. Clone the repository
2. Open the terminal and navigate to the repository folder
3. Run the following command to install the required packages:
```bash
colcon build
source install/setup.bash
# launch UR5E Node
ros2 launch ur_bringup ur5e.launch.py robot_ip:=192.168.0.121 launch_rviz:=false robot_controller:=joint_trajectory_controller initial_joint_controller:=forward_velocity_controller
# launch Joystick Node
ros2 launch ur5e_joystick_control ur5e_joystick_control.launch.py
```
