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

# Learning ROS2: helpful commands
```bash
参考网站： https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
1. open rqt_graph by opening rqt
$ rqt_graph

2. (1) ros2 topic list
$ ros2 topic list
(2) ros2 topic TYPE
$ ros2 topic type /joint_states
(3) return the same list of topics, this time with the topic type appended in brackets
$ ros2 topic list -t
```

# camera rgbd launch and subscriber
```bash
1. write camera launch file  realsense_l515.launch.py :

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense_camera',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'depth_module.profile': '640x480x30',
                'rgb_camera.profile': '640x480x30',
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False
            }]
        )
    ])


2. edit setup.py file
change $('share/' + package_name + '/launch', glob('launch/*.launch.py')), to the following:
$ ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

3. build the workspace
colcon build
source install/setup.bash
ros2 launch ur5e_joystick_control realsense_l515.launch.py
```
   
