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
# Problem
1. 下面的命令不能在conda 环境中输入，要在系统环境中输入
   $ rosdep install -iyr --from-paths src
   $ colcon build
# camera rgbd launch and subscriber
```bash

方法一，参见官网：https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#usage

方法二，如下：

1. write camera launch file  realsense_l515.launch.py :
    
    from launch import LaunchDescription
    from launch_ros.actions import Node
    
    def generate_launch_description():
    
    It seems Conda Environment's libtiff4.5 was the problem. Did the following for resolving:
    
        conda uninstall libtiff
        conda install libtiff=4.0.8
        pip install Pillow==2.2.2
    
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

4. problem and solutions:
(1) Import Error: /lib/libgdal.so.26: undefined symbol: TIFFReadRGBATileExt, version LIBTIFF_4.0
#It seems Conda Environment's libtiff4.5 was the problem. Did the following for resolving:
    conda uninstall libtiff
    conda install libtiff=4.0.8
    pip install Pillow==2.2.2

(2) OpenCV(4.2.0) /io/opencv/modules/highgui/src/window.cpp:634: error: (-2:Unspecified error) The function is not implemented #323 
    pip uninstall opencv-python-headless -y 
    pip install opencv-python --upgrade
```
