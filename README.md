# Overview
This workspace is designated for robot assisted and ultrasound guided project.

# Packages Descriptions
1. ur5_control: 
2. active_echo_serial:
3. virtual_vein:
4. dynamic_reconfig:
5. sensor_description:

# User Guide
1. For simulation mode, find the package ursim-3.1 and type in ./start-ursim.sh in terminal. This will bring up the Universal Robots Graphical Programming Environment(URGPE). 

2. In another terminal, type ifconfig and copy the IP address which will be something like inet addr: 10.162.43.10. This is the "simulated IP address"of URGPE. 

3. In a thrid terminal, type roslaunch ur_bringup ur5_bringup.launch robot_ip:=10.162.43.10 to connect ROS with URGPE. 

4. In a fourth terminal, go to the catkin_ws and source devel/setup.bash. Then roslaunch ur5_contrl ur5.launch to launch the Rviz with UR5 and the ultrasound sensor attached as the end-effector. 

5. 
