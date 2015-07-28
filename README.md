# Overview
This workspace is designated for robot assisted and ultrasound guided project.

# Packages Descriptions
For the core funtionalities, please refer to packages active_echo_serial, ur5_control and sensor_description.

# User Guide
For simulation mode, find the package ursim-3.1 and type in ./start-ursim.sh in terminal. This will bring up the Universal Robots Graphical Programming Environment(URGPE). 

In another terminal, type ifconfig and copy the IP address which will be something like inet addr: 10.162.43.10. This is the "simulated IP address"of URGPE. 

In a thrid terminal, type roslaunch ur_bringup ur5_bringup.launch robot_ip:=10.162.43.10 to connect ROS with URGPE. 

In a fourth terminal, go to the catkin_ws and source devel/setup.bash. Then roslaunch ur5_contrl ur5.launch to launch the Rviz with UR5and the ultrasound sensor attached as the end-effector. 


For real hardware mode, to be continued ...
