# Overview
This workspace is designated for robot assisted and ultrasound guided project.

# Packages Descriptions
1. `ur5_control`: 
2. `active_echo_serial`:
3. `virtual_vein`:
4. `dynamic_reconfig`:
5. `sensor_description`:

# User Guide 
The current packages are compatible with ROS Hydro/Indigo. Compatibility with other ROS versions are yet to be tested. 

## Simulation Mode
>In order to run the simulation mode, one needs the *ursim-3.1* software package provided the *Universal Robots Company* acquired by *Teradyne* company. 

1. For simulation mode, find the package ursim-3.1 and type in _./start-ursim.sh_ in terminal. This will bring up the Universal Robots Graphical Programming Environment(URGPE). 

2. In another terminal, type ifconfig and copy the IP address which will be something like _inet addr: 10.162.43.10_. This is the "simulated IP address"of URGPE. 

3. In a thrid terminal, type `roslaunch ur_bringup ur5_bringup.launch robot_ip:=10.162.43.10` to connect ROS with URGPE. 

4. In a fourth terminal, go to the `catkin_ws and source devel/setup.bash`. Then `roslaunch ur5_control ur5.launch` to launch the Rviz with UR5 and the ultrasound sensor attached as the end-effector. 

5. In a fifth terminal, `rosrun active_echo_serial segment_pose_publisher`.

6. In a sixth terminal, `rosrun active_echo_serial segment_iamge`.

7. In a seventh terminal, `rostopic pub -r 10 /active_echo_data active_echo_serial/Num "l_all: 256 l_ta: 60 dly:2000 tc:10"`.

8. In an eigth termina, `rosrun rqt_reconfigure rqt_reconfigure`.    

## Real Robot Mode
>Be aware of the type of the probe attached as the end-effector of the robot. Currently, there are two types of US probes available: one is 3 mm small linear probe and the other is 4.5 mm larger linear probe. 

1. For real robot mode, bring up the UR5 and connect it to the workstation with the ethernet cable. 

2. In the UR5 teach pendant, go to `Setup Robot` and click `Setup Network`. Next, set up the network connection in Ubuntu (Linux). Use the `Default gateway` of UR5 as the `Address` under the `IPv4 Settings`. Take the ROS tutorial for `universal_robot` for an example [Getting Started with a Universal Robot and ROS-Industrial](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial), then `Address = 192.168.1.1`,`Netmask = 255.255.255.0` and `Gateway = 192.168.1.2`. 

3. Connect the Control Unit board and the designated workstation using an FTDI cable.  


# Current Problems
> For detailed problems, please refer to each individual ROS package. 

Serial communication via termios in Ubuntu14.04 doesn't work. `open_port` cannot open the serial port ttyUSB0. 

When the serial communication works (in Ubuntu 12.04 on Mac), `roslaunch ur5_control ur5.launch` and `rosrun active_echo_serial active_echo_serial` will cause rviz to crash.Current test shows that the rate at which `active_echo_serial/Num` is published is not the cause of the issue.

Ignore the active data when the count number is zero. Fix the vertical motion of the robot. Try to maximize the count number instead of using the Gaussian distribution to serve as the position feedback control law. 



To be continued. 
