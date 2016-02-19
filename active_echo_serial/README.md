# Overview
>In order to use this package with real UR5, one has to follow the `Active Echo Manual.pdf` in the folder `References & Documents/Documents/Active Echo` to build up the "AUSPIS" system. The active echo (AE) catheter is connected to the receiver board, which will communiate with the control unit board. The control unit board is then connected to the workstation through an FTDI cable. 

This ROS package deals with reading active echo signals from the active echo controller into ROS system.
![ActiveEcho] (/Image/ActiveEchoPhantom.jpg?raw=true "Optional title")

The four outputs "`l_all, l_ta, dly, tc `" are combined into a ros message named `active_echo_serial/Num`. 

## Receiver Board

1. 
 
2. The sensitivity nub can be used to tune "tc"  

## Control Unit


## Ultrasound Machine
* Sync: trigger out A:1, trigger out B:2
* 
* Change line density to 128
* Image mode: general
* Put focus on AE point, approximately 4.5 cm or 6.0 cm
* Make Gain 100%
* Set Freq as 10.0M
# Note
Both the frame trigger and line trigger are needed, and the frame trigger has to happen before the line trigger. 

For the ultrasound machine which has only one output port, a function generator is needed to serve as the frame trigger.


# Work To Do
1. There is a delay between receiving the AE signal and showing the reference frame of the AE tip in Rviz. A possible reason might be the usage of "lookupTransform" function which depends on referring to the URDF model of UR5. A potential alternate is to use the forward kinematics of UR5 to generate the reference frame of the tip. 

2. The current control strategy is highly dependent on *tc*, however, *tc* is not very stable and can vary with pressure, depth of the AE element, and open holes above the veins. It will be helpful to perform experiments to see which factors contribute the most to the change of *tc*.
