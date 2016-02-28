# Overview
>In order to use this package with real UR5, one has to follow the `Active Echo Manual.pdf` in the folder `References & Documents/Documents/Active Echo` to build up the "AUSPIS" system. The active echo (AE) catheter is connected to the receiver board, which will communiate with the control unit board. The control unit board is then connected to the workstation through an FTDI cable. 

This ROS package deals with reading active echo signals from the active echo controller into ROS system.
![ActiveEcho] (/Image/ActiveEchoPhantom.jpg?raw=true "Optional title")

The four outputs "`l_all, l_ta, dly, tc `" are combined into a ros message named `active_echo_serial/Num`. 


## Serial Port
First, test the serial port communication in a Windows system. Then, in Ubuntu, open the GtkTerm by `gtkterm` and select /dev/ttyUSB0 in Port and 921600 as the baud rate. Sometimes, if you accidently force quitting the GtkTerminal then it will not be able to parse the signal correctly when turned back on again. There are several ways to fix the issues.

* Close the GtkTerminal by clicking the cross button the top left. Note that this is different from using `Ctrl + c` in the orignal terminal. Then unplug the USB and replug it into the same USB port. Bring up the GtkTerminal and repeat the process as described above. Try `sudo gtkterm` if `gtkterm` is not working. 


Second, bring up roscore and `rosrun active_echo_serial active_echo_serial`. Note that you should leave the previously opened GtkTerminal on as you rosrun the rosnode. 

## Receiver Board

1. 
 
2. The sensitivity nub can be used to tune "tc". Note that an appropriate position of the sensitivity nub is very important for both the stability and the detection range of the US-AE system. The medium (water/phantom), the probe that is used, and the position of the focus point.    

## Control Unit


## Ultrasound Machine
* Sync: trigger out A:1 (line trigger), trigger out B:2 (frame trigger).The frame trigger should go to the 2nd port counted from the right side.
* Change line density to 128
* Image mode: general
* Put focus on AE point, approximately 4.5 cm or 6.0 cm
* Make Gain 100%
* Set Freq as 10.0M
# Note
1. Both the frame trigger and line trigger are needed, and the frame trigger has to happen before the line trigger. 

2. For the ultrasound machine which has only one output port, a function generator is needed to serve as the frame trigger.

3. Remember to disconnect the receiver board and the control unit after finishing an experiment. 

4. Turn off the Sonix software before you change the US probe.


# Work To Do
1. There is a delay between receiving the AE signal and showing the reference frame of the AE tip in Rviz. A possible reason might be the usage of "lookupTransform" function which depends on referring to the URDF model of UR5. A potential alternate is to use the forward kinematics of UR5 to generate the reference frame of the tip. 

2. The current control strategy is highly dependent on *tc*, however, *tc* is not very stable and can vary with pressure, depth of the AE element, and open holes above the veins. It will be helpful to perform experiments to see which factors contribute the most to the change of *tc*.

## Maximum of tc and dly (depth of the AE element)
1. In the water tank, 
