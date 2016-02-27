#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <tf/transform_listener.h>
#include <math.h>
#include <active_echo_serial/Num.h>


int main(int argc, char** argv){

	ros::init(argc, argv, "active_echo_move_signal");

	ros::NodeHandle node;
	ros::Publisher  pub = node.advertise<active_echo_serial::Num>("active_echo_data", 1);

	ros::Rate rate(70);
	
	// active echo signal
	active_echo_serial::Num signal;

	// position of the active echo element
	double x = 0.0; // m
	double y = 0.0; // m
	double z = 0.0; // m

	// parameters of the Gausssian dist. of signal->tc on x
	double a = 39.6629;
	double b = -0.0277;
	double c = 3.1947;

	// US probe parameters
	double element_w = 0.3; // mm
	double AE_SRate = 80*pow(10,6); // hz
	double SOS = 1480; // m/s

	tf::TransformListener listener;
	while(node.ok()){
		tf::StampedTransform transform;

		try{
			listener.lookupTransform("active_echo_position",
					         "ultrasound_sensor",
						 ros::Time(0), transform);
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}	

		// Get the current postion of AE in the US frame
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		z = transform.getOrigin().z();
		
		signal.l_ta = (int) (y*1000/element_w + 64.5);
		signal.dly = -(int)z*AE_SRate/SOS;
		signal.tc = exp(pow((x*1000 - b),2)/(-pow(c,2)))*a;	
		
		pub.publish(signal);
				
		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}
