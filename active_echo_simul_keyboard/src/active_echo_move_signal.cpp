#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <active_echo_serial/Num.h>

// Only publish one time after receiving one subscribed topic 
bool publish = false;
// position of the active echo element
double x = 0.0; // m
double y = 0.0; // m
double z = 0.0; // m

void callback( const geometry_msgs::Point& point)
{


	// get the x,y,z of AE element in the US frame	
	x = point.x;
	y = point.y;
	z = point.z;

	publish = true;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "active_echo_move_signal");

	ros::NodeHandle node;
	ros::Publisher  pub = node.advertise<active_echo_serial::Num>("active_echo_data", 1);
	ros::Subscriber sub = node.subscribe("active_echo_position_topic", 10, callback);

	ros::Rate rate(70);

	// active echo signal
	active_echo_serial::Num signal;

	// parameters of the Gausssian dist. of signal->tc on x
	double a = 39.6629;
	double b = -0.0277;
	double c = 3.1947;

	// US probe parameters
	double element_w = 0.45; // mm
	double AE_SRate = 80*pow(10,6); // hz
	double SOS = 1480; // m/s
	int count = 0;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	while(node.ok()){
		/*
		try{
			listener.lookupTransform( "world",
					"ultrasound_sensor",
					ros::Time(0), 
					transform);
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}	
		*/

		if (publish == true){
			try{
				listener.lookupTransform( "world",
						"ultrasound_sensor",
						ros::Time(0), 
						transform);
			}
			catch (tf::TransformException &ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}	

			x = x - transform.getOrigin().x();
			y = y - transform.getOrigin().y();
			z = z - transform.getOrigin().z();
			std::cout << " x = " << x << std::endl;
			std::cout << " y = " << y << std::endl;
			std::cout << " z = " << z << std::endl;

			if (fabs(x) > 0.005) { 
				std::cout << "AE element is out of the detection range of US probe along x-axis." << std::endl;}	
			else if (fabs(y) > 0.030) { 
				std::cout << "AE element is out of the detection range of UR probe along y-axis" << std::endl;}
			else {	

				signal.l_ta = (int) (y*1000/element_w + 64.5);
				signal.dly = -(int) (z*AE_SRate/SOS);
				signal.tc = exp(pow((x*1000 - b),2)/(-pow(c,2)))*a;	

				pub.publish(signal);
				
			}
			publish = false;
		}

		ros::spinOnce();
		rate.sleep();

	}

	return 0;
}
