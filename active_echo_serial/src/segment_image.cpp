#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <active_echo_serial/Num.h> // Active Echo Mmessage Header File
#include <dynamic_reconfigure/server.h> // Dynamic Reconfigure Header File
#include <dynamic_reconfig/ultrasound_ur5Config.h> // Ultra-UR5 Dynamic Reconfigure Header File


//This node subscribes to the ROS topic /active_echo_data and publishes
//the coordinate information of the segmented point.
 
static bool g_mid_plane = true;

void dynamiconfigCallback(dynamic_reconfig::ultrasound_ur5Config &config, uint32_t level)
{
	g_mid_plane = config.in_plane_assumption;
	ROS_INFO("Reconfigure Request: %s",
		  config.in_plane_assumption?"True":"False");
}


void segmentCallback(const active_echo_serial::Num::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;
	
	
	// Transform to be broadcasted to ultrasound_sensor 
	tf::Transform transform;
	tf::Quaternion q;

	// bool mid_plane = false;

	double element_w = 0.45; // mm
	double AE_SRate = 80*pow(10, 6); // hz
	double SOS = 1480; // m/s
	bool broadcast = true;
	// Linear ultrasound probe
	// Note: x and y are flipped so that the reference frame of the probe
	// and the robot based will be parallel to each while at working status

	// Initialize x, y, and z
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;

	try {
		if (g_mid_plane == true) {
			x = 0.0; // Assume that the segmented point falls within the mid-plane
		}
		else {
			// Gaussian fitting paramters a, b and c; Currently calculated from the gaussianFitting.m file
			// Note: b is the mean of the Gaussian and ideally, it should be 0
			// A Gaussian re-fitting might be needed to obtain more accurate parameter 
			double a = 39.6629;
			double b = -0.0277;
			double c = 3.1947;

			// Note: This inverse Gaussian solution assumes that the positive solution is taken, which will
			// result in the robot moving in a single direction 
			x = (sqrt(-pow(c,2)*log(msg->tc/a)) + b)/1000; // m
		}
		y = ( msg->l_ta - 64.5)*element_w/1000; // Unit:m
		z = -(msg->dly)*(1/AE_SRate)*SOS; // Unit:m


		if ( !isnan(x) ) {

		}
		else { throw false; }
	}
	catch (bool fail) { 
		std::cout << "Rostopic /active_echo_data is not published, or tc is nonpositive." << std::endl;
		broadcast = false;
	}

	if (broadcast == true) {

		transform.setOrigin( tf::Vector3(x, y, z));
		q.setRPY(0, 0, 0);
		transform. setRotation(q);

		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"ultrasound_sensor", "segment_point"));

	}
	else {	std::cout << "Broadcast to /segment_point failed "; }

	// Test whehter dynamic reconfigure changes the value of mid_plane
	std::cout << "Value of mid_plane is " << std::boolalpha << g_mid_plane << std::endl;
	// Spherical ultrasound probe
	//    int offset = -290;

	//    double angle = ( (msg->l_ta - 129/2)/129)*80;
	//    double radius = 1000*(msg->dly + offset)*(1/(AE_SRate))*SOS;
	//    x = radius*sind(angle); // sind is a triangular function
	//    z = radius*cosd(angle); // cosd is a triangular function

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "segment_image");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("active_echo_data", 100, segmentCallback);
	
	dynamic_reconfigure::Server<dynamic_reconfig::ultrasound_ur5Config> server;
	dynamic_reconfigure::Server<dynamic_reconfig::ultrasound_ur5Config>::CallbackType f;

	f = boost::bind(&dynamiconfigCallback, _1, _2);
    server.setCallback(f);

	ros::spin();

	return 0;
}

