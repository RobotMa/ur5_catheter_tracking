#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <active_echo_serial/Num.h> // Active Echo Mmessage Header File
#include <dynamic_reconfigure/server.h> // Dynamic Reconfigure Header File
#include <dynamic_reconfig/segment_imageConfig.h> // Ultra-UR5 Dynamic Reconfigure Header File

// Some unknown runtime error exists between this node and the ur5.launch in ur5_control

//This node subscribes to the ROS topic /active_echo_data and publishes
//the coordinate information of the segmented point.

static bool g_mid_plane = true;
static bool move_forward = true; 
static int step = 1;              // Step length along x-axis

void dynamiconfigCallback(dynamic_reconfig::segment_imageConfig &config, uint32_t level)
{
	g_mid_plane = config.In_Plane_Assumption;
	move_forward = config.Move_Forward;
	step = config.Step_Length; 
	ROS_INFO("Reconfigure Request: %s %s %i", 
		  config.In_Plane_Assumption?"True":"False",
		  config.Move_Forward?"True":"False",
		  config.Step_Length);
}


void segmentCallback(const active_echo_serial::Num::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;


	// Transform to be broadcasted to ultrasound_sensor 
	tf::Transform transform;
	tf::Quaternion q;

	double element_w = 0.3; // mm small probe : 3 mm & large probe 4.5 mm
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

	// Publish the x of the segmented point ahead of the ultrasound mid-plane
	// if direction = 1; behind the ultrasound mid-plane when direction = -1
	double direction = 1; 
	const int t_s = 20; // counter threshold

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
			double scale = 5.0;

			// Change the relative direction of the segmented point w.r.t. the mid-plane
			// This will result in the change of moving direction of the robot arm 
			if ( move_forward == false ) {  direction = -1;  }
			if (msg->tc < 20) {
				x = direction*(t_s - msg->tc)/5*step*0.001; // m 
				// x = direction*step*0.001; // m 
			}
			// x = (sqrt(-pow(c,2)*log(msg->tc/a)) + b)/1000/scale; // m
		}
		y = ( msg->l_ta - 64.5)*element_w/1000; // Unit:m
		z = -(msg->dly)*(1/AE_SRate)*SOS; // Unit:m


		if ( !isnan(x) ) {

		}
		else { std::cout << "Prepare to throw" << std::endl; throw false; }
	}
	catch (bool fail) { 
		std::cout << "Rostopic /active_echo_data is not published, or tc is nonpositive." << std::endl;
		broadcast = false;
	}
	
	// Filter out outliers of /segment_point
	if ( fabs(z) < 0.06 && msg->tc > 0  && broadcast == true ) {

		transform.setOrigin( tf::Vector3(x, y, z));
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
                std::cout << "Preparing to broadcast the transform" << std::endl;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"ultrasound_sensor", "segment_point"));

	}
	else {	std::cout << "Broadcast to /segment_point failed "; }

	// Test whehter dynamic reconfigure changes the value of mid_plane
	std::cout << "Value of In_Plane_Assumption is " << std::boolalpha << g_mid_plane  << std::endl;
	std::cout << "Value of Move_Forward is "        << std::boolalpha << move_forward << std::endl;

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
	
	ros::Rate r(1); // Hz

	ros::Subscriber sub = n.subscribe("active_echo_data", 5, segmentCallback);

	dynamic_reconfigure::Server<dynamic_reconfig::segment_imageConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfig::segment_imageConfig>::CallbackType f;

	f = boost::bind(&dynamiconfigCallback, _1, _2);
	server.setCallback(f);

	ros::spin();
	
	r.sleep();

	return 0;
}

