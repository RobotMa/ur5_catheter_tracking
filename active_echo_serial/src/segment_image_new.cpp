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
static int step_scaler = 1;
static int count = 0;
static bool publish = false; // Flag to enable publisher when receiving the subscribed topic

double x = 0.0;
double y = 0.0;
double z = 0.0;

void dynamiconfigCallback( dynamic_reconfig::segment_imageConfig &config, uint32_t level)
{
	g_mid_plane = config.In_Plane_Assumption;
	move_forward = config.Move_Forward;
	step = config.Step_Length;
	step_scaler = config.Step_Scaler; 
	ROS_INFO("Reconfigure Request: %s %s %i %i", 
			config.In_Plane_Assumption?"True":"False",
			config.Move_Forward?"True":"False",
			config.Step_Length,
			config.Step_Scaler);
}


void segmentCallback( const active_echo_serial::Num::ConstPtr& msg)
{
	static tf::TransformBroadcaster br;


	// Transform to be broadcasted to ultrasound_sensor 
	tf::Transform transform;
	tf::Quaternion q;

	double element_w = 0.45; // mm small probe : 3 mm & large probe 4.5 mm
	double AE_SRate = 80*pow(10, 6); // hz
	double SOS = 1480; // m/s
	bool broadcast = true;

	// Linear ultrasound probe
	// Note: x and y are flipped so that the reference frame of the probe
	// and the robot based will be parallel to each while at working status


	// Broadcast the x of the segmented point ahead of the ultrasound mid-plane
	// if direction = 1; behind the ultrasound mid-plane when direction = -1
	double direction = 1;
	const int tc_upper_bound = 38; // counter threshold
	const int tc_lower_bound = 5;
	const int detection_range = 8; // mm

	static double l_ta_e_prev = 0.0;
	static double l_ta_e_curr = 0.0;

	static double time_prev = ros::Time::now().toSec();
	static double time_curr = ros::Time::now().toSec();
	double time_interval = 0.0;

	try {

		
		std::cout << "l_ta_e_curr is: " << l_ta_e_curr << std::endl;

		time_curr = ros::Time::now().toSec();
		time_interval = time_curr - time_prev;
		double l_ta_e_change = l_ta_e_curr - l_ta_e_prev;
		l_ta_e_prev = l_ta_e_curr;

		std::cout << "time_curr is : " << time_curr << std::endl;
		std::cout << "time_prev is : " << time_prev << std::endl;
		std::cout << "l_ta_e_curr - l_ta_e_prev is: " << l_ta_e_change << std::endl;
		std::cout << "time_interval is: " << time_interval << std::endl;

		l_ta_e_curr = ((double)msg->l_ta - 64.5)/64.5;
		std::cout << "l_ta_e_curr is: " << l_ta_e_curr << std::endl;

		if (l_ta_e_curr > 0.30){
			y = 2*ceil(msg->l_ta - 64.5)*element_w/1000; // Unit:m
		}
		else{
			y = ceil(msg->l_ta - 64.5)*element_w/1000 + 0.0*(l_ta_e_curr - l_ta_e_prev)/time_interval; // Unit:m
		}
		time_prev = time_curr;


		z = -(msg->dly)*(1/AE_SRate)*SOS; // Unit:m
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
			double scale = 1.0;

			// Change the relative direction of the segmented point w.r.t. the mid-plane
			// This will result in the change of moving direction of the robot arm 
			if ( move_forward == false ) {  direction = -1;  }
			if ( msg->tc > tc_lower_bound && msg->tc < tc_upper_bound ) {
				x = (sqrt(-pow(c,2)*log(msg->tc/a)) + b)/1000/scale; // m
			}
			else if ( msg->tc > 0 && msg->tc < tc_lower_bound ) {
				x = (direction*(tc_upper_bound - msg->tc)/(double)tc_upper_bound*detection_range/2)/(step_scaler*1000);
				// x = (tc_lower_bound - msg->tc)/(double)tc_lower_bound*(sqrt(-pow(c,2)*log(msg->tc/a)) + b)/1000/scale; // m

				std::cout << "when tc is small, x will be " << x << std::endl; 
			}

			else { x = 0.0; } // m 

		}

		if ( isnan(x) ) {std::cout << "Prepare to throw" << std::endl; throw false;}

	}
	catch (bool fail) { 
		std::cout << "Rostopic /active_echo_data is not published, or tc is nonpositive." << std::endl;
		broadcast = false;
	}


	// Filter out outliers of /segment_point
	if ( abs(msg->dly) < 2600 && msg->tc > 0  && broadcast == true ) {

		transform.setOrigin( tf::Vector3(x, y, z));
		std::cout << "Value of x is " << x << std::endl;
		std::cout << "Value of y is " << y << std::endl;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		std::cout << "Preparing to broadcast the transform" << std::endl;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"ultrasound_sensor", "segment_point"));

	}
	else if ( abs(msg->dly) > 3000 && msg->tc > 30 && broadcast == true ) {
		transform.setOrigin (tf::Vector3(0.001,y,z));
		q.setRPY(0,0,0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ultrasound_sensor", "segment_point"));
	}
	else {std::cout << "Broadcast to /segment_point failed "; }

	// Test whehter dynamic reconfigure changes the value of mid_plane
	std::cout << "Value of In_Plane_Assumption is " << std::boolalpha << g_mid_plane  << std::endl;
	std::cout << "Value of Move_Forward is "        << std::boolalpha << move_forward << std::endl;
	std::cout << count << std::endl;
	count++;

	publish = true;
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

	int rate = 10*step_scaler;
	ros::Rate r(rate); // Hz

	ros::Subscriber sub = n.subscribe("active_echo_data", 5, segmentCallback);

	// publish the pose of the active echo element within the ultrasound sensor frame
	ros::Publisher pub = n.advertise<geometry_msgs::Pose>("active_echo_pose", 1);

	geometry_msgs::Pose ae_pose; 

	dynamic_reconfigure::Server<dynamic_reconfig::segment_imageConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfig::segment_imageConfig>::CallbackType f;

	f = boost::bind(&dynamiconfigCallback, _1, _2);
	server.setCallback(f);

	while (ros::ok()){

		if (publish == true){
			ae_pose.position.x = x;
			ae_pose.position.y = y;
			ae_pose.position.z = z;
			ae_pose.orientation.x = 0;
			ae_pose.orientation.y = 0;
			ae_pose.orientation.z = 0;
			ae_pose.orientation.w = 1;

			pub.publish(ae_pose);
			publish = false;
		}
		ros::spinOnce();
		r.sleep();
	} 

	return 0;
}
