#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfig/segment_pose_publisherConfig.h>

// This "active_echo_serial" node uses the pose of /segment_point to calculate the next desired pose
// of /ee_link 

// Tracking the segmented point is set to false
static bool g_track = false;
bool publish = false;
geometry_msgs::Pose ae_pose;

// Dynamic reconfigure to enable/disable tracking mode
void dynamiconfigCallback(dynamic_reconfig::segment_pose_publisherConfig &config, uint32_t level)
{
	g_track = config.Enable_Tracking;
	ROS_INFO("Reconfigure Request: %s", config.Enable_Tracking?"True":"False");
}

void activechoCallback( const geometry_msgs::Pose& activechoPose ){

	ae_pose = activechoPose;
	publish = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "segment_pose_publisher");
	ros::NodeHandle n;

	ros::Subscriber sub_ae_pose = n.subscribe("active_echo_pose", 1, activechoCallback);
	ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>( "setpose", 1);

	tf::TransformListener listener_1; // /ee_link w.r.t. /ultrasound_sensor
	tf::TransformListener listener_3; // /ee_link w.r.t. /base_link
	tf::TransformListener listener_4;
	tf::TransformListener listener_5;

	tf::TransformBroadcaster br;

	// Dynamic reconfigure callback to update g_track 
	dynamic_reconfigure::Server<dynamic_reconfig::segment_pose_publisherConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfig::segment_pose_publisherConfig>::CallbackType f;

	f = boost::bind(&dynamiconfigCallback, _1, _2);
	server.setCallback(f);

	ros::Rate r(20); // 10 Hz
	// The publishing rate should be high enough to achieve almost real-time
	// performance, and low enough so as not to cause jammed data for other
	// subscribers. To be further tuned.

	while ( ros::ok() ){


		tf::Transform transform_be;
		tf::StampedTransform transform_be_old; // Current transformation between /ee_link and /base_link
		tf::StampedTransform transform_ue;
		tf::Transform transform_bs;
		tf::StampedTransform transform_bu;
		tf::Transform transform_us;
		tf::Transform transform_su;
		tf::Transform transform_be_calc;

		ROS_INFO("segment_pose_publisher is running");

		std::cout << "Value of track is: " << std::boolalpha << g_track << std::endl;
		if( publish == true){
			try{
				// Calculate the new pose of /ee_link based on /segment_point
				// Look up the transformation between /ee_link and /base_link
				std::string ref_frame( "base_link" );
				std::string tge_frame( "ee_link" );
				std::string tgu_frame( "ultrasound_sensor" );

				listener_1.waitForTransform( tgu_frame, tge_frame, ros::Time(0), ros::Duration(0.05) );
				listener_1.lookupTransform( tgu_frame, tge_frame, ros::Time(0), transform_ue );
				listener_3.lookupTransform( ref_frame, tge_frame, ros::Time(0), transform_be_old );
				listener_4.lookupTransform( ref_frame, tgu_frame, ros::Time(0), transform_bu);

				// convert geometry_msgs::Pose to tf::Transform
				tf::Quaternion us_quaternion;
				tf::Vector3 us_vector;

				// Data conversion in ROS is sooooo dumb
				us_vector.setX(ae_pose.position.x);
				us_vector.setY(ae_pose.position.y);
				us_vector.setZ(ae_pose.position.z);
				transform_us.setOrigin(us_vector);

				us_quaternion.setY(ae_pose.orientation.y);
				us_quaternion.setZ(ae_pose.orientation.z);
				us_quaternion.setW(ae_pose.orientation.w);
				transform_us.setRotation(us_quaternion);


				// calculate transform_bs
				transform_bs.mult(transform_bu, transform_us);

				// calculate transform_su
				transform_su = transform_us.inverse();

				// Calculate the new pose for the end-effector
				tf::Vector3 v_su = transform_su.getOrigin();

				v_su.setX( 0 );
				v_su.setY( 0 );
				transform_su.setOrigin( v_su );

				transform_be_calc.mult(transform_bs, transform_su);

				transform_be.mult(transform_be_calc, transform_ue);

				br.sendTransform(tf::StampedTransform(transform_be, ros::Time::now(), "base_link", "future_end_effector"));

				if (g_track == true ) {

					// Convert tf::Transform to geometry_msgs::Pose for publishing
					geometry_msgs::Pose des_pose;

					des_pose.position.x =  transform_be.getOrigin().x();	
					des_pose.position.y =  transform_be.getOrigin().y();

					// Fix the z-axis translation of the end-effector
					// Set z of the des_pose to the current z of the end-effector
					// des_pose.position.z =  transform_be.getOrigin().z();
					des_pose.position.z =  transform_be_old.getOrigin().z();

					des_pose.orientation.x = transform_be.getRotation().x();
					des_pose.orientation.y = transform_be.getRotation().y();
					des_pose.orientation.z = transform_be.getRotation().z();
					des_pose.orientation.w = transform_be.getRotation().w();

					std::cout << "Current pose x is " << transform_be_old.getOrigin().x() << std::endl;
					std::cout << "Current pose y is " << transform_be_old.getOrigin().y() << std::endl;
					std::cout << "Current pose z is " << transform_be_old.getOrigin().z() << std::endl; 

					std::cout << "Future pose x is " << des_pose.position.x << std::endl;
					std::cout << "Future pose y is " << des_pose.position.y << std::endl;
					std::cout << "Future pose z is " << des_pose.position.z << std::endl;


					std::cout << " " << std::endl;
					std::cout << " Difference of x " << transform_be_old.getOrigin().x() - des_pose.position.x << std::endl;
					std::cout << " Difference of y " << transform_be_old.getOrigin().y() - des_pose.position.y << std::endl;
					pub_pose.publish( des_pose );
				}

				else { std::cout << "Suscribed rostopic active_echo_pose is empty" << std::endl; }

			}
			catch(tf::TransformException ex)
			{ std::cout << ex.what() << std::endl; 
			}
			publish = false;

		}
		ros::spinOnce();

		r.sleep();

	}


	return 0;
}

