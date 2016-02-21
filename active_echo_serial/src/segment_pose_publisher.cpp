#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

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

// Dynamic reconfigure to enable/disable tracking mode
void dynamiconfigCallback(dynamic_reconfig::segment_pose_publisherConfig &config, uint32_t level)
{
	g_track = config.Enable_Tracking;
	ROS_INFO("Reconfigure Request: %s", config.Enable_Tracking?"True":"False");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "segment_pose_publisher");
	ros::NodeHandle n;

	ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>( "setpose", 1);

	tf::TransformListener listener_1; // /ee_link w.r.t. /ultrasound_sensor
	tf::TransformListener listener_2; // /segment_point w.r.t. /base_link
	tf::TransformListener listener_3; // /ee_link w.r.t. /base_link
	tf::TransformListener listener_4;
	tf::TransformListener listener_5;

	// Dynamic reconfigure callback to update g_track 
	dynamic_reconfigure::Server<dynamic_reconfig::segment_pose_publisherConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfig::segment_pose_publisherConfig>::CallbackType f;

	f = boost::bind(&dynamiconfigCallback, _1, _2);
	server.setCallback(f);

	ros::Rate r(10); // 10 Hz
			 // The publishing rate should be high enough to achieve almost real-time
			 // performance, and low enough so as not to cause jammed data for other
			 // subscribers. To be further tuned.

	while ( ros::ok() ){


		tf::Transform transform_be;
		tf::StampedTransform transform_be_old; // Current transformation between /ee_link and /base_link
		tf::StampedTransform transform_ue;
		tf::StampedTransform transform_bs;
		tf::StampedTransform transform_bu;
		tf::StampedTransform transform_su;
		tf::Transform transform_be_calc;

		ROS_INFO("segment_pose_publisher is running");

		std::cout << "Value of track is: " << std::boolalpha << g_track << std::endl;

		try{
			// Calculate the new pose of /ee_link based on /segment_point
			// Look up the transformation between /ee_link and /base_link
			std::string ref_frame( "base_link" );
			std::string tge_frame( "ee_link" );
			std::string tgs_frame( "segment_point" );
			std::string tgu_frame( "ultrasound_sensor" );

			listener_1.waitForTransform( tgu_frame, tge_frame, ros::Time(0), ros::Duration(0.1) );
			listener_1.lookupTransform( tgu_frame, tge_frame, ros::Time(0), transform_ue );

			// listener_2.waitForTransform( ref_frame, tgs_frame, ros::Time(0), ros::Duration(0.8)  );
			listener_2.lookupTransform( ref_frame, tgs_frame, ros::Time(0), transform_bs );
			listener_3.lookupTransform( ref_frame, tge_frame, ros::Time(0), transform_be_old );
			listener_4.lookupTransform( ref_frame, tgu_frame, ros::Time(0), transform_bu);
			listener_5.lookupTransform( tgs_frame, tgu_frame, ros::Time(0), transform_su);

			// Calculate the new pose for the end-effector
			tf::Vector3 v_su = transform_su.getOrigin();

			// std::cout << "x of bu is " <<  transform_bu.getOrigin().x() << std::endl;

			v_su.setX( 0 );
			v_su.setY( 0 );
			transform_su.setOrigin( v_su );

			std::cout << "updated x of bu is" << transform_bu.getOrigin().x() << std::endl;
			transform_be_calc.mult(transform_bs, transform_su);
			transform_be.mult(transform_be_calc, transform_ue);
			// transform_be_calc.mult(transform_be, transform_ue);
			
			// transform_be.mult(transform_be, transform_ue);
			/*
			tf::Pose H1, H2, H3, H4, H5;

			H1.setOrigin(   transform_bu.getOrigin() );
                        H1.setRotation( transform_bu.getRotation() );

			Eigen::Affine3d HA_1;
                        tf::poseTFToEigen( H1, HA_1 );
                        Eigen::Matrix4d Hd_1 = HA_1.matrix();
                        Eigen::MatrixXf Hf_1 = Hd_1.cast <float> ();

			std::cout << " Pose of bu is" << std::endl;
			std::cout << Hf_1 << std::endl;
			*/

			if (g_track == true) {

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
			else { std::cout << "Not able to publish to /setpose" << std::endl; }


		}
		catch(tf::TransformException ex)
		{ std::cout << ex.what() << std::endl; 
			// g_track = false;	
		}


		ros::spinOnce();

		r.sleep();

	}


	return 0;
}

