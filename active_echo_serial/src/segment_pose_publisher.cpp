#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfig/segment_pose_publisherConfig.h>

// This node uses the pose of /segment_point to calculate the new desired pose
// of /ee_link 

// Track the segmented point
static bool g_track = false;

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


	// Dynamic reconfigure callback 
	dynamic_reconfigure::Server<dynamic_reconfig::segment_pose_publisherConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfig::segment_pose_publisherConfig>::CallbackType f;

	f = boost::bind(&dynamiconfigCallback, _1, _2);
	server.setCallback(f);

	ros::Rate r(10); // 10 Hz
	// bool pub = true; // Publish to setpose if a valid des_pose is obtained 

	while ( ros::ok() ){


		tf::Transform transform_be;
		tf::StampedTransform transform_be_old; // Current transformation between /ee_link and /base_link
		tf::StampedTransform transform_ue;
		tf::StampedTransform transform_bs;

		ROS_INFO("segment_pose_publisher is running");

		std::cout << "Value of track is: " << std::boolalpha << g_track << std::endl;

		try{
			// Calculate the new pose of /ee_link based on /segment_point
			// Look up the transformation between /ee_link and /base_link
			std::string ref_frame( "base_link" );
			std::string tge_frame( "ee_link" );
			std::string tgs_frame( "segment_point" );
			std::string tgu_frame( "ultrasound_sensor" );

			listener_1.waitForTransform( tgu_frame, tge_frame, ros::Time(0), ros::Duration(0.8) );
			listener_1.lookupTransform( tgu_frame, tge_frame, ros::Time(0), transform_ue );

			// debug nan x, y, z of des_pose
			// std::cout << transform_ue.getOrigin().x() << std::endl;
			// std::cout << transform_ue.getOrigin().y() << std::endl;
			// std::cout << transform_ue.getOrigin().z() << std::endl;

			listener_2.waitForTransform( ref_frame, tgs_frame, ros::Time(0), ros::Duration(0.8)  );
			listener_2.lookupTransform( ref_frame, tgs_frame, ros::Time(0), transform_bs );

			// debug nan x, y, z of des_pose
			// std::cout << transform_bs.getOrigin().x() << std::endl;
			// std::cout << transform_bs.getOrigin().y() << std::endl;	
			// std::cout << transform_bs.getOrigin().z() << std::endl;

			// Calculate the new pose for the end-effector
			transform_be.mult(transform_bs, transform_ue);

			listener_3.waitForTransform( ref_frame, tge_frame, ros::Time(0), ros::Duration(0.8)  );
			listener_3.lookupTransform( ref_frame, tge_frame, ros::Time(0), transform_be_old );

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

