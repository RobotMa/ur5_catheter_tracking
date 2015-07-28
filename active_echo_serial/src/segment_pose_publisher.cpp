#include "ros/ros.h"
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <math.h>

// This node uses the pose of /segment_point to calculate the new desired pose
// of /ee_link 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "segment_pose_publisher");
	ros::NodeHandle n;

	ros::Publisher pub_pose = n.advertise<geometry_msgs::Pose>( "set_pose", 1);

	tf::TransformListener listener;

	while (n.ok()){


		tf::Transform transform_be;

		try{
        // Calculate the new pose of /ee_link based on the pose /segment_point
        // Look up the transformation between /ee_link and /base_link
			std::string ref_frame( "base_link" );
			std::string tge_frame( "ee_link" );
			std::string tgs_frame( "segment_point" );
			std::string tgu_frame( "ultrasound_sensor" );


			tf::StampedTransform transform_ue;
			tf::StampedTransform transform_bs;
			listener.waitForTransform( tge_frame, tgu_frame, ros::Time(0), ros::Duration(0.8) );
			listener.lookupTransform( tge_frame, tgu_frame, ros::Time(0), transform_ue );
			listener.waitForTransform( ref_frame, tgs_frame, ros::Time(0), ros::Duration(0.8)  );
			listener.lookupTransform( ref_frame, tgs_frame, ros::Time(0), transform_bs );

			transform_be.mult(transform_bs, transform_ue);


		}
		catch(tf::TransformException ex)
		{ std::cout << ex.what() << std::endl; }

		geometry_msgs::Pose des_pose;

		des_pose.position.x =  transform_be.getOrigin().x();
		des_pose.position.y =  transform_be.getOrigin().y();
		des_pose.position.z =  transform_be.getOrigin().z();

		des_pose.orientation.x = transform_be.getRotation().x();
		des_pose.orientation.y = transform_be.getRotation().y();
		des_pose.orientation.z = transform_be.getRotation().z();
		des_pose.orientation.w = transform_be.getRotation().w();


		pub_pose.publish( des_pose );	
		ros::spin();

	}


	return 0;
}

