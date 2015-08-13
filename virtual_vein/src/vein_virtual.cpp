#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "vein_construct");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  tf::TransformListener listener;

  ros::Rate r(1);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points;
    points.header.frame_id = "/base_link";
    points.header.stamp = ros::Time::now();
    points.ns = "vein_points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Relative transform between segmented point and the /base_link of UR5
    tf::StampedTransform transform_bp;

    try{

      std::string ref_frame( "base_link");
      std::string tgt_frame( "segment_point" );

      listener.waitForTransform( ref_frame, tgt_frame, ros::Time(0), ros::Duration(1) );
      listener.lookupTransform( ref_frame, tgt_frame, ros::Time(0), transform_bp );


      geometry_msgs::Point p;
      p.x = transform_bp.getOrigin().x();
      p.y = transform_bp.getOrigin().y();
      p.z = transform_bp.getOrigin().z();

      points.points.push_back(p);

    }
    catch(tf::TransformException ex)

    // The location of this publishing action is yet to be determined in the main file
    marker_pub.publish(points);

    r.sleep();

  }
}