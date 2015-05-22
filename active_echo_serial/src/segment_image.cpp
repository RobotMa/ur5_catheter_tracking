#include "ros/ros.h"
#include "std_msgs/String.h"

// Include message header file
#include <active_echo_serial/Num.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

//This node subscribes to the ROS topic /active_echo_data and publishes
//the coordinate information of the segmented point.



void segmentCallback(const active_echo_serial::Num::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    double element_w = 1; // To be replaced with real value
    double AE_SRate = 0.5; // To be replaced with real value
    double SOS = 340; // To be replaced with real value

    // Linear ultrasound probe
    // Note: x and y are flipped so that the reference frame of the probe
    // and the robot based will be parallel to each while at working status
    double y = ( msg->l_ta - 64.5)*element_w; // Unit:mm
    double x = 0.0; // Assume that the segmented point falls within the plane
    double z = 1000*(msg->dly)*(1/AE_SRate)*SOS; // Unit:mm

//    geometry_msgs::Point segment_point;
//    segment_point.x = x;
//    segment_point.y = y;
//    segment_point.z = z;
    transform.setOrigin( tf::Vector3(x, y, z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"ultrasound_sensor", "segment_point"));


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

    ros::spin();

    return 0;
}

