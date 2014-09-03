#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ur5/ur5_joints.h>
#include <tf/transform_listener.h>

ros::Publisher pb;
sensor_msgs::JointState j;

void forwardKinematicsCallback(const ur5::ur5_joints& joints)
{
	j.position.clear();
	j.position.push_back(joints.q[0]);
	j.position.push_back(joints.q[1]);
	j.position.push_back(joints.q[2]);
	j.position.push_back(joints.q[3]);
	j.position.push_back(joints.q[4]);
	j.position.push_back(joints.q[5]);

	pb.publish(j);
}

// void InverseKinematicsCallback(const std_msgs::Bool& b)
// {

// }

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rviz_joint_state_publisher");
	ros::NodeHandle n;

	pb = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
	ros::Subscriber forwardSub = n.subscribe("forwardKinematics", 10, &forwardKinematicsCallback);

	ros::Rate rate(100);

	j.header.stamp = ros::Time::now();
	j.header.seq = 1;

	j.name.push_back("shoulder_pan_joint");
	j.name.push_back("shoulder_lift_joint");
	j.name.push_back("elbow_joint");
	j.name.push_back("wrist_1_joint");
	j.name.push_back("wrist_2_joint");
	j.name.push_back("wrist_3_joint");
	j.position.push_back(0.0);
	j.position.push_back(0.0);
	j.position.push_back(0.0);
	j.position.push_back(0.0);
	j.position.push_back(0.0);
	j.position.push_back(0.0);
	pb.publish(j);

	while(ros::ok())
  	{
      	//spin
		ros::spinOnce();
		j.header.stamp = ros::Time::now();
		j.header.seq += 1;
		pb.publish(j);
		rate.sleep();
    }
}