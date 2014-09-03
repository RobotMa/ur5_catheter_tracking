#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Bool.h>

ros::Publisher pb;
trajectory_msgs::JointTrajectory j;


void goCallback(const std_msgs::Bool& b)
{
	pb.publish(j);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arm_controller_commander");
	ros::NodeHandle n;

	pb = n.advertise<trajectory_msgs::JointTrajectory>("arm_controller/command", 10);
	j.header.stamp = ros::Time::now();
	j.header.seq = 1;
	j.joint_names.push_back("shoulder_pan_joint");
	j.joint_names.push_back("shoulder_lift_joint");
	j.joint_names.push_back("elbow_joint");
	j.joint_names.push_back("wrist_1_joint");
	j.joint_names.push_back("wrist_2_joint");
	j.joint_names.push_back("wrist_3_joint");

	trajectory_msgs::JointTrajectoryPoint p1;
	p1.positions.push_back(1.57075);
	p1.positions.push_back(1.57075);
	p1.positions.push_back(1.57075);
	p1.positions.push_back(1.57075);
	p1.positions.push_back(1.57075);
	p1.positions.push_back(1.57075);

	p1.velocities.push_back(.25);
	p1.velocities.push_back(.25);
	p1.velocities.push_back(.25);
	p1.velocities.push_back(.25);
	p1.velocities.push_back(.25);
	p1.velocities.push_back(.25);

	p1.time_from_start = ros::Duration(20);

	trajectory_msgs::JointTrajectoryPoint p2;
	p2.positions.push_back(-1.57075);
	p2.positions.push_back(-1.57075);
	p2.positions.push_back(-1.57075);
	p2.positions.push_back(-1.57075);
	p2.positions.push_back(-1.57075);
	p2.positions.push_back(-1.57075);

	p2.velocities.push_back(-.25);
	p2.velocities.push_back(-.25);
	p2.velocities.push_back(-.25);
	p2.velocities.push_back(-.25);
	p2.velocities.push_back(-.25);
	p2.velocities.push_back(-.25);

	p2.time_from_start = ros::Duration(40);

	j.points.push_back(p1);
	j.points.push_back(p2);
	ros::Subscriber sb = n.subscribe("go", 10, &goCallback);

	while (n.ok()){
    ros::spinOnce();
  }
}