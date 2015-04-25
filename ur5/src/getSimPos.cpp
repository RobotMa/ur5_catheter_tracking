#include <ros/ros.h>
#include <ur5/getSimPos.h>
#include <sensor_msgs/JointState.h>

std::vector<double> p;

bool getPos(ur5::getSimPos::Request &req,
			ur5::getSimPos::Response &res)
{
	res.pos = p;
	return true;
}

void joint_statesCallback(const sensor_msgs::JointState state)
{
	p = state.position;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "getSimPos");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("joint_states",10,&joint_statesCallback);

	ros::ServiceServer srv = n.advertiseService("getSimPos",getPos);
	ros::spin();
	return 0;
}