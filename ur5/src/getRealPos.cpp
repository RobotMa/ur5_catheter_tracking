#include <ros/ros.h>
#include <ur5/getRealPos.h>
#include <sensor_msgs/JointState.h>

std::vector<double> p;

bool getPos(ur5::getRealPos::Request &req,
			ur5::getRealPos::Response &res)
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
	ros::init(argc, argv, "getRealPos");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/ur5/state",10,&joint_statesCallback);

	ros::ServiceServer srv = n.advertiseService("getRealPos",getPos);
	ros::spin();
	return 0;
}