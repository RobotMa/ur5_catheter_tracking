#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfig/ActiveEchoConfig.h>

void callback(dynamic_reconfig::ActiveEchoConfig &config, uint32_t level){

	ROS_INFO("Reconfigure Request: %d %f %s %s %d ",
		  config.int_param, 
		  config.str_param,
		  config.size);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "dynamic_reconfig");

	dynamic_reconfigure::Server<dynamic_reconfig::ActiveEchoConfig> server;
	dynamic_reconfigure::Server<dynamic_reconfig::ActiveEchoConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}
