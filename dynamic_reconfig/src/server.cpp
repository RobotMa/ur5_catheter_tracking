#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfig/ultrasound_ur5Config.h>

void callback(dynamic_reconfig::ultrasound_ur5Config &config, uint32_t level){

	ROS_INFO("Reconfigure Request: %d %s %d ",
		  config.int_param, 
		  config.str_param.c_str(),
		  config.size);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "dynamic_reconfig");

	dynamic_reconfigure::Server<dynamic_reconfig::ultrasound_ur5Config> server;
	dynamic_reconfigure::Server<dynamic_reconfig::ultrasound_ur5Config>::CallbackType f;
	
	// Bind data/function types. _1 and _2 are place holders. 
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ROS_INFO("Spinning node");
	ros::spin();
	return 0;
}
