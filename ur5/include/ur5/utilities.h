#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>

class RvizPlotter{


	class RvizFrame{
		public:
			std::string parentName;
			std::string childName;
			tf::Transform transform;
			RvizFrame(tf::Transform transform, std::string parentName, std::string childName);
	};
	std::vector<RvizFrame> frames;
	std::vector<visualization_msgs::Marker> vectors;
	tf::TransformBroadcaster br;
	ros::Publisher pb;
	boost::thread *broadcastThread;

	void broadcast();

	public:
	RvizPlotter();
	RvizPlotter(ros::NodeHandle &n);
	~RvizPlotter();
	void plotf(Eigen::Matrix4f f, std::string parentName, std::string childName);
	void plotf(Eigen::Matrix4f f, std::string frameName);
	void plotv(std::string frameName, Eigen::Vector3f point1, Eigen::Vector3f point2);
};

void printEigen(const Eigen::MatrixXf& m);
Eigen::Matrix4f  getTransformation(std::string parentName, std::string childName);


#endif
