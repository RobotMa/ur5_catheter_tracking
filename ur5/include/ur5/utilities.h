#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

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
	RvizPlotter(const RvizPlotter& other);
	~RvizPlotter();
	RvizPlotter& operator=(const RvizPlotter& other);
	void plotf(Eigen::Matrix4f f, std::string parentName, std::string childName);
	void plotf(Eigen::Matrix4f f, std::string frameName);
	void plotv(std::string frameName, Eigen::Vector3f point1, Eigen::Vector3f point2);

};
template<typename Derived>
void printEigen(const Eigen::MatrixBase<Derived>& m){
  Eigen::IOFormat fmt(2, 0, ", ", "\n", "[", "]");
  std::cout << m.format(fmt) << "\n" << std::endl;
}

template<typename Derived>
void initEigen(const Eigen::MatrixBase<Derived>& m){
  Eigen::IOFormat fmt(Eigen::StreamPrecision, 1, ", ", ", ", "", "", " << ", ";");
  std::cout << m.format(fmt) << "\n" << std::endl;
}

Eigen::Matrix4f  getTransformation(std::string parentName, std::string childName);

template<typename Derived>
bool matrixEquals(const Eigen::MatrixBase<Derived>& m1, const Eigen::MatrixBase<Derived>& m2)
{
	if(m1.rows() != m2.rows() || m1.cols() != m2.cols())
	{
		printf("Matrices are not the same size!\n");
		return 0;
	}

	for(int i = 0; i < m1.rows(); i++)
	{
		for(int j = 0; j < m1.cols(); j++)
		{
			if(fabs(m1(i,j) - m2(i,j)) > .01)
			{
				printf("The (%d,%d) elements are not equal\n",i,j);
				return 0;
			}
		}
	}
	return 1;
}

#endif
