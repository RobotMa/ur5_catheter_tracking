#include <Eigen/Core>
#include <me530646_lab1/lab1.h>
#include <ur5/utilities.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>

#define PI 3.1415926

int main(int argc, char **argv){
	//Initializing ROS
    ros::init(argc, argv, "lab1_main");
    ros::NodeHandle node;
    //Creating an RvizPlotter
	RvizPlotter p = RvizPlotter(node);
	
	/*****Examples of how to use necessary functions*****/
	//Creates an 4x4 identity matrix
	Eigen::Matrix4f T0 = Eigen::MatrixXf::Identity(4,4);
	//Plot the identity matrix in rviz
	p.plotf(T0,"Frame");
	//Print out the matrix
	printEigen(T0);
	//Plot a vector
	p.plotv("map", Eigen::Vector3f(0,0,0), Eigen::Vector3f(2,4,1));
	/****************************************************/

	Eigen::Matrix4f T10 = xf(1, 1, 0, 0, 0, PI/2);
	printEigen(T10);
	p.plotf(T10,"Frame","Frame1");

	Eigen::Matrix4f T21 = xf(-1, 1, 0, PI/2, PI/2, PI/2);
	printEigen(T21);
	p.plotf(T21,"Frame","Frame2");

	Eigen::Matrix3f rpyR = T21.block<3,3>(0,0);
	Eigen::Vector3f rpy = rpyrinv(rpyR);
	printEigen(rpy);

	Eigen::Matrix3f R = rpyr(rpy(0), rpy(1), rpy(2));
	printEigen(R);
	Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4,4);
	T.block<3,3>(0,0) = R;
	p.plotf(T,"Frame","Test");

	XYZFixedPlot(p, PI/3, PI/4, PI/3);
	sleep(2);
	ZYXRelativePlot(p, PI/3, PI/4, PI/3);

}