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
	Eigen::Matrix4f h = Eigen::MatrixXf::Identity(4,4);
	//Plot the identity matrix in rviz
	p.plotf(h,"Frame");
	//Print out the matrix
	printEigen(h);
	//Plot a vector
	p.plotv("map", Eigen::Vector3f(0,0,0), Eigen::Vector3f(2,4,1));
	/****************************************************/
}