#include <Eigen/Core>
#include <ros/ros.h>
#include <ur5/utilities.h>
#include <ur5/ur5_joints.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>

#define PI 3.1415926

//An array of the six alpha D-H parameters for the UR5
double UR5::alpha[] = {0,0,0,0,0,0};
//An array of the six a D-H parameters for the UR5
double UR5::a[] = {0,0,0,0,0,0};
//An array of the six d D-H parameters for the UR5
double UR5::d[] = {0,0,0,0,0,0};


/**
 * Creates an instance of the class to allow plotting 
 * of the UR5 in rviz.
 */
UR5::UR5(ros::NodeHandle &n):rvizPlotter(n){
 	fwd_pb = n.advertise<ur5::ur5_joints>("forwardKinematics", 10);
	link_names.push_back("shoulder");
	link_names.push_back("upper_arm");
	link_names.push_back("forearm");
	link_names.push_back("wrist_1");
	link_names.push_back("wrist_2");
	link_names.push_back("wrist_3");
	link_names.push_back("end-effector");
}

/**
 * Returns the 4x4 transformation given the a 4x1 vector of
 * Denavit-Hartenberg parameters.
 */
Eigen::Matrix4f UR5::dhf(double alpha, double a, double d, double theta){

	// Transformation associated with the joint
	Eigen::Matrix4f Z = Eigen::MatrixXf::Identity(4,4);
	Z.block<3,3>(0,0) = yawr(theta);
	Z.block<3,1>(0,3) << 0, 0, d;

	// Transformation associated with the link
	Eigen::Matrix4f X = Eigen::MatrixXf::Identity(4,4);
	Z.block<3,3>(0,0) = rollr(alpha);
	Z.block<3,1>(0,3) << a, 0, 0;

	// Full D-H Transformation
	Eigen::Matrix4f T;
	T = Z*X;
	return T;

}

/**
 * Returns the 4x4 transformation from the base of the
 * UR5 to its gripper given a 6x1 vector of the joint
 * angles.
 */
Eigen::Matrix4f UR5::fwd(double q[]){
	//TODO
}

/**
 * Plots the robot in rviz by defining its joint angles.
 */
 void UR5::plotfwd(double q[]){
	ur5::ur5_joints j;

	j.q.push_back(q[0]-PI);
	for(int i = 1; i < 6; i++)
	{
	  j.q.push_back(q[i]);
	}
	while(fwd_pb.getNumSubscribers() < 1){}
	fwd_pb.publish(j);
}

/**
 * Plots the coordinate frames (according to D-H convention) for the
 * UR5 given a set of joint angles. Each frame is plotted relative to
 * fixed frame base_link.
 */	
 void UR5::plotframes(double q[]){
 	//TODO
 	//For plotting use: this->rvizPlotter.plotf(...)
}


