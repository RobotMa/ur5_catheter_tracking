#include <Eigen/Core>
#include <ros/ros.h>
#include <ur5/utilities.h>
#include <ur5/ur5_joints.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>

#define PI 3.1415926

//An array of the six alpha D-H parameters for the UR5
double UR5::alpha[] = {PI/2,0,0,PI/2,-PI/2,0};
//An array of the six a D-H parameters for the UR5
double UR5::a[] = {0,-0.425,-0.39225,0,0,0};
//An array of the six d D-H parameters for the UR5
double UR5::d[] = {0.089159,0,0,0.10915,0.09465,0.0823};


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
	
	//Creating the matrix which describes the rotation of theta about Z_i-1
	Eigen::Matrix4f Rz;
	Rz << cos(theta), -sin(theta), 0, 0, 
	      sin(theta),  cos(theta), 0, 0, 
	               0,   	    0, 1, 0, 
	               0,		    0, 0, 1;

	//Creating the matrix which describes the translation of d along Z_i-1
	Eigen::Matrix4f Tz = Eigen::MatrixXf::Identity(4,4);
	Tz.col(3) = Eigen::Vector4f(0,0,d,1); 

	//Creating the matrix which describes the translation of a along X_i
	Eigen::Matrix4f Tx = Eigen::MatrixXf::Identity(4,4);
	Tx.col(3) = Eigen::Vector4f(a,0,0,1); 

	//Creating the matrix which describes the rotation of alpha about X_i
	Eigen::Matrix4f Rx;
	Rx << 1,          0,          0, 0, 
		  0, cos(alpha), -sin(alpha), 0, 
		  0, sin(alpha), cos(alpha), 0, 
		  0,		  0,          0, 1;

	//The matrix composed of the sucessive rotations and translations
	Eigen::Matrix4f h = Rz*Tz*Tx*Rx;
	return h;
}

/**
 * Returns the 4x4 transformation from the base of the
 * UR5 to its gripper given a 6x1 vector of the joint
 * angles.
 */
Eigen::Matrix4f UR5::fwd(double q[]){

	Eigen::Matrix4f H_0_6 = Eigen::MatrixXf::Identity(4,4);
	
	for(int i = 0; i < 6; i++)
	{
		H_0_6 = H_0_6*dhf(alpha[i], a[i], d[i], q[i]);  
	}
	return H_0_6;
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
	Eigen::Matrix4f H_total = Eigen::MatrixXf::Identity(4,4);
	for(int i = 0; i < 6; i++)
	{
		H_total = H_total*dhf(UR5::alpha[i],UR5::a[i],UR5::d[i],q[i]);
		this->rvizPlotter.plotf(H_total,"base_link",link_names[i]);
	}
}


