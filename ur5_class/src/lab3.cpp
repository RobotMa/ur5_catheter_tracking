#include <Eigen/Core>
#include <ros/ros.h>
#include <ur5/utilities.h>
#include <ur5/ur5_joints.h>
#include <ur5/getSimPos.h>
#include <ur5/getRealPos.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <rviz_plot/lab1.h>
#include <rviz_animate/lab2.h>
#include <ur5_class/lab3.h>

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
 	move_pb = n.advertise<sensor_msgs::JointState>("ur5_joint_state", 10);
	hand_pb = n.advertise<std_msgs::Float64>("tilt_controller/command",10);
	simPos_client = n.serviceClient<ur5::getSimPos>("getSimPos");
	realPos_client = n.serviceClient<ur5::getRealPos>("getRealPos");
	link_names.push_back("shoulder");
	link_names.push_back("upper_arm");
	link_names.push_back("forearm");
	link_names.push_back("wrist_1");
	link_names.push_back("wrist_2");
	link_names.push_back("wrist_3");
	link_names.push_back("end-effector");
	usleep(2000000);
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
	               0,	    0, 0, 1;

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
 * Moves the UR5 hardware by defining desired joint angles.
 */
 void UR5::movefwd(double q[]){
 	sensor_msgs::JointState msg;
 	for(int i=0;i<6;i++)
 	{
 		msg.position.push_back(q[i]);
 	}
 	move_pb.publish(msg);
}

/**
 * Gets position of robot.
 */
void UR5::getPos(double *q)
{
	ur5::getSimPos simsrv;
	ur5::getRealPos realsrv;
	if(simPos_client.call(simsrv))
	{
		for (int i = 0; i < 6; ++i)
		{
			q[i] = simsrv.response.pos[i];
			if(i==0)
			{
				printf("Adding PI\n");
				q[0] += PI;
			}
		}
	}
	else if(realPos_client.call(realsrv)){
		for (int i = 0; i < 6; ++i)
		{
			q[i] = realsrv.response.pos[i];
		}
	}
	else
		ROS_ERROR("No position services available");
}

/**
 * Close the UR5 gripper.
 */
void UR5::openHand()
{
	std_msgs::Float64 openMsg;
	openMsg.data = 1;
	hand_pb.publish(openMsg);
}

/**
 * Open the UR5 gripper.
 */
void UR5::closeHand()
{
	std_msgs::Float64 closeMsg;
	closeMsg.data = 1;
	hand_pb.publish(closeMsg);
}

/**
 * Plots the coordinate frames (according to D-H convention) for the
 * UR5 given a set of joint angles. Each frame is plotted relative to
 * fixed frame base_link. Use the strings stored in link_names for 
 * names of each frame 1-6.
 */	
 void UR5::plotframes(double q[]){
	Eigen::Matrix4f H_total = Eigen::MatrixXf::Identity(4,4);
	for(int i = 0; i < 6; i++)
	{
		H_total = H_total*dhf(UR5::alpha[i],UR5::a[i],UR5::d[i],q[i]);
		this->rvizPlotter.plotf(H_total,"base_link",link_names[i]);
	}
}
