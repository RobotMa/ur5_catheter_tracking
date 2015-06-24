#ifndef LAB3_H
#define LAB3_H

#include <Eigen/Core>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class UR5 {
	//Creating a publisher for the forward kinematics
	ros::Publisher fwd_pb;
	//An array holding the names of the links
	std::vector<std::string> link_names;
	//Plotter for rviz
	//	RvizPlotter rvizPlotter;
	//Creating a publisher for the forward kinematics
	ros::Publisher move_pb;
	//Creating publisher for hand control
	ros::Publisher hand_pb;
	//Creating service client for rviz jointstate 
	ros::ServiceClient simPos_client;
	//Creating subscriber for hardware jointstate  
	ros::ServiceClient realPos_client;




	public:
	//An array of the six alpha D-H parameters for the UR5
	static double alpha[];
	//An array of the six a D-H parameters for the UR5
	static double a[];
	//An array of the six d D-H parameters for the UR5
	static double d[];
	//An array of the six joint angles of the UR5
	double qCurrent[];
	static double off[];

	/**
	 * Creates an instance of the class to allow plotting 
	 * of the UR5 in rviz.
	 */
	UR5(ros::NodeHandle &n);
	/**
	 * Returns the 4x4 transformation from the base of the
	 * UR5 to its gripper given a 6x1 vector of the joint
	 * angles.
	 */
	static Eigen::Matrix4f fwd(double q[]);
	/**
	 * Plots the robot in rviz by defining its joint angles.
	 */
	void plotfwd(double q[]);

	/**
	 * Moves the UR5 hardware by defining desired joint angles.
	 */
	void movefwd(double q[]);

	/**
	 * Gets position of robot.
	 */
	void getPos(double *q);	

	/**
	 * Close the UR5 gripper.
	 */
	void openHand();

	/**
	 * Open the UR5 gripper.
	 */
	void closeHand();

	/**
	 * Returns the 4x4 transformation given the a 4x1 vector of
	 * Denavit-Hartenberg parameters.
	 */
	static Eigen::Matrix4f dhf(double alpha, double a, double d, double theta);
};

#endif
