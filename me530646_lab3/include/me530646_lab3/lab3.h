#ifndef LAB3_H
#define LAB3_H

#include <Eigen/Core>
#include <ros/ros.h>

class UR5 {
//Creating a publisher for the forward kinematics
ros::Publisher fwd_pb;
//An array holding the names of the links
std::vector<std::string> link_names;
//Plotter for rviz
RvizPlotter rvizPlotter;

	public:
		//An array of the six alpha D-H parameters for the UR5
		static double alpha[];
		//An array of the six a D-H parameters for the UR5
		static double a[];
		//An array of the six d D-H parameters for the UR5
		static double d[];
		
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
		 * Plots the coordinate frames (according to D-H convention) for the
		 * UR5 given a set of joint angles. Each frame is plotted relative to
		 * fixed frame base_link.
		 */	
		void plotframes(double q[]);

		/**
		 * Returns the 4x4 transformation given the a 4x1 vector of
		 * Denavit-Hartenberg parameters.
		 */
		static Eigen::Matrix4f dhf(double alpha, double a, double d, double theta);
};

#endif