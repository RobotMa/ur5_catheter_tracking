#include <Eigen/Core>
#include <me530646_lab1/lab1.h>
#include <ur5/utilities.h>
#include <math.h>
#include <iostream>

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the x axis (roll) of phi radians.
 */
Eigen::Matrix3f rollr(double phi){
	//TODO
}

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the y axis (pitch) of theta radians.
 */
Eigen::Matrix3f pitchr(double theta){
	//TODO		 
}

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the z axis (yaw) of psi radians.
 */
Eigen::Matrix3f yawr(double psi){
	//TODO
}

/**
 * Returns the 3x3 rotation matrix that represents successive 
 * roll, pitch, and yaw rotations.
 */
Eigen::Matrix3f rpyr(double roll, double pitch, double yaw){
	//TODO
}

/**
 * Returns the roll, pitch, and yaw from the given 3x3 rotation matrix.
 */
Eigen::Vector3f rpyrinv(Eigen::Matrix3f r){
	//TODO
}

/**
 * Returns the 4x4 homogeneous transformation that represents a 
 * translation of (x,y,z) and a rotation of (roll,pitch,yaw).
 */
Eigen::Matrix4f xf(double x, double y, double z, double roll, double pitch, double yaw){
	//TODO
}

/**
 * Returns the matrix inverse to the given homogeneous transformation.
 */
Eigen::Matrix4f finv(Eigen::Matrix4f f){
	//TODO
}

/**
 * Animates a roll-pitch-yaw rotation about the world
 * coordinate frame
 */
void XYZFixedPlot(double roll, double pitch, double yaw){
	//TODO

	//Hint:First apply the roll rotation in increments,
	//using plotf to draw the frame in rviz
}

/**
 * Animates a roll-pitch-yaw rotation about the body-fixed
 * coordinate frame
 */
void ZYXRelativePlot(double roll, double pitch, double yaw){
	//TODO
}


////////////////////Helpful Functions///////////////////
/**
 * Returns the homogenous transformation for rotation
 * about the x axis.
 */
Eigen::Matrix4f roll4(double roll)
{
	Eigen::Matrix4f r = Eigen::MatrixXf::Identity(4,4);
	r.block<3,3>(0,0) = rollr(roll);
	return r;
}

/**
 * Returns the homogenous transformation for rotation
 * about the y axis.
 */
Eigen::Matrix4f pitch4(double pitch)
{
	Eigen::Matrix4f r = Eigen::MatrixXf::Identity(4,4);
	r.block<3,3>(0,0) = pitchr(pitch);
	return r;
}

/**
 * Returns the homogenous transformation for rotation
 * about the z axis.
 */
Eigen::Matrix4f yaw4(double yaw)
{
	Eigen::Matrix4f r = Eigen::MatrixXf::Identity(4,4);
	r.block<3,3>(0,0) = yawr(yaw);
	return r;
}
