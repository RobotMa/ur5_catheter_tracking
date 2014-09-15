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

 	Eigen::Matrix3f R;
 	R << 1,        0,         0,
 	     0, cos(phi), -sin(phi),
 	     0, sin(phi),  cos(phi);
 	return R;
 }

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the y axis (pitch) of theta radians.
 */
 Eigen::Matrix3f pitchr(double theta){

 	Eigen::Matrix3f R;
 	R << cos(theta),  0,  sin(theta),
 	              0,  1,           0,
 	    -sin(theta),  0,  cos(theta);
 	return R;		 
 }

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the z axis (yaw) of psi radians.
 */
 Eigen::Matrix3f yawr(double psi){

 	Eigen::Matrix3f R;
 	R << cos(psi),  -sin(psi),  0,
 	     sin(psi),   cos(psi),  0,
 	            0,          0,  1;
 	return R;
 }

/**
 * returns the 3x3 rotation matrix that represents successive 
 * roll, pitch, and yaw rotations. (Should be yaw->pitch->roll)
 */
Eigen::Matrix3f rpyr(double roll, double pitch, double yaw){
	
	Eigen::Matrix3f R;
	R = yawr(yaw)*pitchr(pitch)*rollr(roll);
	return R;
}

/**
 * Returns the roll, pitch, and yaw from the given 3x3 rotation matrix.
 */
Eigen::Vector3f rpyrinv(Eigen::Matrix3f R){
	
	Eigen::Vector3f v;
	v(0) = atan2(  R(2,1), R(2,2) );
	v(1) = atan2( -R(2,0), sqrt( pow(R(2,1),2) + pow(R(2,2),2)) );
	v(2) = atan2(  R(1,0), R(0,0) );
	return v;
}

/**
 * Returns the 4x4 homogeneous transformation that represents a 
 * translation of (x,y,z) and a rotation of (roll,pitch,yaw).
 */
Eigen::Matrix4f xf(double x, double y, double z, double roll, double pitch, double yaw){
	
	Eigen::Matrix4f T = Eigen::Matrix4f::Identity(4,4);
	T.block<3,3>(0,0) = rpyr(roll, pitch, yaw);
	T.block<3,1>(0,3) << x, y, z;
	return T; 
}

/**
 * Returns the matrix inverse to the given homogeneous transformation.
 */
Eigen::Matrix4f finv(Eigen::Matrix4f T){
	
	Eigen::Matrix4f invT = Eigen::Matrix4f::Identity(4,4);
	Eigen::Matrix3f R = T.block<3,3>(0,0);
	Eigen::Vector3f t = T.block<3,1>(0,2);
	Eigen::Matrix3f R_t = R.transpose();
	invT.block<3,3>(0,0) = R_t;
	invT.block<3,1>(0,3) = -R_t*t;
	return invT;
}

/**
 * Animates a roll-pitch-yaw rotation about the world
 * coordinate frame
 */
 void XYZFixedPlot(RvizPlotter& p, double roll, double pitch, double yaw){

 	const int n = 50;
 	Eigen::Matrix4f T_fixed = Eigen::Matrix4f::Identity(4,4);
 	for (int i = 0; i < n+1; ++i)
 	{
 		T_fixed.block<3,3>(0,0) = rollr(roll/n*i);
 		p.plotf(T_fixed, "Frame");
 	}

 	for (int j = 0; j < n+1; ++j)
 	{
 		T_fixed.block<3,3>(0,0) = pitchr(pitch/n*j)*rollr(roll);
 		p.plotf(T_fixed, "Frame");
 	}

 	for (int k = 0; k < n+1; ++k)
 	{
 		T_fixed.block<3,3>(0,0) = yawr(yaw/n*k)*pitchr(pitch)*rollr(roll);
 		p.plotf(T_fixed, "Frame");
 	}
	//Hint:First apply the roll rotation in increments,
	//using plotf to draw the frame in rviz
}

/**
 * Animates a roll-pitch-yaw rotation about the body-fixed
 * coordinate frame
 */
void ZYXRelativePlot(RvizPlotter& p, double roll, double pitch, double yaw){

	const int n = 50;
	Eigen::Matrix4f T_fixed = Eigen::Matrix4f::Identity(4,4);
	for (int i = 0; i < n+1; ++i)
	{
		T_fixed.block<3,3>(0,0) = yawr(yaw/n*i);
		p.plotf(T_fixed, "Frame");
	}

	for (int j = 0; j < n+1; ++j)
	{
		T_fixed.block<3,3>(0,0) = yawr(yaw)*pitchr(pitch/n*j);
		p.plotf(T_fixed, "Frame");
	}

	for (int k = 0; k < n+1; ++k)
	{
		T_fixed.block<3,3>(0,0) = yawr(yaw)*pitchr(pitch)*rollr(roll/n*k);
		p.plotf(T_fixed, "Frame");
	}
}


////////////////////Helpful Functions///////////////////
/**
 * Returns the homogenous transformation for rotation
 * about the x axis.
 */
Eigen::Matrix4f roll4(double roll)
{
	Eigen::Matrix4f T = Eigen::MatrixXf::Identity(4,4);
	T.block<3,3>(0,0) = rollr(roll);
	return T;
}

/**
 * Returns the homogenous transformation for rotation
 * about the y axis.
 */
Eigen::Matrix4f pitch4(double pitch)
{
	Eigen::Matrix4f T = Eigen::MatrixXf::Identity(4,4);
	T.block<3,3>(0,0) = pitchr(pitch);
	return T;
}

/**
 * Returns the homogenous transformation for rotation
 * about the z axis.
 */
Eigen::Matrix4f yaw4(double yaw)
{
	Eigen::Matrix4f T = Eigen::MatrixXf::Identity(4,4);
	T.block<3,3>(0,0) = yawr(yaw);
	return T;
}
