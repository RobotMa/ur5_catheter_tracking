#include <Eigen/Dense>
#include <ur5/lab1.h>
#include <math.h>
#include <iostream>

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the x axis (roll) of phi radians.
 */
Eigen::Matrix3f rollr(double phi){
	Eigen::Matrix3f r;
	r << 1,          0,         0,
		 0,	cos(phi), -sin(phi),
		 0, sin(phi),  cos(phi); 

	return r;
}

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the y axis (pitch) of theta radians.
 */
Eigen::Matrix3f pitchr(double theta){
	Eigen::Matrix3f r;
	r <<  cos(theta), 0,  sin(theta),
		           0, 1,           0,
		 -sin(theta), 0,  cos(theta); 

	return r;		 
}

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the z axis (yaw) of psi radians.
 */
Eigen::Matrix3f yawr(double psi){
	Eigen::Matrix3f r;
	r <<  cos(psi), -sin(psi), 0,
		  sin(psi),  cos(psi), 0,
		          0,        0, 1;

	return r;
}

/**
 * Returns the 3x3 rotation matrix that represents successive 
 * roll, pitch, and yaw rotations.
 */
Eigen::Matrix3f rpyr(double roll, double pitch, double yaw){
	Eigen::Matrix3f r = yawr(yaw)*pitchr(pitch)*rollr(roll);
	return r;
}

/**
 * Returns the roll, pitch, and yaw from the given 3x3 rotation matrix.
 */
Eigen::Vector3f rpyrinv(Eigen::Matrix3f r){
	Eigen::Vector3f rpy;
	rpy(2) = atan2(r(1,0),r(0,0));
	rpy(1) = atan2(-r(2,0),sqrt(pow(r(0,0),2)+pow(r(1,0),2)));
	rpy(0) = atan2(r(2,1),r(2,2));
	return rpy;
}

/**
 * Returns the 4x4 homogeneous transformation that represents a 
 * translation of (x,y,z) and a rotation of (roll,pitch,yaw).
 */
Eigen::Matrix4f xf(double x, double y, double z, double roll, double pitch, double yaw){
	Eigen::Matrix4f h;
	h.block<3,3>(0,0) = rpyr(roll, pitch, yaw);
	Eigen::Vector4f col3(x,y,z,1);
	h.col(3) = col3;
	Eigen::Vector4f row3(0,0,0,1);
	h.row(3) = row3;
	return h;
}

/**
 * Returns the matrix inverse to the given homogeneous transformation.
 */
Eigen::Matrix4f finv(Eigen::Matrix4f f){
	Eigen::Matrix3f r = f.block<3,3>(0,0);
	Eigen::Matrix3f rT = r.transpose();
	Eigen::Matrix4f hinv;
	hinv.block<3,3>(0,0) = rT;
	Eigen::Vector3f xyz(f(0,3),f(1,3),f(2,3));
	xyz = rT*xyz;
	hinv.block<3,1>(0,3) = xyz;
	Eigen::Vector4f row3(0,0,0,1);
	hinv.row(3) = row3;

	return hinv;
}



