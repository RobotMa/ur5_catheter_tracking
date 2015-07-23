#ifndef LAB4_H
#define LAB4_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ur5/utilities.h>
#include <ur5_class/lab3.h>

int inverse(Eigen::Matrix4f H0_6, double **q, double q6_des = 0.0);

Eigen::MatrixXf J(double q[6]);

/**
 * Returns the 4x4 homogeneous transformation that represents a 
 * translation of (x,y,z) and a rotation of (roll,pitch,yaw).
 */
Eigen::Matrix4f xf(double x, double y, double z, double roll, double pitch, double yaw);

/**
 * Returns the 3x3 skew-symmetric matrix that corresponds to vector e.
 */
Eigen::Matrix3f skew3(Eigen::Vector3f e);

/**
 * Returns the 3x3 rotation matrix that represents successive 
 * roll, pitch, and yaw rotations.
 */
Eigen::Matrix3f rpyr(double roll, double pitch, double yaw);

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the x axis (roll) of phi radians.
 */
Eigen::Matrix3f rollr(double phi);

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the y axis (pitch) of theta radians.
 */
Eigen::Matrix3f pitchr(double theta);

/**
 * Returns the 3x3 rotation matrix that represents a rotation about 
 * the z axis (yaw) of psi radians.
 */
Eigen::Matrix3f yawr(double psi);

/**
 * Returns the matrix inverse to the given homogeneous transformation.
 */
Eigen::Matrix4f finv(Eigen::Matrix4f f);

#endif
