#ifndef LAB2_H
#define LAB2_H

#include <Eigen/Dense>

/**
 * Returns the 3x3 skew-symmetric matrix that corresponds to vector e.
 */
Eigen::Matrix3f skew3(Eigen::Vector3f e);

/**
 * Returns the 3x3 rotation matrix that corresponds to a rotation of 
 * |e| radians about vector e.
 */
Eigen::Matrix3f expr(Eigen::Vector3f e);

/**
 * Returns the (x,y,z) translation and (roll,pitch,yaw) rotations
 * of the given homogeneous transformation.
 */
Eigen::VectorXf xfinv(Eigen::Matrix4f xf);

#endif