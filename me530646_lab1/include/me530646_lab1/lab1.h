#ifndef LAB1_H
#define LAB1_H

#include <Eigen/Core>
#include <ur5/utilities.h>

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
 * Returns the 3x3 rotation matrix that represents successive 
 * roll, pitch, and yaw rotations.
 */
Eigen::Matrix3f rpyr(double roll, double pitch, double yaw);

/**
 * Returns the roll, pitch, and yaw of the given 3x3 rotation matrix.
 */
Eigen::Vector3f rpyrinv(Eigen::Matrix3f rpyr);

/**
 * Returns the 4x4 homogeneous transformation that represents a 
 * translation of (x,y,z) and a rotation of (roll,pitch,yaw).
 */
Eigen::Matrix4f xf(double x, double y, double z, double roll, double pitch, double yaw);

/**
 * Returns the matrix inverse to the given homogeneous transformation.
 */
Eigen::Matrix4f finv(Eigen::Matrix4f f);

/**
 * Animates a roll-pitch-yaw rotation about the world
 * coordinate frame
 */
void XYZFixedPlot(RvizPlotter &plotter, double roll, double pitch, double yaw);

/**
 * Animates a roll-pitch-yaw rotation about the body-fixed
 * coordinate frame
 */
void ZYXRelativePlot(RvizPlotter &plotter, double roll, double pitch, double yaw);

#endif