#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <rviz_plot/lab1.h>
#include <Eigen/Eigenvalues>

/**
 * Returns the 3x3 skew-symmetric matrix that corresponds to vector e.
 */
Eigen::Matrix3f skew3(Eigen::Vector3f e){
	Eigen::Matrix3f E;
	E <<     0, -e(2),    e(1),
		  e(2),     0,   -e(0),
		 -e(1),  e(0),       0;
	return E;
}

/**
 * Returns the 3x3 rotation matrix that corresponds to a rotation of
 * |e| radians about vector e.
 */
Eigen::Matrix3f expr(Eigen::Vector3f e){
	// Eigen::Matrix3f E = skew3(e);
	// Eigen::Matrix3f R = exp(E);
	double theta = e.norm();
	Eigen::Matrix3f E = skew3(e);
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3,3);
	Eigen::Matrix3f R;
	R = I + sin(theta)*E + (1 - cos(theta))*E*E;
	return R;
}

/**
 * Returns the (x,y,z) translation and (roll,pitch,yaw) rotations
 * of the given homogeneous transformation.
 */
Eigen::VectorXf xfinv(Eigen::Matrix4f xf){
	Eigen::VectorXf xyzrpy(6);
	for (int i = 0; i < 3; ++i)
	{
		xyzrpy(i) = xf(i,3);
	}
	Eigen::Matrix3f R = xf.block<3,3>(0,0);
	Eigen::Vector3f rpy = rpyrinv( R );
	for (int i = 3; i < 6; ++i)
	{
		xyzrpy(i) = rpy(i - 3);

	}
	return xyzrpy;
}


