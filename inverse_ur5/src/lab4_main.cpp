#include <Eigen/Core>
#include <ur5/utilities.h>
#include <rviz_plot/lab1.h>
#include <rviz_animate/lab2.h>
#include <ur5_class/lab3.h>
#include <inverse_ur5/lab4.h>
#include <ur_kinematics/ur_kin.h>
#include <iostream>

int main(int argc, char **argv){
	ros::init(argc,argv,"lab3");
	ros::NodeHandle n;
	UR5 robot = UR5(n);

	//How to calculate the trace of a matrix
	Eigen::MatrixXf M= Eigen::MatrixXf::Random(4,4);
	double trace = M.trace();

	//How to call inverse
	double q[6] = {0,0,0,0,0,0};
	double *q_sol[8];
	for(int i = 0; i < 8; i++)
	{
		q_sol[i] = new double[6];
	}
	int num_sol = inverse(UR5::fwd(q), q_sol);

	for (int i = 0; i < 8; ++i)
	{
		for (int j = 0; j < 6; ++j)
		{
			std::cout << q_sol[i][j] << " ";
			if ( j == 5)
			{
				std::cout << "\n";
			}
		}
	}
	std::cout << num_sol << std::endl;
  }
