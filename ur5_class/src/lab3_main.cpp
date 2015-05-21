#include <Eigen/Core>
#include <ur5/utilities.h>
#include <rviz_plot/lab1.h>
#include <rviz_animate/lab2.h>
#include <ur5_class/lab3.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"lab3");
	ros::NodeHandle n;
	UR5 robot = UR5(n);
	double q1[] = {1,1,2,3,1,0};
	robot.plotfwd(q1);
}
