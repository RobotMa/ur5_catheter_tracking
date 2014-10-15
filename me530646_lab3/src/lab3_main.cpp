#include <Eigen/Core>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>

int main(int argc, char **argv){
	ros::init(argc,argv,"lab3");
	ros::NodeHandle n;
	UR5 robot = UR5(n);
	double q1[] = {1,1,2,3,1,0};
	robot.plotfwd(q1);
}
