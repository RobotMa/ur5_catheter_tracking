#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <inverse_ur5/lab4.h>

#define PI M_PI

int main(int argc, char **argv){

	ros::init(argc,argv,"lab3");
	ros::NodeHandle n;
	UR5 robot = UR5(n);

	//Example of how to use the matrix logarithm
	Eigen::Vector3f v(0,0,PI/4);
	printf("Vector v:\n");
	printEigen(v);
	Eigen::Matrix3f R =  expr(v);
	printf("expr(v)\n");
	printEigen(R);
	Eigen::Matrix3f skew = R.log();
	printf("expr(v).log()\n");
	printEigen(skew);
	printf("\"Unskew\"  of expr(v).log():\n");
	printEigen(Eigen::Vector3f(skew(2,1),skew(0,2),skew(1,0)));

}
