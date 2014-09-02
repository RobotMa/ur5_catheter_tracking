#include <Eigen/Dense>
#include <ur5/utilities.h>
#include <ur5/lab1.h>
#include <ur5/lab2.h>
#include <iostream>



int main(int argc, char** argv){

	const double pi = 3.1415269;

	Eigen::Matrix4f h = xf(2,3,4,pi/2,0,pi);

	std::cout << "xf\n" << h <<"\n"<< std::endl;

	std::cout << "finv\n" << finv(h) <<"\n"<< std::endl;

	std::cout << "xfinv\n" << xfinv(h) <<"\n"<< std::endl;

	std::cout << "rollr\n" << rollr(pi/2) <<"\n"<< std::endl;
	std::cout << "pitchr\n" << pitchr(pi/2) <<"\n"<< std::endl;
	std::cout << "yawr\n" << yawr(pi/2) <<"\n"<< std::endl;

	std::cout << "rpyr\n" << rpyr(pi/2, 0, 0) <<"\n"<< std::endl;

	Eigen::Matrix3f r = rpyr(3.0, 2.1, 1.5);
	std::cout << "r\n" << r << "\n" << std::endl;
	Eigen::Vector3f rpy = rpyrinv(r);
	std::cout << "rpyrinv\n" << rpy <<"\n"<< std::endl;
	std::cout << "rpyr\n" << rpyr(rpy(0), rpy(1), rpy(2)) <<"\n"<< std::endl;

	Eigen::Vector3f e(0,0,pi/2); 
	std::cout << "skew3\n" << skew3(e) << "\n" << std::endl;

	std::cout << "expr\n" << expr(e) << "\n" << std::endl;

	plotf(h);
}