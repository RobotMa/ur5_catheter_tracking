#include <Eigen/Core>
#include <ur5/utilities.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <Eigen/Eigenvalues>

int main(int argc, char **argv){
	//Initializing ROS
    ros::init(argc, argv, "lab1_main");
    ros::NodeHandle node;
    //Creating an RvizPlotter
	RvizPlotter p = RvizPlotter(node);
	
	/*****Examples of how to use necessary functions*****/
	//Calculating the norm of vector v
	Eigen::Vector3f v = Eigen::MatrixXf::Random(3,1);
	double normV = v.norm();
	Eigen::Vector3f v_bar = normV*v;
	Eigen::MatrixXf V(3,3) = skew3(v);

	// Output the result of  Problem 2
	std::cout << normV << std::endl; // (a)
	std::cout << v_bar << std::endl; // (b)
	std::cout << V     << std::endl; // (c)
	std::cout << V*v   << std::endl; // (d)

	Eigen::EigenSolver<Eigen::MatrixXf> es(V);
	//VectorXcf MatrixXcf are used because there may 
	//be complex values for eigenvalues and eigenvectors
	Eigen::VectorXcf evals = es.eigenvalues();
	Eigen::MatrixXcf evecs = es.eigenvectors();
	std::cout << "Eigenvalues of V are " << es.eigenvalues()   << std::endl; // (e1) 
	std::cout << "Eigenvectors of V are " << es.eigenvectors() << std::endl; // (e2)

	Eigen::Matrix4f expV = V.expr();
	std::cout << "Exponential of v_hat is " << expV << std:: endl; // (f)

	if  (v - expV*v) < 0.0001
	{
		std::cout <<  "v equals to Exp(V)*v" << std::endl;
	}
	else
	{
		std::cout << "Something wrong" << std::endl;
    }

	//Calculating the matrix expontential of matrix m
	Eigen::Matrix4f m = Eigen::MatrixXf::Random(4,4);
	Eigen::Matrix4f expM = m.exp();

	/****************************************************/

	

}