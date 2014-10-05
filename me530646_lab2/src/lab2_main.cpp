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
	
	/********************** Problem 2 ***********************/
    printf(" /********** Problem 2 **********/ \n \n");

    // Generate a random 3 by 1 vector 
	Eigen::Vector3f v = Eigen::MatrixXf::Random(3,1);
	
	// Output the result of  Problem 2
    // (a) Calculte the 2-norm of vector v
    double normV = v.norm();
	std::cout << "(a) The norm of vector v is \n" 
              << normV << std::endl << std::endl;

    // (b) Normalize vector v
    Eigen::Vector3f v_bar = v/normV;
    std::cout << "(b) Normalized v is \n" 
              <<   v_bar  << std::endl << std::endl;

    // (c) Calculate the skew-symmetric matrix
    Eigen::MatrixXf V = skew3(v);
	std::cout << "(c) The skew symmetric matrix of vector v is \n"
              <<    V     << std::endl << std::endl;

    // (d) Verify that v is in the null space of ss matrix V 
	std::cout << "(d) V*v is \n" 
              <<    V*v   << std::endl << std::endl;

    // (e) Calculate the eigenvalues and eigenvectors of matrix V
	//NOTE : VectorXcf MatrixXcf are used because there may 
	//be complex values for eigenvalues and eigenvectors

    Eigen::EigenSolver<Eigen::MatrixXf> es(V);
	Eigen::VectorXcf evals = es.eigenvalues();
	Eigen::MatrixXcf evecs = es.eigenvectors();
	std::cout << "(e1) Eigenvalues of V are \n"
              << es.eigenvalues()   << std::endl << std::endl;  
	std::cout << "(e2) Eigenvectors of V are \n"
              << es.eigenvectors()  << std::endl << std::endl; 

    // (f) Compute the matrix exponential of the skew-symmetric matrix V
	Eigen::MatrixXf expV = V.exp();
	std::cout << "(f) Exponential of v_hat is \n"
              << expV << std:: endl << std::endl;

    // (g) Verify that vector v in invariant under its own matrix exponential
    Eigen::Vector3f vcomp = v - expV*v;
	if  ( vcomp.norm() < 0.0001 )
	{
		std::cout <<  "(g) v equals to expV*v \n" << std::endl;
	}
	else
	{
		std::cout << "Something wrong \n" << std::endl;
    }

    // (h) Compute the eigenvalues and eigenvectors of expV
    Eigen::EigenSolver<Eigen::MatrixXf> esexp(expV);
    Eigen::VectorXcf evals1 = esexp.eigenvalues();
    Eigen::MatrixXcf evecs1 = esexp.eigenvectors();
    std::cout << "(e1) Eigenvalues of expV are \n"
              << esexp.eigenvalues()   << std::endl << std::endl;  
    std::cout << "(e2) Eigenvectors of expV are \n"
              << esexp.eigenvectors()  << std::endl << std::endl; 

	/********************** Problem 3 ***********************/
    printf(" /********** Problem 3 **********/ \n \n");
    // Create the action transformation
    Eigen::Matrix4f T = Eigen::MatrixXf::Identity(4,4);
    T.block<3,3>(0,0) = expr(v);

    // Create the base frame
    Eigen::Vector3f v1 = Eigen::MatrixXf::Random(3,1);
    Eigen::Matrix4f T1 = Eigen::MatrixXf::Identity(4,4);
    T1.block<3,3>(0,0) = expr(v1);
    // Compute the rotated & transformed base frame
    Eigen::Matrix4f T2 = T*T1;
    // Plot the base frame and the rotated frame in RVIZ
    // Note that p.plotf() only takes 4 by 4 transformation
    p.plotf(T1, "base frame");
    p.plotf(T2, "rotated frame");

    std::cout << "As shown in RVIZ \n " << std::endl;

    /********************** Problem 4 ***********************/
    printf(" /********** Problem 4 **********/ \n \n");
    // (a) Demenstrate that 4 by 4 homogenous transformation is 
    // periodic in regard to rotatio angle with the period of 2*pi
    const double pi = 3.1415926;
    Eigen::VectorXf twi1 = Eigen::MatrixXf::Random(6,1);
    Eigen::VectorXf twi2 = twi1;
    twi2(3) += 2*pi;

    // Compute the difference between two  4 by 4 homogenoeous transformation
    Eigen::Matrix4f D12;
    D12 =  xf( twi1(0),twi1(1),twi1(2),twi1(3),twi1(4),twi1(5) ) 
         - xf( twi2(0),twi2(1),twi2(2),twi2(3),twi2(4),twi2(5) );

    if (D12.squaredNorm() < 0.0001)
    {
        std::cout << " (a) is true \n" << std::endl;
    }
    else
    {
        std::cout << "Something is wrong \n" << std::endl;
    }

    // (b) Prove by example that xfinv(H) is numerically ill-defined for some
    // homogeneous transformations

    // (c)

    // (d)

    /********************** Problem 5 ***********************/
    printf(" /********** Problem 5 **********/ \n \n");
    // (a) Done by hand and is trivial
    std::cout << " (a) Done by hand \n" << std::endl;

    // (b) Plot 3 concatenated frames and animate
    Eigen::Vector3f q = Eigen::MatrixXf::Random(3,1); 
    Eigen::Matrix4f H01 = Eigen::MatrixXf::Identity(4,4);
    H01(0,3) = q(1);

	

}