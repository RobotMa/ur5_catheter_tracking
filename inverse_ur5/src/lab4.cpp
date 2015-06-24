#include <Eigen/Core>
#include <ur5/utilities.h>
#include <math.h>
#include <rviz_plot/lab1.h>
#include <rviz_animate/lab2.h>
#include <ur5_class/lab3.h>
#include <inverse_ur5/lab4.h>
#include <functional>
#include <algorithm>
#define PI M_PI
#define approxZero 0.00001

int sign(double x){
	return (x > 0) - (x < 0);
}

Eigen::MatrixXf J(double q[6])
{
	Eigen::MatrixXf J(6,6);
	Eigen::Matrix4f H06 = UR5::fwd(q);
	Eigen::Matrix4f H = Eigen::MatrixXf::Identity(4,4);
	for(int i = 0; i < 6; i++)
	{
		J.block<3,1>(0,i) = skew3(H.block<3,1>(0,2))*(H06.block<3,1>(0,3)-H.block<3,1>(0,3));
		J.block<3,1>(3,i) = H.block<3,1>(0,2);
		H = H*UR5::dhf(UR5::alpha[i], UR5::a[i], UR5::d[i], q[i]);
	}

	return J;
}

int inverse(Eigen::Matrix4f H0_6, double **q, double q6Des){

	double d1 = UR5::d[0];
	double d4 = UR5::d[3];
	double d5 = UR5::d[4];
	double d6 = UR5::d[5];
	double a2 = UR5::a[1];
	double a3 = UR5::a[2];
	int numSols = 0;

	//Finding the location of the the fifth frame w/ respect
	//to the base frame
	Eigen::Vector4f d6Vec(0.0,0.0,-d6,1);
	Eigen::Vector4f p0_5;
	p0_5 = H0_6*d6Vec;

	double p0_5xy = sqrt(p0_5[0]*p0_5[0]+p0_5[1]*p0_5[1]);

	//Case for no solutions
	if(fabs(UR5::d[4]/p0_5xy) > 1)
	{
		return numSols;
	}
	/*==========================================
	/		Solving for q1
	/=========================================*/
	double q1[2];
	//shoulder right
	double p = atan2(p0_5[1],p0_5[0])+acos(d4/p0_5xy) + PI/2;
	//shoulder left
	double n = atan2(p0_5[1],p0_5[0])-acos(d4/p0_5xy) + PI/2;
	if(fabs(p) < approxZero)
	{
		q1[0] = 0;
	}
	/*	else if (p >= 0.0)
	{
		q1[0] = p;
	} */
	else 
        {
	  q1[0] = p;// + 2*PI;
	}
	if(fabs(n) < approxZero) 
        {
	        q1[1] = 0;
        }
	/*	else if(n >= 0.0)
	{
		q1[1] = n; 
		} */
	else
	{
	  q1[1] = n;// + 2*PI;
	}
	/*==========================================
	/		Solving for q5
	/=========================================*/
	double q5[2][2];
	for(int i = 0; i < 2; i++)
	{
		double arg = (H0_6(0,3)*sin(q1[i])-H0_6(1,3)*cos(q1[i])-d4)/d6;
		//Ensure the arg for acos is in the correct range of [-1,1]
		if (fabs(arg) > 1)
		{
			double numer = (H0_6(0,3)*sin(q1[i])-H0_6(1,3)*cos(q1[i])-d4);
			arg = sign(arg);
		}
		//Wrist up and down
		q5[i][0] = acos(arg);
		q5[i][1] = -q5[i][0];//2*PI-q5[i][0];

	}
	/*==========================================
	/	    Solving for q6,q2-q4
	/=========================================*/
	
	////////////////q6////////////////

	double q6;
	for(int i = 0; i < 2; i++)
	{

	  Eigen::Matrix4f T0_1 = UR5::dhf(UR5::alpha[0],UR5::a[0],UR5::d[0],q1[i]);
		Eigen::Matrix4f T1_6 = finv(T0_1)*H0_6;
		Eigen::Matrix4f T6_1 = finv(T1_6);
		for(int j = 0; j < 2; j++)
		{
			double s5 = sin(q5[i][j]);

			//Case when joints 2,3,4, and 6 are parallel
			if(fabs(s5) < approxZero)
			{
				q6 = q6Des;
			}
			else
			{
				q6 = atan2(-T6_1(1,2)*sign(s5),T6_1(0,2)*sign(s5));
				if(fabs(q6) < approxZero)
					q6 = 0;
				//else if(q6 < 0)
				//	q6 += 2*PI;
			}
			Eigen::Matrix4f T4_6 = UR5::dhf(UR5::alpha[4],UR5::a[4],UR5::d[4],q5[i][j])
			  *UR5::dhf(UR5::alpha[5],UR5::a[5],UR5::d[5],q6);
			Eigen::Matrix4f T1_4 = T1_6*finv(T4_6);
			Eigen::Vector4f P1_3 = T1_4*Eigen::Vector4f(0,-d4,0,1);
			double normP1_3 = sqrt(P1_3(0)*P1_3(0)+P1_3(1)*P1_3(1));
			double c3 = (normP1_3*normP1_3-a2*a2-a3*a3)/(2*a2*a3);
			if(fabs(fabs(c3) -1) < approxZero) 
			{
				c3 = sign(c3);
			}
			else if(fabs(c3) > 1)
			{
				continue;
			}
			/*	double acosArg = (a3*a3 - -a2*a2 -normP1_3*normP1_3)/(2*a2*normP1_3);
			if(fabs(fabs(acosArg) -1) < approxZero) 
			{
				acosArg = sign(acosArg);
				}*/
			////////////////q2////////////////
			double q3[2];
			q3[0] = acos(c3);
			q3[1] = -q3[0];//2*PI-q3[0];
			double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
			double q2[2];
			double s3 = sin(q3[0]);
			double A = (a2 + a3*c3), B = a3*s3;
			for(int k = 0; k < 2; k++)
			{   
				double q2 = -atan2(P1_3(1),-P1_3(0))+asin((a3*sin(q3[k]))/normP1_3);
				if (fabs(q2) < approxZero)
				{
					q2 = 0;
				}
				/*	else if(q2 < 0)
				{
					q2 += 2*PI;
					}*/
				Eigen::Matrix4f T1_3 = (UR5::dhf(UR5::alpha[1],UR5::a[1],UR5::d[1],q2)
							*UR5::dhf(UR5::alpha[2],UR5::a[2],UR5::d[2],q3[k]));
				Eigen::Matrix4f T3_4 = T1_3.inverse()*T1_4;
				double q4 = atan2(T3_4(1,0),T3_4(0,0));
				double modRes;
				if(fabs(q4) < approxZero) 
				{
					q4 = 0;
				}
				/*	else if(q4 < 0)
				{
					q4 += 2*PI;
					}*/
				q[numSols][0] = q1[i]; q[numSols][1] = q2; q[numSols][2] = q3[k];
				q[numSols][3] = q4; q[numSols][4] = q5[i][j]; q[numSols][5] = q6;
				numSols++;
			}
		}
	}
	return numSols;
}
