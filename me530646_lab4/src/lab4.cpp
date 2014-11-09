#include <Eigen/Core>
#include <ur5/utilities.h>
#include <math.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>
#include <me530646_lab4/lab4.h>

#define PI M_PI
#define approxZero 0.00001

Eigen::MatrixXf J(double q[6])
{
  //TODO
}

int inverse(Eigen::Matrix4f H0_6, double **q, double q6Des){
  int numSols = 0;
  /*==========================================
  /       Solving for q1
  /=========================================*/
  double q1[2];

  /*==========================================
  /       Solving for q5
  /=========================================*/
  double q5[2][2];
  /*==========================================
  /       Solving for q6,q2-q4
  /=========================================*/
  //For each solution to q1
  for(int i = 0; i < 2; i++)
  {
    //For each solution to q5
    for(int j = 0; j < 2; j++)
    {
      //For each solution to q2/q3
      for(int k = 0; k < 2; k++)
      {   
        q[numSols][0] = q1[i]; //...
        q[numSols][4] = q5[i][j];
        numSols++;
      }
    }
  }
  return numSols;
}
