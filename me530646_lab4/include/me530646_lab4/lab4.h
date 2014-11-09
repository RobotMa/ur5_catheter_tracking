#ifndef LAB4_H
#define LAB4_H

#include <Eigen/Core>
#include <ur5/utilities.h>
#include <me530646_lab1/lab1.h>
#include <me530646_lab2/lab2.h>
#include <me530646_lab3/lab3.h>

int inverse(Eigen::Matrix4f H0_6, double **q, double q6_des = 0.0);

Eigen::MatrixXf J(double q[6]);

#endif
