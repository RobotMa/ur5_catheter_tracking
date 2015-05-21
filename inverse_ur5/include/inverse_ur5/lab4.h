#ifndef LAB4_H
#define LAB4_H

#include <Eigen/Core>
#include <ur5/utilities.h>
#include <rviz_plot/lab1.h>
#include <rviz_animate/lab2.h>
#include <ur5_class/lab3.h>

int inverse(Eigen::Matrix4f H0_6, double **q, double q6_des = 0.0);

Eigen::MatrixXf J(double q[6]);

#endif
