#include <iostream>
#include <cmath>
#include <../lib/Eigen/Dense>

// typedef Eigen::Matrix<double, 5, 1> Vector5d;


Eigen::VectorXd fcn_ik_q_2_p(const Eigen::VectorXd &q);

double fcn_q5_2_alpha(double q5);
double fcn_q5_2_beta(double q5);
double fcn_alphabeta2_lambda(double alpha, double beta);
Eigen::MatrixXd fcn_Jaco_dq_2_dp(const Eigen::VectorXd& q);
Eigen::MatrixXd fcn_Jaco_dp_2_dq(const Eigen::VectorXd& q);