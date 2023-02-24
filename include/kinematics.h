#include <iostream>
#include <cmath>
#include <../lib/Eigen/Dense>
#include <srbm_kinematics.h>
#include "srbm_structs.h"

// typedef Eigen::Matrix<double, 5, 1> Vector5d;

// Motor to Joint Kinematics
Eigen::VectorXd fcn_ik_q_2_p(const Eigen::VectorXd &q);
double fcn_q5_2_alpha(double q5);
double fcn_q5_2_beta(double q5);
double fcn_alphabeta2_lambda(double alpha, double beta);
Eigen::MatrixXd fcn_Jaco_dq_2_dp(const Eigen::VectorXd& q);
Eigen::MatrixXd fcn_Jaco_dp_2_dq(const Eigen::VectorXd& q);

// Joint to Task Kinematics helpers
Eigen::VectorXd tello_leg_IK(Eigen::Vector3d& lf1, Eigen::Vector3d& lf2);
Eigen::VectorXd tello_leg_IK_pointFoot(const Eigen::VectorXd& pos);

Eigen::MatrixXd fcn_Jaco_dq_2_dT_front(const Eigen::VectorXd& q);
Eigen::MatrixXd fcn_Jaco_dq_2_dT_back(const Eigen::VectorXd& q);

Eigen::VectorXd fk_motors_to_joints(const Eigen::VectorXd& p);
Eigen::VectorXd fk_joints_to_task(const Eigen::VectorXd& q);

