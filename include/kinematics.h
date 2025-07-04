#include <iostream>
#include <cmath>
#include <../lib/Eigen/Dense>
#include "SRBM-Ctrl/kinematics.h"
#include "SRBM-Ctrl/structs.h"

// typedef Eigen::Matrix<double, 5, 1> Vector5d;

// Motor to Joint Kinematics
Eigen::VectorXd fcn_ik_q_2_p(const Eigen::VectorXd &q);
double fcn_q5_2_alpha(double q5);
double fcn_q5_2_beta(double q5);
double fcn_alphabeta2_lambda(double alpha, double beta);
Eigen::MatrixXd fcn_Jaco_dq_2_dp(const Eigen::VectorXd& eig_q);
Eigen::MatrixXd fcn_Jaco_dp_2_dq(const Eigen::VectorXd& eig_q);

// Joint to Task Kinematics helpers
Eigen::VectorXd tello_leg_IK(Eigen::VectorXd& lf1, Eigen::VectorXd& lf2);
Eigen::VectorXd tello_leg_IK_pointFoot(const Eigen::VectorXd& pos);
Eigen::VectorXd tello_leg_IK_biped(const Eigen::VectorXd& task_positions);

Eigen::MatrixXd fcn_Jaco_dq_2_dT_front(const Eigen::VectorXd& q);
Eigen::MatrixXd fcn_Jaco_dq_2_dT_back(const Eigen::VectorXd& q);

Eigen::VectorXd fk_motors_to_joints(const Eigen::VectorXd& p);
Eigen::VectorXd fk_joints_to_task(const Eigen::VectorXd& q);

Eigen::VectorXd ik_joints_to_motors(const Eigen::VectorXd& q);

Eigen::MatrixXd fcn_lf1_Jv_dot(const Eigen::VectorXd& eig_q, const Eigen::VectorXd& eig_qd);

