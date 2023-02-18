#include "../lib/Eigen/Dense"
#include "kinematics.h"
#include <vector>
#include "user_config.h"

using namespace Eigen;

std::vector<Eigen::Vector3d> gen_lin_traj_3d_with_duration(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double duration);
std::vector<Eigen::Vector3d> gen_lin_traj_3d_with_speed(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double speed);

VectorXd joint_vel_to_motor_vel(const VectorXd& joint_positions, const VectorXd& joint_velocities);
VectorXd motor_vel_to_joint_vel(const VectorXd& joint_positions, const VectorXd& motor_velocities);

VectorXd joint_torque_to_motor_torque(const VectorXd& joint_positions, const VectorXd& joint_torques);
VectorXd motor_torque_to_joint_torque(const VectorXd& joint_positions, const VectorXd& motor_torques);
