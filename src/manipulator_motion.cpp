#include "manipulator_motion.h"

using namespace Eigen;

std::vector<Vector3d> gen_lin_traj_3d_with_duration(const Vector3d& p1, const Vector3d& p2, double duration)
{
    Vector3d dir = p2 - p1;
    double dist = dir.norm();
    double speed = dist / duration; // Speed in m/s
    double timestep = 1.0/(double)UPDATE_HZ; // Time step in seconds (1 millisecond)
    double step_size = speed * timestep; // Distance between each point
    int num_points = (int)(dist / step_size); // Number of points along the line

    std::vector<Vector3d> points(num_points + 1); // Vector of points
    for (int i = 0; i <= num_points; i++) {
        double t = i * timestep;
        points[i] = p1 + t / duration * dir;
    }

    return points;
}

std::vector<Vector3d> gen_lin_traj_3d_with_speed(const Vector3d& p1, const Vector3d& p2, double speed)
{
    Vector3d dir = p2 - p1;
    double dist = dir.norm();
    double speed_m_per_s = speed / 1000; // Convert speed to m/s
    double timestep = 1.0/(double)UPDATE_HZ; // Time step in seconds (1 millisecond)
    double step_size = speed_m_per_s * timestep; // Distance between each point
    int num_points = (int)(dist / step_size); // Number of points along the line

    std::vector<Vector3d> points(num_points + 1); // Vector of points
    for (int i = 0; i <= num_points; i++) {
        double t = i * timestep;
        points[i] = p1 + t / (dist / speed) * dir;
    }

    return points;
}

VectorXd joint_vel_to_motor_vel(const VectorXd& joint_positions, const VectorXd& joint_velocities)
{
    MatrixXd jacobian_inverse = fcn_Jaco_dq_2_dp(joint_positions);
    VectorXd motor_velocities = jacobian_inverse*joint_velocities;
    return motor_velocities;
}

VectorXd motor_vel_to_joint_vel(const VectorXd& joint_positions, const VectorXd& motor_velocities)
{
    MatrixXd jacobian = fcn_Jaco_dp_2_dq(joint_positions);
    VectorXd joint_velocities = jacobian*motor_velocities;
    return joint_velocities;
}

VectorXd joint_torque_to_motor_torque(const VectorXd& joint_positions, const VectorXd& joint_torques)
{
    MatrixXd jacobian = fcn_Jaco_dp_2_dq(joint_positions);
    VectorXd motor_torques = jacobian.transpose()*joint_torques;
    return motor_torques;
}

VectorXd motor_torque_to_joint_torque(const VectorXd& joint_positions, const VectorXd& motor_torques)
{
    MatrixXd jacobian_inverse = fcn_Jaco_dq_2_dp(joint_positions);
    VectorXd joint_torques = jacobian_inverse.transpose()*motor_torques;
    return joint_torques;
}
