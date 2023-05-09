#include "dynamics.h"

void dash_dyn::SRB_Dyn(VectorXd& x_next, VectorXd& net_external_wrench, SRB_Params srb_params, VectorXd x, MatrixXd lfv, VectorXd u, VectorXd tau_ext)
{
    // Parameters
    double m = srb_params.m;
    Matrix3d Ib = srb_params.Ib;
    double g = srb_params.g;
    double dt = srb_params.dt;
    int num_end_effector_pts = 4;
    int position_vec_size = 3;

    // get SRB states
    Vector3d pc_curr = x.segment(0, 3);
    Vector3d dpc_curr = x.segment(3, 3);
    Vector3d EA_curr = x.segment(18, 3);
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d wb_curr = x.segment(15, 3);

    // calculate position vectors from end-effector (s) to CoM
    MatrixXd r_mat(position_vec_size, num_end_effector_pts);
    for (int i = 0; i < num_end_effector_pts; i++) {
        r_mat.col(i) = lfv.row(i).transpose() - pc_curr;
    }

    // get control inputs
    MatrixXd GRF_mat_curr = Eigen::Map<MatrixXd>(u.data(), 3, 4);
    MatrixXd r_mat_curr = r_mat;

    Matrix3d I_3 = Matrix3d::Identity();
    net_external_wrench = VectorXd::Zero(6);
    for (int u_idx = 0; u_idx < 4; u_idx++) {
        Vector3d u_i = GRF_mat_curr.col(u_idx);
        Vector3d r_i = r_mat_curr.col(u_idx);
        Matrix3d hatMap_r_i;
        hatMap_r_i << 0, -r_i(2), r_i(1),
                      r_i(2), 0, -r_i(0),
                     -r_i(1), r_i(0), 0;
        MatrixXd I_3_hatmap_r_i(6,3);
        I_3_hatmap_r_i.block(0,0,3,3) = I_3;
        I_3_hatmap_r_i.block(3,0,3,3) = hatMap_r_i;
        net_external_wrench = net_external_wrench + I_3_hatmap_r_i* u_i;
    }

    // Get wrench forces and torques
    Vector3d F = net_external_wrench.segment(0, 3) + tau_ext.segment(0, 3);
    Vector3d TAU = net_external_wrench.segment(3, 3) + tau_ext.segment(3, 3);

    // gravity force vector
    Vector3d a_g(0, 0, -g);

    // dynamics, i.e. x_dot = f(x, u)
    Vector3d dpc = dpc_curr;
    Vector3d ddpc = (1.0/m)*F + a_g;
    Matrix3d dR_mat = R_curr*(dash_utils::hatMap(wb_curr));
    VectorXd dR = Eigen::Map<VectorXd>(dR_mat.data(), dR_mat.size());
    Vector3d dwb = Ib.inverse()*(R_curr.transpose()*TAU - dash_utils::hatMap(wb_curr)*Ib*wb_curr);
    Vector3d dEA_curr = dash_utils::calc_dEA(R_curr, wb_curr);

    // Euler integration
    x_next.segment<3>(0) = pc_curr + dpc*dt;
    x_next.segment<3>(3) = dpc_curr + ddpc*dt;
    VectorXd Rvec = Eigen::Map<VectorXd>(R_curr.data(), R_curr.size());
    x_next.segment<9>(6) = Rvec + dR*dt;
    x_next.segment<3>(15) = wb_curr + dwb*dt;
    x_next.segment<3>(18) = EA_curr + dEA_curr*dt;

}