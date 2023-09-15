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

void dash_dyn::HLIP_dyn_xk2x0(double& x0, double& dx0, VectorXd xk, double w, double Ts) {

    // Given pre-impact state of LIP SSP dynamics find initial conditions, i.e.,
    // post-impact state of current step

    // Get pre-impact states xk
    double xT = xk(0);
    double dxT = xk(1);

    // HLIP SSP dynamics (x0 to xk)
    double c1 = (1.0/2.0) * exp(-w * Ts) * (xT + (1.0 / w) * dxT);
    double c2 = (1.0/2.0) * exp(w * Ts) * (xT - (1.0 / w) * dxT);
    x0 = c1 + c2;
    dx0 = w * (c1 - c2);

}


void dash_dyn::HLIP_S2S_Dyn(MatrixXd& A, VectorXd& B, double T_SSP, double T_DSP, double w) {

    // HLIP S2S Dynamics: x_(k+1) = A*x_k + B*u_k
    
    // Calculations
    double sh = sinh(T_SSP * w); // sinh(T_SSP*w)
    double ch = cosh(T_SSP * w); // cosh(T_SSP*w)

    // State-Transition Matrix: A
    A << ch, T_DSP * ch + (1.0 / w) * sh,
        w * sh, ch + (T_DSP * w * sh);

    // Input-Mapping Vector: B
    B << -ch, -w * sh;
}

void dash_dyn::HLIP_SSP_dyn(double& x, double& dx, double t, double w, double x0, double dx0) {

    // HLIP SSP dynamics

    // x0 to xk (completely passive)
    double c1 = (1.0/2.0) * (x0 + (1.0/w) * dx0);
    double c2 = (1.0/2.0) * (x0 - (1.0/w) * dx0);
    x = c1 * exp(w * t) + c2 * exp(-w * t);
    dx = w * (c1 * exp(w * t) - c2 * exp(-w * t));
}

void dash_dyn::HLIP_DSP_dyn(double& x, double& dx, double t, double x0, double dx0) {

    // HLIP DSP dynamics

    // xk+ to xk+1_0 (post-impact to initial SSP of next step)
    x = x0 + dx0 * t;
    dx = dx0;

}

void dash_dyn::HLIP_Reset_Map_SSP_DSP(Vector2d& x_plus, Vector2d x_minus) {

    // HLIP SSP->DSP reset map

    // get pre-impact states (SSP) -- S2S dynamics state variable
    double p_minus = x_minus(0);
    double v_minus = x_minus(1);

    // reset map (/_\)
    double p_plus = p_minus;
    double v_plus = v_minus;

    // updated CoM states
    x_plus << p_plus, v_plus;

}

void dash_dyn::HLIP_Reset_Map_DSP_SSP(Vector2d& x_plus, Vector2d x_minus, double u) {

    // HLIP DSP->SSP reset map
    
    // get pre-liftoff states (DSP)
    double p_minus = x_minus(0);
    double v_minus = x_minus(1);

    // reset map (/_\)
    double p_plus = p_minus - u;
    double v_plus = v_minus;

    // updated CoM states
    x_plus << p_plus, v_plus;

}
