#ifndef __TELLO_STRUCTS__
#define __TELLO_STRUCTS__

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;

struct SRB_Params
{
    double dt;
    double init_type;
    double g;
    double mu;
    double m;
    double hLIP;
    Matrix3d Ib;
    double W;
    double L;
    double H;
    double thigh_length;
    double calf_length;
    double foot_length;
    double heel_length;
    double CoM2H_z_dist;
    double planner_type;
    double T;
    Vector3d x_sinu_traj_params;
    Vector3d y_sinu_traj_params;
    Vector3d z_sinu_traj_params;
    Vector3d roll_sinu_traj_params;
    Vector3d pitch_sinu_traj_params;
    Vector3d yaw_sinu_traj_params;
    VectorXd vx_des_t; 
    VectorXd vx_des_vx;
    double t_beg_stepping;
    double t_end_stepping;
    double zcl;
    double xDCMH_deadband;
    double KxDCMH;
    double Kx_DCM_mult;
    double Ky_DCM_mult;
    double T_DSP;
    double lmaxR;
    double Kp_xR;
    double Kd_xR;
    double Kp_yR;
    double Kd_yR;
    double Kp_zR;
    double Kd_zR;
    double Kp_phiR;
    double Kd_phiR;
    double Kp_thetaR;
    double Kd_thetaR;
    double Kp_psiR;
    double Kd_psiR;
    double QP_opt_sol_type;
    double W_wrench;
    double W_u_minus_u0_norm; 
    double Act_const_type;
    double tau_m_max;
    double tau_m_stall;
    double alpha_m;
    double beta_trans;
    double gamma_trans;
    double Fz_min_QP;
    double Fz_min_FSM;
    Vector2d q1_lim;
    Vector2d q2_lim;
    double des_walking_speed;
};

struct Human_params {
    double m;
    double hLIP;
    double human_nom_ft_width;
};

struct Human_dyn_data {
    double xH;
    double dxH;
    double pxH;
    double yH;
    double dyH;
    double pyH;
    double fxH_R;
    double fyH_R;
    double fzH_R;
    double fxH_L;
    double fyH_L;
    double fzH_L;
    double fdxH_R;
    double fdyH_R;
    double fdzH_R;
    double fdxH_L;
    double fdyH_L;
    double fdzH_L;
    double FxH_hmi;
    double FyH_hmi;
    double FxH_spring;
};

struct Traj_planner_dyn_data
{
    bool stepping_flg;
    double T_step;
    double t_sw_start;
    double t_dsp_start;
    int next_SSP;
    double step_width;
    Vector3d st2CoM_beg_step;
    Vector3d sw2CoM_beg_step;
    Vector2d xLIP_init;
    Vector3d sw_beg_step;
    Vector3d human_leg_joystick_pos_beg_step;
    double sigma1H;
    bool left_in_contact = true;
    bool right_in_contact = true;
    int left_off_gnd_cnt = 0;
    int right_off_gnd_cnt = 0;
};

struct Teleop_Ref {
    VectorXd time;
    VectorXd FSM_R;
    VectorXd xR_curr;
    VectorXd yR_curr;
    VectorXd fxR_R;
    VectorXd fxR_L;
    VectorXd fyR_R;
    VectorXd fyR_L;
};

struct Teleop_2DLIP_Traj_Data {
    double time;
    double Ts_prev;
    double FSM_R;
    double FSM_H;
    double xR_curr;
    double dxR_curr;
    double pxR_curr;
    double yR_curr;
    double dyR_curr;
    double pyR_curr;
    double xDCMH_ref_curr;
    double pxR_beg_step;
    double fxR_R;
    double fyR_R;
    double fzR_R;
    double fxR_L;
    double fyR_L;
    double fzR_L;
    double yH_curr;
    double dyH_curr;
    double pyH_curr;
    double xH_curr;
    double pxH_curr;
    double fxH_R;
    double fyH_R;
    double fzH_R;
    double fxH_L;
    double fyH_L;
    double fzH_L;
    double FxH_spring;
    double FyH_hmi_comm;
    double FxH_hmi_comm;
    double FxR_ext;
    double phiR_curr;
    double dphiR_curr;
    double MxR_curr;
};

struct Joint_PD_config
{
    double hip_yaw_Kp;
    double hip_yaw_Kd;
    double hip_roll_Kp;
    double hip_roll_Kd;
    double hip_pitch_Kp;
    double hip_pitch_Kd;
    double knee_Kp;
    double knee_Kd;
    double ankle_Kp;
    double ankle_Kd;
};

#endif