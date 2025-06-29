#ifndef __TELLO_STRUCTS__
#define __TELLO_STRUCTS__

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;

//#pragma pack(push, 1)

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
    double swing_time_scaler;
};

struct Human_params {
    double m=76;
    double hLIP = 1.05;
    double human_nom_ft_width=0.175;
    double fyH_home;
    double foot_2_joystick;
};

struct Human_dyn_data {
    float xH;
    float dxH;
    float pxH;
    float yH;
    float dyH;
    float pyH;
    float fxH_R;
    float fyH_R;
    float fzH_R;
    float fxH_L;
    float fyH_L;
    float fzH_L;
    float fdxH_R;
    float fdyH_R;
    float fdzH_R;
    float fdxH_L;
    float fdyH_L;
    float fdzH_L;
    float FxH_hmi;
    float FyH_hmi;
    float FxH_spring;
}__attribute__((packed, aligned(1)));

struct Human_dyn_data_4LISAs {
    float t;
    float xH;
    float dxH;
    float pxH;
    float yH;
    float dyH;
    float pyH;

    float r11;
    float r21;
    float r31;
    float r12;
    float r22;
    float r32;
    float r13;
    float r23;
    float r33;

    float roll;
    float pitch;
    float yaw;

    float fxH_R;
    float fyH_R;
    float fzH_R;
    float fxH_L;
    float fyH_L;
    float fzH_L;
    float fdxH_R;
    float fdyH_R;
    float fdzH_R;
    float fdxH_L;
    float fdyH_L;
    float fdzH_L;

    float r_elbow;
    float r_shoulder_pitch;
    float r_shoulder_roll;
    float r_shoulder_yaw;

    float l_shoulder_pitch;
    float l_shoulder_roll;
    float l_shoulder_yaw;
    float l_elbow;

    float DSP_ctrl_trigger_val;

    float fx;
    float fy;
    float my;
    float mz;

    float fx_lim;
    float fy_lim;
    float my_lim;
    float mz_lim;

}__attribute__((packed, aligned(1)));

struct HMI_extended_data {

    float DSP_ctrl_trigger_val;

}__attribute__((packed, aligned(1)));

struct Human_dyn_data_filter {
    VectorXd* xH  = new VectorXd(100);
    VectorXd* dxH  = new VectorXd(100);
    VectorXd* pxH  = new VectorXd(100);
    VectorXd* yH  = new VectorXd(100);
    VectorXd* dyH  = new VectorXd(100);
    VectorXd* pyH  = new VectorXd(100);
    VectorXd* fxH_R  = new VectorXd(100);
    VectorXd* fyH_R  = new VectorXd(100);
    VectorXd* fzH_R  = new VectorXd(100);
    VectorXd* fxH_L  = new VectorXd(100);
    VectorXd* fyH_L  = new VectorXd(100);
    VectorXd* fzH_L  = new VectorXd(100);
    VectorXd* fdxH_R  = new VectorXd(100);
    VectorXd* fdyH_R  = new VectorXd(100);
    VectorXd* fdzH_R  = new VectorXd(100);
    VectorXd* fdxH_L  = new VectorXd(100);
    VectorXd* fdyH_L  = new VectorXd(100);
    VectorXd* fdzH_L  = new VectorXd(100);
    VectorXd* FxH_hmi  = new VectorXd(100);
    VectorXd* FyH_hmi  = new VectorXd(100);
    VectorXd* FxH_spring  = new VectorXd(100);
};

struct Traj_planner_dyn_data
{
    bool stepping_flg;
    double T_step = 0.25;
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
    double x_HWRM;
    double dx_HWRM;
    Vector2d x_plus_HWRM; 
    double uk_HWRM;      
    Vector3d st_beg_step;   
    double y_LIP_offset;
    double step_z_offset_L = 0.016;
    double step_z_offset_R = 0.003;
    int human_FSM = 0;
    int human_next_SSP = 1;
    double T_step_predicted = 0.4;
    double AH_step_predicted = 0.03;
    int curr_SSP_sample_count;
    VectorXd step_z_history_L = VectorXd::Zero(1000);
    VectorXd step_z_history_R = VectorXd::Zero(1000);
    double T_step_actual=0.4;
    double AH_step_actual=0.03;
    double dx_HWRM_pre_impact;
    double dxR_pre_impact;
    double xHR;
    double dxHR;
    double pxHR;
    double xHR_SSP_plus;
    double dxHR_SSP_plus;
    double xHR_DSP_plus;
    double dxHR_DSP_plus;
    int ctrl_mode; 
    double x0H;
    double pxR_beg_step;
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
    double hip_yaw_Ka;
    double hip_roll_Kp;
    double hip_roll_Kd;
    double hip_roll_Ka;
    double hip_pitch_Kp;
    double hip_pitch_Kd;
    double hip_pitch_Ka;
    double knee_Kp;
    double knee_Kd;
    double knee_Ka;
    double ankle_Kp;
    double ankle_Kd;
    double ankle_Ka;
};

//#pragma pack(pop)

#endif
