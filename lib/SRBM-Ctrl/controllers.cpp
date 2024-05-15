#include "controllers.h"
#include "../../include/user_config.h"

extern MatrixXd lfv_dsp_start;
bool first_time_running_qp = true;
real_t xOpt[12];
QProblem GRFs_distribution_QP;
int prev_FSM=0;
extern bool PS4_connected;
extern double vx_desired_ps4;
extern double vy_desired_ps4;
extern double yaw_desired_ps4;

extern bool use_adaptive_step_time;

double p_star_shared;

double fzH0_min_L = 1000;
double fzH0_min_R = 1000;

double FxH_hmi_out, FxH_spring_out, FyH_hmi_out;

void dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                        SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                        int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext)
{
    // Human-generated walking reference model (HWRM) as static dynamical system
    // ICRA 2023: https://ieeexplore.ieee.org/abstract/document/10160278 
    // Bipedal Robot Walking Control Using Human Whole-Body Dynamic Telelocomotion

    // Get robot parameters
    double mR = srb_params.m; 
    double hR = srb_params.hLIP; 
    double ft_l = srb_params.foot_length;
    double lmaxR = srb_params.lmaxR;
    double g = srb_params.g; 
    double xDCMH_deadband = srb_params.xDCMH_deadband;
    double KxDCMH = 2.0; //srb_params.KxDCMH;
    double Kx_DCM_mult = srb_params.Kx_DCM_mult;
    double Ky_DCM_mult = srb_params.Ky_DCM_mult;
    double T_DSP = srb_params.T_DSP;

    // Get human parameters
    double mH = human_params.m; 
    double hH = human_params.hLIP; 
    // double human_nom_ft_width = human_params.human_nom_ft_width;
    // double fyH_home = human_params.fyH_home;

    // Get trajectory planner data
    double t_sw_start = traj_planner_dyn_data.t_sw_start;
    double T_step = traj_planner_dyn_data.T_step;
    double sigma1H = traj_planner_dyn_data.sigma1H;
    double swx0 = traj_planner_dyn_data.sw_beg_step[0];
    double stx0 = traj_planner_dyn_data.st_beg_step[0];
    double swy0 = traj_planner_dyn_data.sw_beg_step[1];
    double swz0 = traj_planner_dyn_data.sw_beg_step[2];
    double sty0 = traj_planner_dyn_data.st_beg_step[1];
    double stz0 = traj_planner_dyn_data.st_beg_step[2];
    double fyH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step[1];
    double fzH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step[2];

    // Get human dynamic data
    double xH = human_dyn_data.xH; 
    double dxH = human_dyn_data.dxH; 
    double yH = human_dyn_data.yH; 
    double dyH = human_dyn_data.dyH;
    double pyH = human_dyn_data.pyH; 
    double fyH_R = human_dyn_data.fyH_R;
    double fzH_R = human_dyn_data.fzH_R;
    double fyH_L = human_dyn_data.fyH_L;
    double fzH_L = human_dyn_data.fzH_L;
    double fdyH_R = human_dyn_data.fdyH_R;
    double fdzH_R = human_dyn_data.fdzH_R;
    double fdyH_L = human_dyn_data.fdyH_L;
    double fdzH_L = human_dyn_data.fdzH_L;

    // Initialize local variables
    double FyH, xR, dxR, yR, dyR, FxR_ext = 0.0, FyR_ext = 0.0;
    double t_step, s, wR, wH, K_DCM, ks, xR_LIP, yH_LIP;
    double xDCMR_local, xDCMH, yDCMR, yDCMH;
    Vector3d LIPR_params, LIPH_params;

    // Get SRB states
    xR = x(0);
    dxR = x(3);
    yR = x(1) - traj_planner_dyn_data.y_LIP_offset;
    dyR = x(4);

    // Compute LIP natural frequencies
    wR = std::sqrt(g / hR);
    wH = std::sqrt(g / hH);

    // Create LIP parameter vectors for human and robot
    LIPR_params << mR, hR, wR;
    LIPH_params << mH, hH, wH;

    // DCM tracking gain and spring stiffness theoretical values
    K_DCM = mR * hR * wR * wR;
    ks = mH * wH * wH;

    // Human (frontal) and robot (sagittal) LIPs
    xR_LIP = xR - stx0;
    yH_LIP = yH - pyH;

    // DCM calculations
    xDCMR_local = xR_LIP + (dxR / wR);
    xDCMH = xH + (dxH / wH);
    yDCMR = yR + (dyR / wR);
    yDCMH = yH + (dyH / wH);

    // Human (frontal) CoM force
    FyH = mH * wH * wH * yH_LIP;

    // Step time variable (and phase)
    t_step = t - t_sw_start;
    s = t_step / T_step;    

    // Human walking reference LIPM generation

    // Human DCM surrogate outside step-in-place deadband -- use CoM position as surrogate since CoM velocity is noisy
    if (abs(xDCMH) > xDCMH_deadband) {
        // scale human DCM command
        xDCMH = KxDCMH*xH;     
    }
    
    // if step takes longer than assumed duration then update the orbital slope
    // -- since we are extending the reference trajectory
    if (s > 1.0 && abs(FSM) == 1.0) {
        // update orbital line slope
        sigma1H = wH*(1.0/(tanh((t_step/2.0)*wH)));
    }
    
    // end of step calculations for human reference trajectory based on assumed step frequency and actual human DCM -- assuming stable P1 orbit
    double xHf_ref = xDCMH/(1.0 + (sigma1H/wH)); // final human CoM position of reference trajectory
    double dxHf_ref = wH*(xDCMH - xHf_ref); // final human CoM velocity of reference trajectory

    // corresponding initial conditions of P1 orbit
    double xH0_ref = -1.0*xHf_ref; // initial human CoM position of reference trajectory
    double dxH0_ref = dxHf_ref; // initial human CoM velocity of reference trajectory

    // human reference trajectory (based on analytical solution to LIP dyn.)
    double xDCMH0_ref = xH0_ref + (dxH0_ref/wH); // initial human DCM of reference trajectory
    double xDCMH_ref = xDCMH0_ref*exp(wH*t_step); // human DCM reference trajectory at current time step
    //cout << "xDCMH_ref: " << xDCMH_ref << "   xH: " << xH << endl;
    double xH_ref, dxH_ref;
    dash_utils::LIP_dyn_ref(t_step, wH, xH0_ref, dxH0_ref, xH_ref, dxH_ref);

    // update for data logging purposes
    traj_planner_dyn_data.x_HWRM = xH_ref;
    traj_planner_dyn_data.dx_HWRM = dxH_ref;

    // human force profile for desired P1 orbit
    double FxH_ref = mH * wH * wH * xH_ref; // feedforward force to track human reference trajectory

    // virtual spring force to regulate human motion along the force plate
    double FxH_spring = (-1.0 / KxDCMH) * ks * xHf_ref; // LIP model FF

    // during DSP track actual DCM command and assume constant CoM velocity
    // perhaps need to revise
    if (FSM == 0)
    {
        // track actual human DCM
        xDCMH_ref = xDCMH;

        // assumed constant human CoM velocity
        dxH_ref = dxHf_ref;

        // no feedforward
        FxH_ref = 0;
    }
    // Sagittal Plane Control (x-direction)
    // Dynamic Bilateral Teleoperation

    // Enforce dynamic similarity of human walking
    // reference LIP model and robot LIP model
    VectorXd xLIPR_dyn(2);
    xLIPR_dyn << xDCMR_local, dxR;
    VectorXd xLIPH_dyn(2);
    xLIPH_dyn << xDCMH_ref, dxH_ref;
    double FxR_ext_est = FxR_ext;
    double Kx_DCM = Kx_DCM_mult * K_DCM;
    double FxH_hmi;
    bilateral_teleop_law(LIPR_params, LIPH_params, xLIPR_dyn, xLIPH_dyn, FxH_ref, FxR_ext_est, Kx_DCM, FxR, FxH_hmi);
    human_dyn_data.FxH_hmi = FxH_hmi;
    human_dyn_data.FxH_spring = FxH_spring;

    FxH_hmi_out = FxH_hmi;
    FxH_spring_out = FxH_spring;

    // -------------------------------------------------------------------------

    // Frontal Plane Control (y-direction)
    // Dynamic Bilateral Teleoperation

    // Enforce dynamic similarity of actual human
    // LIP model and robot LIP model
    VectorXd yLIPR_dyn(2);
    yLIPR_dyn << yDCMR, dyR;
    VectorXd yLIPH_dyn(2);
    yLIPH_dyn << yDCMH, dyH;
    double FyR_ext_est = FyR_ext;
    double Ky_DCM = Ky_DCM_mult * K_DCM;
    double FyH_hmi;
    bilateral_teleop_law(LIPR_params, LIPH_params, yLIPR_dyn, yLIPH_dyn, FyH, FyR_ext_est, Ky_DCM, FyR, FyH_hmi);
    human_dyn_data.FyH_hmi = FyH_hmi;

    FyH_hmi_out = FyH_hmi;
    // -------------------------------------------------------------------------

     // initialize commanded end-effector positions (DSP)
    lfv_comm = lfv_dsp_start;
    lfv_comm.col(2).setConstant(-srb_params.hLIP);
    lfdv_comm.setZero();

    // swing-leg trajectories
    if (abs(FSM) == 1) { // SSP

        // x-direction step placement strategy

        // minimize error in dynamic similarity between human walking reference
        // LIP model and robot LIP model at step transitions (feedback)
        double l_fb = xDCMR_local - (xDCMH0_ref * (hR/hH));
        // assume constant CoM velocity through DSP (feedforward)
        double l_ff = dxR * T_DSP;
        // overall step length
        double l_cl = l_fb + l_ff;
        // step length magnitude
        double l_mag = abs(l_cl);
        // step direction
        double l_dir = 0.0;
        if (l_cl > 0.0)
            l_dir = 1.0;
        else
            l_dir = -1.0;
        // desired swing-leg x-position at the end of the step
        double swxf = 0.0;
        if (l_mag > lmaxR)
            swxf = stx0 + l_dir*lmaxR;
        else
            swxf = stx0 + l_cl;

        // generate x-direction swing-leg trajectory through the step
        VectorXd swx_traj(2);
        if (s < 1) 
            swx_traj = dash_utils::sw_leg_ref_xy(s, swx0, swxf);
        else {
            swx_traj[0] = swxf;
            swx_traj[1] = 0.0;
        }            

        // set desired end-effector positions based on desired x-direction step
        // placement and tracking normalized human end-effector dynamics
        if (FSM == 1) { // SSP_L
            // x-position trajectories
            lfv_comm(0,0) = swx_traj[0] + (1.0/2.0)*ft_l; lfv_comm(1,0) = swx_traj[0] - (1.0/2.0)*ft_l;
            lfdv_comm(0,0) = swx_traj[1]; lfdv_comm(1,0) = lfdv_comm(0,0);
            // y-position trajectories
            lfv_comm(0,1) = swy0 + (hR/hH)*(fyH_R - fyH0); lfv_comm(1,1) = lfv_comm(0,1);
            lfdv_comm(0,1) = (wR/wH)*fdyH_R*0.0; lfdv_comm(1,1) = lfdv_comm(0,1);
            // z-position trajectories
            lfv_comm(0,2) = swz0 + (hR/hH)*(fzH_R - fzH0); lfv_comm(1,2) = lfv_comm(0,2);
            lfdv_comm(0,2) = (wR/wH)*fdzH_R*0.0; lfdv_comm(1,2) = lfdv_comm(0,2);
        }
        else if (FSM == -1) { // SSP_R
        // x-position trajectories
            lfv_comm(2,0) = swx_traj(0) + 0.5*ft_l; lfv_comm(3,0) = swx_traj(0) - 0.5*ft_l;
            lfdv_comm(2,0) = swx_traj(1); lfdv_comm(3,0) = lfdv_comm(2,0);
            // y-position trajectories
            lfv_comm(2,1) = swy0 - (hR/hH)*(fyH_L - fyH0); lfv_comm(3,1) = lfv_comm(2,1);
            lfdv_comm(2,1) = (wR/wH)*fdyH_L*0.0; lfdv_comm(3,1) = lfdv_comm(2,1);
            // z-position trajectories
            lfv_comm(2,2) = swz0 + (hR/hH)*(fzH_L - fzH0); lfv_comm(3,2) = lfv_comm(2,2);
            lfdv_comm(2,2) = (wR/wH)*fdzH_L*0.0; lfdv_comm(3,2) = lfdv_comm(2,2);
        }            

        // update commanded task space trajectories
        sw_teleop_step_strategy(lfv_comm, lfdv_comm, lfddv_comm, srb_params, human_params, traj_planner_dyn_data, human_dyn_data, FSM, s, swxf, lfv, lfdv, x); 
    }

}

void dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion_v2(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                        SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                        int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext)
{
    // Human-generated walking reference model (HWRM) as discrete dynamical system
    // Humanoids 2023: https://ieeexplore.ieee.org/abstract/document/10375168  
    // Whole-Body Dynamic Telelocomotion: A Step-to-Step Dynamics Approach to Human Walking Reference Generation
    
    // Get robot parameters
    double mR = srb_params.m; 
    double hR = srb_params.hLIP; 
    double ft_l = srb_params.foot_length;
    double lmaxR = srb_params.lmaxR;
    double g = srb_params.g; 
    double xDCMH_deadband = srb_params.xDCMH_deadband;
    double KxDCMH = srb_params.KxDCMH;
    double Kx_DCM_mult = srb_params.Kx_DCM_mult;
    double Ky_DCM_mult = srb_params.Ky_DCM_mult;
    double T_DSP = srb_params.T_DSP;

    // Get human parameters
    double mH = human_params.m; 
    double hH = human_params.hLIP; 

    // Get trajectory planner data
    double t_sw_start = traj_planner_dyn_data.t_sw_start;
    double t_dsp_start = traj_planner_dyn_data.t_dsp_start;
    double swing_T_scaler = srb_params.swing_time_scaler;
    double T_step = traj_planner_dyn_data.T_step;
    if(use_adaptive_step_time) T_step = traj_planner_dyn_data.T_step_predicted*swing_T_scaler;
    // if(abs(FSM) == 1)
    // {
    //     cout << "T_step: " << traj_planner_dyn_data.T_step << endl;
    //     cout << "T_step_predicted: " << traj_planner_dyn_data.T_step_predicted << endl;
    //     cout << "swing_T_scaler: " << swing_T_scaler << endl;
    // }
    double sigma1H = traj_planner_dyn_data.sigma1H;
    double swx0 = traj_planner_dyn_data.sw_beg_step[0];
    double swy0 = traj_planner_dyn_data.sw_beg_step[1];
    double swz0 = traj_planner_dyn_data.sw_beg_step[2];

    double stx0 = traj_planner_dyn_data.st_beg_step[0];
    double sty0 = traj_planner_dyn_data.st_beg_step[1];
    double stz0 = traj_planner_dyn_data.st_beg_step[2];

    double fyH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step[1];
    double fzH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step[2];
    Vector2d x_plus_HWRM = traj_planner_dyn_data.x_plus_HWRM;

    // Get human dynamic data
    double xH = human_dyn_data.xH; 
    double dxH = human_dyn_data.dxH; 
    double yH = human_dyn_data.yH; 
    double dyH = human_dyn_data.dyH;
    double pyH = human_dyn_data.pyH; 
    double fyH_R = human_dyn_data.fyH_R;
    double fzH_R = human_dyn_data.fzH_R;
    double fyH_L = human_dyn_data.fyH_L;
    double fzH_L = human_dyn_data.fzH_L;
    double fdyH_R = human_dyn_data.fdyH_R;
    double fdzH_R = human_dyn_data.fdzH_R;
    double fdyH_L = human_dyn_data.fdyH_L;
    double fdzH_L = human_dyn_data.fdzH_L;

    // Initialize local variables
    double FyH, xR, dxR, yR, dyR, FxH_spring, FxR_ext = 0.0, FyR_ext = 0.0;
    double t_step, s, wR, wH, K_DCM, ks, xR_LIP, yH_LIP;
    double xDCMR_local, xDCMR, xDCMH, yDCMR, yDCMH;
    double x_HWRM, dx_HWRM, xDCM_HWRM, Fx_HWRM; 
    double x_HWRM_pre_impact, dx_HWRM_pre_impact, uk_HWRM, x0_HWRM_next_step, dx0_HWRM_next_step, xDCM_HWRM_next_step;
    double pxR_lim_lb, pxR_lim_ub, Ts;
    double p_star, v_star;
    VectorXd xk_HWRM_des(2), xk_HWRM(2), xkp1_HWRM(2);
    MatrixXd A_S2S(2, 2);
    VectorXd B_S2S(2);
    Vector3d LIPR_params, LIPH_params;
    
    // Step time variable
    if (abs(FSM) == 1) { // SSP
        t_step = t - t_sw_start;
    } else { // DSP
        t_step = t - t_dsp_start;
    }
    // Compute phase variable
    s = t_step / T_step;     

    // cout << "x_HWRM: " << x_HWRM << "     dx_HWRM: " << dx_HWRM << "    t_step: " << t_step;

    // Get SRB states
    xR = x(0);
    dxR = x(3);
    yR = x(1) - traj_planner_dyn_data.y_LIP_offset;
    dyR = x(4);

    // Compute LIP natural frequencies
    wR = std::sqrt(g / hR);
    wH = std::sqrt(g / hH);

    // Create LIP parameter vectors for human and robot
    LIPR_params << mR, hR, wR;
    LIPH_params << mH, hH, wH;

    // DCM tracking gain and spring stiffness theoretical values
    K_DCM = mR * hR * wR * wR;
    ks = mH * wH * wH;

    // Human (frontal) and robot (sagittal) LIPs
    xR_LIP = xR - stx0;
    yH_LIP = yH - pyH;

    // DCM calculations
    xDCMR = xR + (dxR / wR);
    xDCMR_local = xR_LIP + (dxR / wR);
    xDCMH = xH + (dxH / wH);
    yDCMR = yR + (dyR / wR);
    yDCMH = yH + (dyH / wH);

    // Human (frontal) CoM force
    FyH = mH * wH * wH * yH_LIP;   

    // HWRM Dynamics

    // H-LIP Model
    if (abs(FSM) == 1) { // SSP

        // SSP HWRM dynamics
        dash_dyn::HLIP_SSP_dyn(x_HWRM, dx_HWRM, t_step, wH, x_plus_HWRM(0), x_plus_HWRM(1));

    } else { // DSP

        // DSP HWRM dynamics
        dash_dyn::HLIP_DSP_dyn(x_HWRM, dx_HWRM, t_step, x_plus_HWRM(0), x_plus_HWRM(1));

        // Condition to switch from H-LIP model to actual human pilot LIP

        // Based on robot's static stability 
        pxR_lim_lb = lfv.col(0).minCoeff(); pxR_lim_ub = lfv.col(0).maxCoeff();

        // Use actual Human DCM to stabilize robot around zero velocity
        // if ((xDCMR > pxR_lim_lb && xDCMR < pxR_lim_ub) && (t_step > T_DSP)) {
        //     x_HWRM = xH;
        //     dx_HWRM = dxH;
        // }

    }    
    
    // Compute HWRM DCM
    xDCM_HWRM = x_HWRM + (dx_HWRM / wH);    

    // HWRM force profile
    Fx_HWRM = mH * wH * wH * x_HWRM;

    // LISA spring force (apply to actual human)
    // Equal to the opposite GRF of the HWRM
    FxH_spring = 0.0;//-1.0 * Fx_HWRM;   

    // Update HWRM dynamics in traj_planner_dyn_data
    traj_planner_dyn_data.x_HWRM = x_HWRM;
    traj_planner_dyn_data.dx_HWRM = dx_HWRM;

    //cout << "FSM: "<< FSM << "    t_step: " << t - t_sw_start << "    x_plus_HWRM: " << x_plus_HWRM.transpose() << "    x_minus_HWRM: " << traj_planner_dyn_data.x_HWRM << " " << traj_planner_dyn_data.dx_HWRM <<  "    uk_HWRM: " << traj_planner_dyn_data.uk_HWRM << endl;
    

    // Sagittal Plane Control (x-direction)
    // Dynamic Bilateral Teleoperation

    // Enforce dynamic similarity of human walking
    // reference LIP model and robot LIP model
    VectorXd xLIPR_dyn(2);
    xLIPR_dyn << xDCMR_local, dxR;
    VectorXd xLIPH_dyn(2);
    xLIPH_dyn << xDCM_HWRM, dx_HWRM;
    double FxR_ext_est = FxR_ext;
    double Kx_DCM = Kx_DCM_mult * K_DCM;
    double FxH_hmi;
    bilateral_teleop_law(LIPR_params, LIPH_params, xLIPR_dyn, xLIPH_dyn, Fx_HWRM, FxR_ext_est, Kx_DCM, FxR, FxH_hmi);
    human_dyn_data.FxH_hmi = FxH_hmi;
    human_dyn_data.FxH_spring = FxH_spring;

    FxH_hmi_out = FxH_hmi;
    FxH_spring_out = FxH_spring;

    // -------------------------------------------------------------------------

    // Frontal Plane Control (y-direction)
    // Dynamic Bilateral Teleoperation

    // Enforce dynamic similarity of actual human
    // LIP model and robot LIP model
    VectorXd yLIPR_dyn(2);
    yLIPR_dyn << yDCMR, dyR;
    VectorXd yLIPH_dyn(2);
    yLIPH_dyn << yDCMH, dyH;
    double FyR_ext_est = FyR_ext;
    double Ky_DCM = Ky_DCM_mult * K_DCM;
    double FyH_hmi;
    bilateral_teleop_law(LIPR_params, LIPH_params, yLIPR_dyn, yLIPH_dyn, FyH, FyR_ext_est, Ky_DCM, FyR, FyH_hmi);
    human_dyn_data.FyH_hmi = FyH_hmi;

    FyH_hmi_out = FyH_hmi;
    // -------------------------------------------------------------------------

    // In dynamic telelocomotion framework swing-leg trajectories are generated
    // two ways: using human motion capture/re-targeting in the frontal plane
    // and using step placement law that reduces the normalized DCM of the human
    // walking reference LIP and robot LIP at step transitions

    // initialize commanded end-effector positions (DSP)

    // Calculate the center of the right feet positions
    Eigen::Vector3d right_center = lfv.topRows(2).colwise().mean();

    // Calculate the center of the left feet positions
    Eigen::Vector3d left_center = lfv.bottomRows(2).colwise().mean();

    // Compute the new positions for each foot to align them properly
    Eigen::MatrixXd lfv_straight = lfv;
    lfv_straight.topRows(2).col(1).setConstant(right_center(1)); // Set y value for right feet
    lfv_straight.bottomRows(2).col(1).setConstant(left_center(1)); // Set y value for left feet
    lfv_straight(0,0) = (right_center(0) + 0.06 ); // Adjust x value for right feet
    lfv_straight(1,0) = (right_center(0) - 0.06 ); // Adjust x value for right feet
    lfv_straight(2,0) = (left_center(0)  + 0.06 ); // Adjust x value for left feet
    lfv_straight(3,0) = (left_center(0)  - 0.06 ); // Adjust x value for left feet

    lfv_comm = lfv_straight;//lfv_dsp_start;
    lfv_comm.col(2).setConstant(-srb_params.hLIP);
    lfdv_comm.setZero();

    // swing-leg trajectories
    if (abs(FSM) == 1) { // SSP

        // x-direction step placement strategy

        // dynamic step time variable
        Ts = T_step;  

        // Teleoperation Law Mapping (end-of-step DCM)
        // Corresponds, along with previous step frequency, to a desired stable P1 orbit for the HWRM
        if (abs(xDCMH) > xDCMH_deadband) {
            // scale human DCM command and use CoM as surrogate for DCM
            xDCMH = KxDCMH * xH;     
        }

        // check if step is taking longer than predicted
        if (s > 1.0) {
            Ts = t_step;
            sigma1H = wH*(1.0/(tanh((Ts/2.0)*wH)));
        }      

        double xDCMH_shifted;
        if
        (xDCMH > 0) xDCMH_shifted = xDCMH - xDCMH_deadband;
        else xDCMH_shifted = xDCMH + xDCMH_deadband;

        // Desired P1 orbit from human mapping
        double tdsp = T_DSP;
        double tssp = Ts;
        double u_star = 0.5*(tdsp+tssp);
        p_star = xDCMH_shifted / (1.0 + (sigma1H / wH));
        v_star = wH * (xDCMH_shifted - p_star);

        p_star_shared = p_star;

        // p_star = u_star/(2.0+tdsp*sigma1H);
        // v_star = sigma1H*p_star;

        if(abs(xDCMH) < xDCMH_deadband){
            p_star = 0.0;
            v_star = 0.0;
        }
        
        xk_HWRM_des << p_star, v_star;

        // Calculate estimated pre-impact state of HWRM
        dash_dyn::HLIP_SSP_dyn(x_HWRM_pre_impact, dx_HWRM_pre_impact, Ts, wH, x_plus_HWRM(0), x_plus_HWRM(1));
        xk_HWRM << x_HWRM_pre_impact, dx_HWRM_pre_impact;

        // Optimal control policy for HWRM-LIP (based on S2S dynamics)
        dash_ctrl::opt_stepping_controller(uk_HWRM, xk_HWRM, xk_HWRM_des, Ts, T_DSP, wH);
        traj_planner_dyn_data.uk_HWRM = uk_HWRM;

        // cout << "    Controller uk_HWRM: " << uk_HWRM << endl;

        // S2S Dynamics (next-step)
        dash_dyn::HLIP_S2S_Dyn(A_S2S, B_S2S, Ts, T_DSP, wH);
        xkp1_HWRM = A_S2S * xk_HWRM + B_S2S * uk_HWRM;

        // Compute predicted begining-of-next step CoM values (back-calculate based on S2S dynamics)
        dash_dyn::HLIP_dyn_xk2x0(x0_HWRM_next_step, dx0_HWRM_next_step, xkp1_HWRM, wH, Ts);
    
        // Predicted beginning of next step DCM of HWRM
        xDCM_HWRM_next_step = x0_HWRM_next_step + (dx0_HWRM_next_step / wH);         

        // minimize error in dynamic similarity between human walking reference
        // LIP model and robot LIP model at step transitions (feedback)
        double l_fb = xDCMR_local - xDCM_HWRM_next_step*(hR/hH);

        // assume constant CoM velocity through DSP (feedforward)
        double l_ff = dxR*T_DSP;

        // overall step length
        double l_cl = l_fb + l_ff;

        // step length magnitude
        double l_mag = abs(l_cl);

        // step direction
        double l_dir = 0.0;
        if (l_cl > 0.0)
            l_dir = 1.0;
        else
            l_dir = -1.0;

        // desired swing-leg x-position at the end of the step
        double swxf = 0.0;
        if (l_mag > lmaxR)
            swxf = stx0 + l_dir*lmaxR;
        else
            swxf = stx0 + l_cl;

        // generate x-direction swing-leg trajectory through the step
        VectorXd swx_traj(2);
        if (s < 1) 
            swx_traj = dash_utils::sw_leg_ref_xy(s, swx0, swxf);
        else {
            swx_traj[0] = swxf;
            swx_traj[1] = 0.0;
        }

        // update commanded task space trajectories

        // set desired end-effector positions based on desired x-direction step
        // placement and tracking normalized human end-effector dynamics
        if (FSM == 1) { // SSP_L
            // x-position trajectories
            lfv_comm(0,0) = swx_traj[0] + (1.0/2.0)*ft_l; lfv_comm(1,0) = swx_traj[0] - (1.0/2.0)*ft_l;
            lfdv_comm(0,0) = swx_traj[1]; lfdv_comm(1,0) = lfdv_comm(0,0);
            // y-position trajectories
            lfv_comm(0,1) = swy0 + (hR/hH)*(fyH_R - fyH0); lfv_comm(1,1) = lfv_comm(0,1);
            lfdv_comm(0,1) = (wR/wH)*fdyH_R*0.0; lfdv_comm(1,1) = lfdv_comm(0,1);
            // z-position trajectories
            lfv_comm(0,2) = swz0 + (hR/hH)*(fzH_R - fzH0); lfv_comm(1,2) = lfv_comm(0,2);
            lfdv_comm(0,2) = (wR/wH)*fdzH_R*0.0; lfdv_comm(1,2) = lfdv_comm(0,2);
        }
        else if (FSM == -1) { // SSP_R
        // x-position trajectories
            lfv_comm(2,0) = swx_traj(0) + 0.5*ft_l; lfv_comm(3,0) = swx_traj(0) - 0.5*ft_l;
            lfdv_comm(2,0) = swx_traj(1); lfdv_comm(3,0) = lfdv_comm(2,0);
            // y-position trajectories
            lfv_comm(2,1) = swy0 - (hR/hH)*(fyH_L - fyH0); lfv_comm(3,1) = lfv_comm(2,1);
            lfdv_comm(2,1) = (wR/wH)*fdyH_L*0.0; lfdv_comm(3,1) = lfdv_comm(2,1);
            // z-position trajectories
            lfv_comm(2,2) = swz0 + (hR/hH)*(fzH_L - fzH0); lfv_comm(3,2) = lfv_comm(2,2);
            lfdv_comm(2,2) = (wR/wH)*fdzH_L*0.0; lfdv_comm(3,2) = lfdv_comm(2,2);
        }
        // update commanded task space trajectories
        sw_teleop_step_strategy(lfv_comm, lfdv_comm, lfddv_comm, srb_params, human_params, traj_planner_dyn_data, human_dyn_data, FSM, s, swxf, lfv, lfdv, x);
    } 

}

void dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion_v3(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                        SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                        int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext, VectorXd u)
{
    // Human-generated walking reference model (HWRM) as discrete dynamical system
    // Human haptic feedback law based on synchronizing end-of-step DCM
    // Human modeled as mass-spring-damper system
    // Unpublished work

    // Get robot parameters
    double mR = srb_params.m; 
    double hR = srb_params.hLIP; 
    double ft_l = srb_params.foot_length;
    double lmaxR = srb_params.lmaxR;
    double g = srb_params.g; 
    double xDCMH_deadband = srb_params.xDCMH_deadband;
    double KxDCMH = srb_params.KxDCMH;
    double Kx_DCM_mult = srb_params.Kx_DCM_mult;
    double Ky_DCM_mult = srb_params.Ky_DCM_mult;
    double T_DSP = srb_params.T_DSP;

    // Get human parameters
    double mH = human_params.m; 
    double hH = human_params.hLIP; 

    // Get trajectory planner data
    double t_sw_start = traj_planner_dyn_data.t_sw_start;
    double t_dsp_start = traj_planner_dyn_data.t_dsp_start;
    double swing_T_scaler = srb_params.swing_time_scaler;
    double T_step = traj_planner_dyn_data.T_step;
    if(use_adaptive_step_time) T_step = traj_planner_dyn_data.T_step_predicted*swing_T_scaler;
    double sigma1H = traj_planner_dyn_data.sigma1H;
    double swx0 = traj_planner_dyn_data.sw_beg_step[0];
    double swy0 = traj_planner_dyn_data.sw_beg_step[1];
    double swz0 = traj_planner_dyn_data.sw_beg_step[2];

    double stx0 = traj_planner_dyn_data.st_beg_step[0];
    double sty0 = traj_planner_dyn_data.st_beg_step[1];
    double stz0 = traj_planner_dyn_data.st_beg_step[2];

    double fyH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step[1];
    double fzH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step[2];
    Vector2d x_plus_HWRM = traj_planner_dyn_data.x_plus_HWRM;

    // Get human dynamic data
    double xH = human_dyn_data.xH; 
    double dxH = human_dyn_data.dxH; 
    double yH = human_dyn_data.yH; 
    double dyH = human_dyn_data.dyH;
    double pyH = human_dyn_data.pyH; 
    double fyH_R = human_dyn_data.fyH_R;
    double fzH_R = human_dyn_data.fzH_R;
    double fyH_L = human_dyn_data.fyH_L;
    double fzH_L = human_dyn_data.fzH_L;
    double fdyH_R = human_dyn_data.fdyH_R;
    double fdzH_R = human_dyn_data.fdzH_R;
    double fdyH_L = human_dyn_data.fdyH_L;
    double fdzH_L = human_dyn_data.fdzH_L;

    // Initialize local variables
    double FyH, xR, dxR, yR, dyR, FxH_spring, FxR_ext = 0.0, FyR_ext = 0.0;
    double t_step, s, wR, wH, K_DCM, ks, xR_LIP, yH_LIP;
    double xDCMR_local, xCCMR_local, xDCMR, xDCMH, yDCMR, yDCMH;
    double x_HWRM, dx_HWRM, xDCM_HWRM, Fx_HWRM; 
    double x_HWRM_pre_impact, dx_HWRM_pre_impact, uk_HWRM, x0_HWRM_next_step, dx0_HWRM_next_step, xDCM_HWRM_next_step;
    double pxR_lim_lb, pxR_lim_ub, Ts;
    double p_star, v_star;
    double F_HMI = 0.0;
    VectorXd xk_HWRM_des(2), xk_HWRM(2), xkp1_HWRM(2);
    MatrixXd A_S2S(2, 2);
    VectorXd B_S2S(2);
    Vector3d LIPR_params, LIPH_params;
    
    // Step time variable
    if (abs(FSM) == 1) { // SSP
        t_step = t - t_sw_start;
    } else { // DSP
        t_step = t - t_dsp_start;
    }
    // Compute phase variable
    s = t_step / T_step;     

    // cout << "x_HWRM: " << x_HWRM << "     dx_HWRM: " << dx_HWRM << "    t_step: " << t_step;

    // Get SRB states
    xR = x(0);
    dxR = x(3);
    yR = x(1) - traj_planner_dyn_data.y_LIP_offset;
    dyR = x(4);

    // Compute LIP natural frequencies
    wR = std::sqrt(g / hR);
    wH = std::sqrt(g / hH);

    // Create LIP parameter vectors for human and robot
    LIPR_params << mR, hR, wR;
    LIPH_params << mH, hH, wH;

    // DCM tracking gain and spring stiffness theoretical values
    K_DCM = mR * hR * wR * wR;
    ks = mH * wH * wH;

    // Human (frontal) and robot (sagittal) LIPs
    xR_LIP = xR - stx0;
    yH_LIP = yH - pyH;

    // DCM & CCM calculations
    xDCMR = xR + (dxR / wR);
    xDCMR_local = xR_LIP + (dxR / wR);
    xCCMR_local = xR_LIP - (dxR / wR); 
    xDCMH = xH + (dxH / wH);
    yDCMR = yR + (dyR / wR);
    yDCMH = yH + (dyH / wH);

    // Human (frontal) CoM force
    FyH = mH * wH * wH * yH_LIP;   

    // HWRM Dynamics
    // H-LIP Model
    if (abs(FSM) == 1) { // SSP

        // SSP HWRM dynamics
        dash_dyn::HLIP_SSP_dyn(x_HWRM, dx_HWRM, t_step, wR, x_plus_HWRM(0), x_plus_HWRM(1));        

    } else { // DSP

        // DSP HWRM dynamics
        dash_dyn::HLIP_DSP_dyn(x_HWRM, dx_HWRM, t_step, x_plus_HWRM(0), x_plus_HWRM(1));

        // Condition to switch from H-LIP model to actual human pilot LIP

        // Based on robot's static stability 
        pxR_lim_lb = lfv.col(0).minCoeff(); pxR_lim_ub = lfv.col(0).maxCoeff();

        // Use actual Human DCM to stabilize robot around zero velocity
        // if ((xDCMR > pxR_lim_lb && xDCMR < pxR_lim_ub) && (t_step > T_DSP)) {
        //     x_HWRM = xH;
        //     dx_HWRM = dxH;
        // }

    }    
    
    // Compute HWRM DCM
    xDCM_HWRM = x_HWRM + (dx_HWRM / wR);    

    // HWRM force profile
    Fx_HWRM = mR * wR * wR * x_HWRM;

    // LISA spring force (apply to actual human)
    // Equal to the opposite GRF of the HWRM
    FxH_spring = 0.0;//-1.0 * Fx_HWRM;   

    // Update HWRM dynamics in traj_planner_dyn_data
    traj_planner_dyn_data.x_HWRM = x_HWRM;
    traj_planner_dyn_data.dx_HWRM = dx_HWRM;

    //cout << "FSM: "<< FSM << "    t_step: " << t - t_sw_start << "    x_plus_HWRM: " << x_plus_HWRM.transpose() << "    x_minus_HWRM: " << traj_planner_dyn_data.x_HWRM << " " << traj_planner_dyn_data.dx_HWRM <<  "    uk_HWRM: " << traj_planner_dyn_data.uk_HWRM << endl;
    

    // Sagittal Plane Control (x-direction)
    // Dynamic Bilateral Teleoperation

    // Enforce dynamic similarity of human walking
    // reference LIP model and robot LIP model
    VectorXd xLIPR_dyn(2);
    xLIPR_dyn << xDCMR_local, dxR;
    VectorXd xLIPH_dyn(2);
    xLIPH_dyn << xDCM_HWRM, dx_HWRM;
    double FxR_ext_est = FxR_ext;
    double Kx_DCM = Kx_DCM_mult * K_DCM;
    double FxH_hmi;
    bilateral_teleop_law(LIPR_params, LIPR_params, xLIPR_dyn, xLIPH_dyn, Fx_HWRM, FxR_ext_est, Kx_DCM, FxR, FxH_hmi);
    human_dyn_data.FxH_hmi = FxH_hmi;
    human_dyn_data.FxH_spring = FxH_spring;

    FxH_hmi_out = F_HMI; // V3: used to be FxH_hmi
    FxH_spring_out = FxH_spring;

    // -------------------------------------------------------------------------

    // Frontal Plane Control (y-direction)
    // Dynamic Bilateral Teleoperation

    // Enforce dynamic similarity of actual human
    // LIP model and robot LIP model
    VectorXd yLIPR_dyn(2);
    yLIPR_dyn << yDCMR, dyR;
    VectorXd yLIPH_dyn(2);
    yLIPH_dyn << yDCMH, dyH;
    double FyR_ext_est = FyR_ext;
    double Ky_DCM = Ky_DCM_mult * K_DCM;
    double FyH_hmi;
    bilateral_teleop_law(LIPR_params, LIPH_params, yLIPR_dyn, yLIPH_dyn, FyH, FyR_ext_est, Ky_DCM, FyR, FyH_hmi);
    human_dyn_data.FyH_hmi = FyH_hmi;

    FyH_hmi_out = FyH_hmi;
    // -------------------------------------------------------------------------

    // In dynamic telelocomotion framework swing-leg trajectories are generated
    // two ways: using human motion capture/re-targeting in the frontal plane
    // and using step placement law that reduces the normalized DCM of the human
    // walking reference LIP and robot LIP at step transitions

    // initialize commanded end-effector positions (DSP)
    lfv_comm = lfv_dsp_start;
    lfv_comm.col(2).setConstant(-srb_params.hLIP);
    lfdv_comm.setZero();

    // swing-leg trajectories
    if (abs(FSM) == 1) { // SSP

        // x-direction step placement strategy

        // dynamic step time variable
        Ts = T_step;  

        // Teleoperation Law Mapping (end-of-step DCM)
        // Corresponds, along with previous step frequency, to a desired stable P1 orbit for the HWRM
        if (abs(xDCMH) > xDCMH_deadband) {
            // scale human DCM command and use CoM as surrogate for DCM
            xDCMH = KxDCMH * xH;     
        }

        // check if step is taking longer than predicted
        // if (s > 1.0) {
        //     Ts = t_step;
        // }      

        double sigma1R = wR*(1.0/(tanh((Ts/2.0)*wR)));

        double xDCMH_shifted;
        if
        (xDCMH > 0) xDCMH_shifted = xDCMH - xDCMH_deadband;
        else xDCMH_shifted = xDCMH + xDCMH_deadband;

        // Desired P1 orbit from human mapping
        double tdsp = T_DSP;
        double tssp = Ts;
        double u_star = 0.5*(tdsp+tssp);
        p_star = xDCMH_shifted / (1.0 + (sigma1R / wR)); // V3: changed to signma1R wR
        v_star = wR * (xDCMH_shifted - p_star);

        // p_star = u_star/(2.0+tdsp*sigma1H);
        // v_star = sigma1H*p_star;

        if(abs(xDCMH) < xDCMH_deadband){
            p_star = 0.0;
            v_star = 0.0;
        }
        
        xk_HWRM_des << p_star, v_star;

        // Calculate estimated pre-impact state of HWRM 
        dash_dyn::HLIP_SSP_dyn(x_HWRM_pre_impact, dx_HWRM_pre_impact, Ts, wR, x_plus_HWRM(0), x_plus_HWRM(1));
        xk_HWRM << x_HWRM_pre_impact, dx_HWRM_pre_impact;

        // Optimal control policy for HWRM-LIP (based on S2S dynamics)
        dash_ctrl::opt_stepping_controller(uk_HWRM, xk_HWRM, xk_HWRM_des, Ts, T_DSP, wR);
        traj_planner_dyn_data.uk_HWRM = uk_HWRM;

        // cout << "    Controller uk_HWRM: " << uk_HWRM << endl;

        // S2S Dynamics (next-step)
        dash_dyn::HLIP_S2S_Dyn(A_S2S, B_S2S, Ts, T_DSP, wR);
        xkp1_HWRM = A_S2S * xk_HWRM + B_S2S * uk_HWRM;

        // Compute predicted begining-of-next step CoM values (back-calculate based on S2S dynamics)
        dash_dyn::HLIP_dyn_xk2x0(x0_HWRM_next_step, dx0_HWRM_next_step, xkp1_HWRM, wR, Ts);
    
        // Predicted beginning of next step DCM of HWRM
        xDCM_HWRM_next_step = x0_HWRM_next_step + (dx0_HWRM_next_step / wR);         

        // minimize error in dynamic similarity between human walking reference
        // LIP model and robot LIP model at step transitions (feedback)
        double l_fb = xDCMR_local - xDCM_HWRM_next_step;

        // assume constant CoM velocity through DSP (feedforward)
        double l_ff = dxR*T_DSP;

        // overall step length
        double l_cl = l_fb + l_ff;

        // step length magnitude
        double l_mag = abs(l_cl);

        // step direction
        double l_dir = 0.0;
        if (l_cl > 0.0)
            l_dir = 1.0;
        else
            l_dir = -1.0;

        // desired swing-leg x-position at the end of the step
        double swxf = 0.0;
        if (l_mag > lmaxR)
            swxf = stx0 + l_dir*lmaxR;
        else
            swxf = stx0 + l_cl;

        // generate x-direction swing-leg trajectory through the step
        VectorXd swx_traj(2);
        if (s < 1) 
            swx_traj = dash_utils::sw_leg_ref_xy(s, swx0, swxf);
        else {
            swx_traj[0] = swxf;
            swx_traj[1] = 0.0;
        }

        // update commanded task space trajectories

        // set desired end-effector positions based on desired x-direction step
        // placement and tracking normalized human end-effector dynamics
        if (FSM == 1) { // SSP_L
            // x-position trajectories
            lfv_comm(0,0) = swx_traj[0] + (1.0/2.0)*ft_l; lfv_comm(1,0) = swx_traj[0] - (1.0/2.0)*ft_l;
            lfdv_comm(0,0) = swx_traj[1]; lfdv_comm(1,0) = lfdv_comm(0,0);
            // y-position trajectories
            lfv_comm(0,1) = swy0 + (hR/hH)*(fyH_R - fyH0); lfv_comm(1,1) = lfv_comm(0,1);
            lfdv_comm(0,1) = (wR/wH)*fdyH_R*0.0; lfdv_comm(1,1) = lfdv_comm(0,1);
            // z-position trajectories
            lfv_comm(0,2) = swz0 + (hR/hH)*(fzH_R - fzH0); lfv_comm(1,2) = lfv_comm(0,2);
            lfdv_comm(0,2) = (wR/wH)*fdzH_R*0.0; lfdv_comm(1,2) = lfdv_comm(0,2);
        }
        else if (FSM == -1) { // SSP_R
        // x-position trajectories
            lfv_comm(2,0) = swx_traj(0) + 0.5*ft_l; lfv_comm(3,0) = swx_traj(0) - 0.5*ft_l;
            lfdv_comm(2,0) = swx_traj(1); lfdv_comm(3,0) = lfdv_comm(2,0);
            // y-position trajectories
            lfv_comm(2,1) = swy0 - (hR/hH)*(fyH_L - fyH0); lfv_comm(3,1) = lfv_comm(2,1);
            lfdv_comm(2,1) = (wR/wH)*fdyH_L*0.0; lfdv_comm(3,1) = lfdv_comm(2,1);
            // z-position trajectories
            lfv_comm(2,2) = swz0 + (hR/hH)*(fzH_L - fzH0); lfv_comm(3,2) = lfv_comm(2,2);
            lfdv_comm(2,2) = (wR/wH)*fdzH_L*0.0; lfdv_comm(3,2) = lfdv_comm(2,2);
        }
        // update commanded task space trajectories
        sw_teleop_step_strategy(lfv_comm, lfdv_comm, lfddv_comm, srb_params, human_params, traj_planner_dyn_data, human_dyn_data, FSM, s, swxf, lfv, lfdv, x);

        // haptic feedback to human **************************************************************
        // if this works well we should make into a function!!!!!!!!!!!!!!!!!

        // calculate position vectors from end-effector (s) to CoM
        MatrixXd r_mat(3, 4);
        for (int i = 0; i < 4; i++) {
            r_mat.col(i) = lfv.row(i).transpose() - x.segment<3>(0); // << first three values are pc_curr
        }

        // get control inputs
        MatrixXd GRF_mat_curr = Eigen::Map<MatrixXd>(u.data(), 3, 4);
        MatrixXd r_mat_curr = r_mat;

        Matrix3d I_3 = Matrix3d::Identity();
        VectorXd net_external_wrench = VectorXd::Zero(6);
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

        Vector2d stance_grfs;

        if(FSM == 1) // Left SSP
        {
            stance_grfs << GRF_mat_curr(2,2), GRF_mat_curr(2,3); // left foot z forces
        }

        if(FSM == -1) // Right SSP
        {
            stance_grfs << GRF_mat_curr(2,0), GRF_mat_curr(2,1); // right foot z forces
        }

        double frontContactPosition = 0.06; // Front contact point position in meters
        double backContactPosition = -0.06; // Back contact point position in meters

        // Calculate the total force and moment about the origin
        double totalForce = stance_grfs.sum();
        double moment = stance_grfs[1] * backContactPosition + stance_grfs[0] * frontContactPosition;

        // Calculate the center of pressure along the x-axis
        double copX = moment / totalForce;
        double pxR_CZMP = copX; // x distance from center of stance foot to CoP
        double tau = net_external_wrench(3); // moment from gfrs about y axis
        double pxR_CCMP = pxR_CZMP + (tau / (mR*g)); // need to calculate using robot control: pxR_CCMP = pxR_CZMP + (tau / mR*g)
        double dxR_pre_impact = (1.0 / 2.0) * wR * ((xDCMR_local - pxR_CCMP) * exp(wR * (Ts - t_step)) - (xCCMR_local - pxR_CCMP) * exp(-1.0 * wR * (Ts - t_step)));

        // Error dynamics (map DCM error dynamics to human motion variables)
        double c_prime = (1.0 / tanh(Ts*wR)); // coth(Tw)
        double alpha = (1.0 - (sigma1R / wR)*c_prime) / (1.0 + (sigma1R / wR)); // from periodic orbit mapping
        double e_DCM_pre_impact = ((c_prime - 1.0) / (alpha * wR)) * (dx_HWRM_pre_impact - dxR_pre_impact); // error dynamics for virtual MSD

        // Virtual mass-spring-damper system -- compute F_HMI
        double wn = wH; // natural frequency of virtual MSD -- assume same as natural frequency of human LIP
        double zeta = 0.1; // damping ratio of virtual MSD -- tune as needed
        double k = mH * wH * wH; // spring stiffness
        double b = 2.0 * mH * zeta * wn; // damping coefficient
        F_HMI = -1.0 * (k * e_DCM_pre_impact) - (b * dxH); // haptic force feedback to human in sagittal plane  
        FxH_hmi_out = F_HMI; // update   
        traj_planner_dyn_data.dx_HWRM_pre_impact = dx_HWRM_pre_impact;
        traj_planner_dyn_data.dxR_pre_impact = dxR_pre_impact;
        // cout << "Fx_HMI: " << FxH_hmi_out << "  \t(Ts-t_step): " << (Ts - t_step) << "  \tCoP: " << copX << "  \te_DCM_pre_impact: " << e_DCM_pre_impact << "  \tdxR_pre_impact: " << dxR_pre_impact << "  \txDCMR_local: " << xDCMR_local << "  \txCCMR_local: " << xCCMR_local << "  \tpxR_CCMP: " << pxR_CCMP << endl;

    } 

}

void dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion_v4(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                        SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                        int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext)
{
    // Human-generated walking reference model (HWRM) as continuous dynamical system
    // Targeted RAL/ICRA 2024 work
    // Function written to shadow RAL2024_3DLIP_Sim_v3.c (LabView implementation)

    // Parameters ------------------------------------------------------------------------

    // Constants
    double g = srb_params.g; // acceleration due to gravity in m/s^2 
    double dt = srb_params.dt; // simulation time step

    // Human parameters
    double mH = human_params.m; // human weight in kg 
    double hH = human_params.hLIP; // human CoM height in m 

    // Robot parameters
    double mR = srb_params.m; // robot weight in kg 
    double hR = srb_params.hLIP; // robot CoM height in m  
    double lmaxR = srb_params.lmaxR;; // maximum step length in m     

    // Telelocomotion control parameters 
    double Kx_DCM_mult = srb_params.Kx_DCM_mult; // multiplier of k_DCM for sagittal plane control 
    double Ky_DCM_mult = srb_params.Ky_DCM_mult; // multiplier of k_DCM for frontal plane control
    double T_DSP = srb_params.T_DSP; // assumed duration of DSP
    double swing_T_scaler = srb_params.swing_time_scaler; // human swing-time scaler for step time estimation
    double ks = 1000.0; // human haptic spring stiffness in N/m (hard-code for now)

    // Robot & Human Data ----------------------------------------------------------------

    // time data
    double time = t; // time in s (global)  
    double t_DSP = time - traj_planner_dyn_data.t_dsp_start; // time of current DSP in s
    double t_SSP = time - traj_planner_dyn_data.t_sw_start; // time of current SSP in s
    
    // FSM data 
    int FSM_val = FSM; // FSM value (SSP_L = 1, SSP_R = -1, DSP = 0)

    // human data
    double xH = human_dyn_data.xH; // human x-CoM position in m
    double dxH = human_dyn_data.dxH; // human x-CoM velocity in m/s
    double yH = human_dyn_data.yH; // human y-CoM position in m
    double dyH = human_dyn_data.dyH; // human y-CoM velocity in m/s
    double pyH = human_dyn_data.pyH; // human y-CoP position in m
    double T_SSP = traj_planner_dyn_data.T_step; // human step time estimate in s
    if (use_adaptive_step_time) T_SSP = traj_planner_dyn_data.T_step_predicted * swing_T_scaler; // adjusted human step time estimate in s

    // ************************************* NEED TO ADD HMI INTERFACE!!!
    double DSP_ctrl_trigger_val = 1.0; // DSP control mode trigger val (not pressed = -1, pressed = 1)
    // ************************************* NEED TO ADD HMI INTERFACE!!!

    // human generated robot reference data 
    double xHR = traj_planner_dyn_data.xHR; // human generated robot reference CoM position in m
    double dxHR = traj_planner_dyn_data.dxHR; // human generated robot reference CoM velocity in m/s
    double pxHR = traj_planner_dyn_data.pxHR; // human generated robot reference control CoP position in m
    double xHR_SSP_plus = traj_planner_dyn_data.xHR_SSP_plus; // human generated robot reference CoM position at the beginning of SSP in m
    double dxHR_SSP_plus = traj_planner_dyn_data.dxHR_SSP_plus; // human generated robot reference CoM velocity at the beginning of SSP in m/s
    double xHR_DSP_plus = traj_planner_dyn_data.xHR_DSP_plus; // human generated robot reference CoM position at the beginning of DSP in m
    double dxHR_DSP_plus = traj_planner_dyn_data.dxHR_DSP_plus; // human generated robot reference CoM velocity at the beginning of DSP in m/s 

    // robot data
    double xR = x(0); // robot x-CoM position in m
    double dxR = x(3); // robot x-CoM velocity in m/s
    double yR = x(1) - traj_planner_dyn_data.y_LIP_offset; // robot y-CoM position in m w/ offset
    double dyR = x(4); // robot y-CoM velocity in m/s
    double pxR_beg_step = traj_planner_dyn_data.st_beg_step[0]; // initial robot stance-foot x-position at the beginning of step in m 

    // external force data (tau_ext is zero for now, currently we cannot estimate external forces in hardware)
    double FxR_ext = 0.0; // external x-CoM force in N
    double FyR_ext = 0.0; // external y-CoM force in N

    // Calculations ----------------------------------------------------------------------

    // natural frequencies 
    double wR = std::sqrt(g / hR); // human LIP natural frequency in s^-1
    double wH = std::sqrt(g / hH); // robot LIP natural frequency in s^-1  

    // LIP parameter vectors for human and robot (mass, LIP height, natural frequency)
    Vector3d RLIP_params << mR, hR, wR; 
    Vector3d HLIP_params << mH, hH, wH;   

    // DCM feedback gains
    double k_DCM = mR * hR * (wR * wR); // theoretical feedback gain for normalized DCM tracking in N
    double kx_DCM = Kx_DCM_mult * k_DCM; // feedback gain for sagittal plane normalized DCM tracking in N
    double ky_DCM = Ky_DCM_mult * k_DCM; // feedback gain for frontal plane normalized DCM tracking in N

    // LIPM variables (CoM relative to the CoP)
    double xHR_LIP = xHR - pxHR; // HRLIP sagittal plane in m
    double yH_LIP = yH - pyH; // HLIP frontal plane in m

    // LIPM forces
    double FxHR = mR * (wR * wR) * xHR_LIP; // HRLIP sagittal plane GRF in N
    double FyH = mH * (wH * wH) * yH_LIP; // HLIP frontal plane GRF in N 

    // human haptic spring force
    double FxH_spring = -1.0 * ks * xH; // HLIP haptic spring force in N

    // convergent (CCM) & divergent (DCM) component of motion variables
    double xCCMHR = xHR - (dxHR / wR); // HRLIP sagittal plane CCM in m
    double xDCMHR = xHR + (dxHR / wR); // HRLIP sagittal plane DCM in m
    double xDCMHR_SSP_plus = xHR_SSP_plus + (dxHR_SSP_plus / wR); // HRLIP sagittal plane DCM at the beginning of SSP in m
    double xDCMHR_DSP_plus = xHR_DSP_plus + (dxHR_DSP_plus / wR); // HRLIP sagittal plane DCM at the beginning of DSP in m      
    double xDCMR = xR + (dxR / wR); // RLIP sagittal plane DCM in m
    double xDCMR_local = xDCMR - pxR_beg_step; // RLIP sagittal plane DCM relative to stance foot in m      
    double yDCMH = yH + (dyH / wH); // HLIP frontal plane DCM in m
    double yDCMR = yR + (dyR / wR); // RLIP frontal plane DCM in m          
 
    // Local Variables --------------------------------------------------------------------
    
    // human step time info
    double Ts; // human step time estimate (final) in s
    double s; // normalized SSP time variable (0-1)

    // HRLIP simulation variables (sagittal plane only)
    // Note: HRLIP dynamics evolve relative to the stance foot
    double pxHR_comm; // commanded human generated robot reference control CoP/CMP position in m
    double xHR_next; // human generated robot reference CoM position update in m
    double dxHR_next; // human generated robot reference CoM velocity update in m/s  

    // control variables (DSP mode)
    int DSP_ctrl_mode; // DSP control mode (1 -> Walking, 2 -> Balancing) 

    // Control variables (sagittal plane balance ctrl)
    double HLIP2DCMdelta; // mapping from human LIP variable to DCM delta in m
    double DCMdelta_comm; // DCM delta command in m
    double xDCMHR_passive_SSP = 0.0; // passive HRLIP DCM in m
    double DCM_delta_SSP = 0.0; // DCM delta between HRLIP and passive HRLIP in m
    double xDCMHR_des_DSP = 0.0; // desired HRLIP DCM during DSP in m
    double FxHR_des_DSP = 0.0; // desired HRLIP CoM force to track xDCMHR_des_DSP in N
    VectorXd xHRLIP_dyn(2); // HRLIP sagittal plane state vector 
    VectorXd xRLIP_dyn(2); // RLIP sagittal plane state vector 
    double FxR_ext_est = FxR_ext; // robot sagittal plane estimated external force in N
    double FxR; // total robot sagittal plane force in N
    double FxH_HMI; // human sagittal plane haptic force feedback in N

    // Control variables (frontal plane balance ctrl)
    VectorXd yHLIP_dyn(2); // HLIP frontal plane state vector 
    VectorXd yRLIP_dyn(2); // RLIP frontal plane state vector 
    double FyR_ext_est = FyR_ext; // robot frontal plane estimated external force in N
    double FyR; // total robot frontal plane force in N
    double FyH_HMI; // human frontal plane haptic force feedback in N

    // Control variables (step placement ctrl) -- SSP only
    double s1 = 0.0; // orbital line slope (sigma1) in s^-1
    double eps = 0.0; // DCM offset gain on robot velocity in s
    double xDCMR_offset_b = 0.0; // DCM offset in m
    double step_length = 0.0; // robot step length selected by step-placement control law in m
    double step_magnitude = 0.0; // step magnitude in m
    double step_direction = 1.0;  // step direction (unitless)
    double swxf = 0.0; // desired swing-foot location at the end-of-step in m    

    // Human Step Time --------------------------------------------------------------------

    // Based on curve-fitting & simple phase variable sanity check  

    // human step time estimate (from curve fitting)
    Ts = T_SSP; 

    // calculate phase variable (0-1)
    s = t_SSP / Ts; 

    // update human step time estimate (if curve fitting is working we should not execute this at all)
    if (s >= 1.0) { // step is taking longer than assumed
        Ts = t_SSP + dt; // update step time
        s = 1.0; // approximate phase variable to be 1 at all times
    }     

    // Balance Control (sagittal plane) ---------------------------------------------------

    // Kinodynamic re-targeting using human LIP local variable to regulate HRLIP DCM delta
    // Simulates dynamics of moving reference model (HRLIP) with a control CoP/CMP input
    // SSP: Delta DCM at impact during SSP (walking)
    // DSP Mode 1: Walking -> keep constant robot velocity during DSP
    // DSP Mode 2: Balance -> track DCM reference relative to beginning of DSP DCM
    // Robot controller tracks LIP reference  

    // HRLIP trajectory generation 

    // set DSP control mode
    if (DSP_ctrl_trigger_val < 0.0) { // trigger not pressed (default)
        DSP_ctrl_mode = 1; // walking mode
    } else { // trigger pressed
        DSP_ctrl_mode = 2; // balancing mode
    }

    // human kinodynamic mapping variable
    HLIP2DCMdelta = xH * (hR / hH);

    // compute HRLIP control CMP
    if (abs(FSM_val) == 1) { // SSP

        // SSP ankle + hip strategy
        DCMdelta_comm = HLIP2DCMdelta; // delta DCM at impact command input from human kinodynamic mapping
        xDCMHR_passive_SSP = xDCMHR_SSP_plus * exp(wR * t_SSP); // passive DCM at current time
        DCM_delta_SSP = xDCMHR - xDCMHR_passive_SSP; // DCM delta between actuated and passive trajectories at current time
        pxHR_comm = (DCM_delta_SSP - (DCMdelta_comm * exp(-1.0 * wR * (Ts - t_SSP)))) / (1.0 - exp(-1.0 * wR * (Ts - t_SSP))); // control CoP/CMP

    } else { // DSP

        // DSP ankle + hip strategy
        if (DSP_ctrl_mode == 1) { // walking mode
            DCMdelta_comm = dxHR_DSP_plus * t_DSP; // maintain constant velocity through DSP (Hybrid-LIP approach)    
        } else { // balancing mode (DSP_ctrl_mode == 2)
            DCMdelta_comm = HLIP2DCMdelta; // track human DCM position command
        }

        // track desired DCM delta
        xDCMHR_des_DSP = DCMdelta_comm + xDCMHR_DSP_plus; // desired DCM
        FxHR_des_DSP = (k_DCM / hR) * (xDCMHR_des_DSP - xDCMHR); // convert desired DCM to CoM force
        pxHR_comm = xHR - (FxHR_des_DSP / (mR * (wR * wR))); // convert desired CoM force to control CoP/CMP

    }

    // simulate forward HRLIP dynamics (closed-loop solution)
    xHR_next = (1.0 / 2.0) * (((xDCMHR - pxHR_comm) * exp(wR * dt)) + ((xCCMHR - pxHR_comm) * exp(-1.0 * wR * dt))) + pxHR_comm; // update CoM pos
    dxHR_next = (1.0 / 2.0) * wR * (((xDCMHR - pxHR_comm) * exp(wR * dt)) - ((xCCMHR - pxHR_comm) * exp(-1.0 * wR * dt))); // update CoM vel    

    // robot controller (enforce dynamic similarity between HRLIP & RLIP)
    xRLIP_dyn << xDCMR_local, dxR;
    xHRLIP_dyn << xHR, dxHR;
    bilateral_teleop_law(RLIP_params, RLIP_params, xRLIP_dyn, xHRLIP_dyn, FxHR, FxR_ext_est, kx_DCM, FxR, FxH_HMI);

    // scale haptic force by human weight
    FxH_HMI = (mH / mR) * FxH_HMI;
    
    // update data 
    human_dyn_data.FxH_spring = FxH_spring; FxH_spring_out = FxH_spring;       
    human_dyn_data.FxH_hmi = FxH_HMI; FxH_hmi_out = FxH_HMI;    
    traj_planner_dyn_data.xHR = xHR_next;
    traj_planner_dyn_data.dxHR = dxHR_next;
    traj_planner_dyn_data.pxHR = pxHR_comm;

    // Balance Control (frontal plane) ----------------------------------------------------     

    // Kinodynamic re-targeting enforcing dynamic similarity between the human and robot LIPMs
    // Robot controller tracks LIP reference
    // Includes haptic force feedback to the human to enforce dynamic similarity 

    // robot controller (enforce dynamic similarity between HLIP & RLIP)
    yRLIP_dyn << yDCMR, dyR;
    yHLIP_dyn << yDCMH, dyH;
    bilateral_teleop_law(RLIP_params, HLIP_params, yRLIP_dyn, yHLIP_dyn, FyH, FyR_ext_est, ky_DCM, FyR, FyH_HMI);
    
    // update data 
    human_dyn_data.FyH_hmi = FyH_HMI; FyH_hmi_out = FyH_HMI;    

    // Swing-leg Control ------------------------------------------------------------------ 

    // Update robot swing-leg commands based on human motion data & step placement strategy
    // Sagittal plane step-placement strategy is based on simple linear control framework 
    // Goal is to preserve the LIPM orbital energy at impact                         

    // initialize commanded end-effector positions (DSP)
    lfv_comm = lfv_dsp_start; lfv_comm.col(2).setConstant(-srb_params.hLIP);
    lfdv_comm.setZero();
    lfddv_comm.setZero();

    // x-position step-placement & swing-leg control function call
    if (abs(FSM_val) == 1) { // SSP

        // orbital line slope
        s1 = wR * (1.0 / (tanh(wR * (Ts / 2.0))));

        // DCM offset gain epsilon w/ FF for DSP
        eps = (1.0 / wR) - (1.0 / s1) - T_DSP;

        // DCM offset to drive robot to P1 orbit at given state
        xDCMR_offset_b = eps * dxR;

        // robot step placement strategy: conserve orbital energy & reach steady state periodic walking
        step_length = xDCMR_local - xDCMR_offset_b;

        // calculate step magnitude
        step_magnitude = abs(step_length);

        // step direction
        if (step_length > 0.0) { // walking forward
            step_direction = 1.0; // set to 1
        } else { // walking backwards
            step_direction = -1.0; // set to -1
        }

        // desired swing-leg x-position at the end of the step
        if (step_magnitude > lmaxR) { // exceeded maximum step length
            swxf = pxR_beg_step + step_direction * lmaxR; // set desired swing-foot position to maximum
        } else { // within allowable step length
            swxf = pxR_beg_step + step_length; // set desired swing-foot position to desired
        }    

        // main telelocomotion swing-leg control function call
        sw_teleop_step_strategy(lfv_comm, lfdv_comm, lfddv_comm, srb_params, human_params, traj_planner_dyn_data, human_dyn_data, FSM, s, swxf, lfv, lfdv, x);

    } 

}

bool first_step_ylip = true;
void dash_ctrl::bilateral_teleop_law(VectorXd LIPR_params, VectorXd LIPH_params, VectorXd LIPR_dyn, VectorXd LIPH_dyn, 
                            double FH, double FR_ext_est, double FB_gain, double& FR, double& FH_hmi)
{
    // Dynamic similarity based kinodynamic re-targeting of human motion (LIPM)
    // Major take-away from Prof Ramos PhD for human-robot stepping synchronization
    // TRO 2017: https://ieeexplore.ieee.org/abstract/document/8375643 
    // Humanoid Dynamic Synchronization Through Whole-Body Bilateral Feedback Teleoperation

    // Robot LIP Parameters
    double mR = LIPR_params(0);
    double hR = LIPR_params(1);
    double wR = LIPR_params(2);

    // Human LIP Parameters
    double mH = LIPH_params(0);
    double hH = LIPH_params(1);
    double wH = LIPH_params(2);

    // Robot CoM dynamics
    double DCMR = LIPR_dyn(0);
    double dR = LIPR_dyn(1);

    // Human CoM dynamics
    double DCMH = LIPH_dyn(0);
    double dH = LIPH_dyn(1);

    // feedforward force
    double FR_ff = ((mR*hR*wR*wR)/(mH*hH*wH*wH))*FH;

    // feedback force (track normalized human DCM)
    double FR_fb = FB_gain*((DCMH/hH) - (DCMR/hR));

    // robot force
    FR = FR_ff + FR_fb;

    // haptic feedback force to human
    FH_hmi = (mH*hH*wH*wH)*((dR/(hR*wR)) - (dH/(hH*wH))) + ((mH*hH*wH*wH)/(mR*hR*wR*wR))*FR_ext_est;

}

double yLIP_offset = 0;
void dash_ctrl::LIP_ang_mom_strat(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm,
                                        SRB_Params srb_params, Traj_planner_dyn_data traj_planner_dyn_data, 
                                        int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv)
{
    // Regulation of angular momentum around contact point (ALIP) controller
    // From Umich Biped Robotics Laboratory
    // ICRA 2021: https://ieeexplore.ieee.org/document/9560821
    // One-Step Ahead Prediction of Angular Momentum about the Contact Point for Control of 
    // Bipedal Locomotion: Validation in a LIP-inspired Controller

    // Get robot parameters
    double m = srb_params.m;
    double H = srb_params.hLIP;
    double ft_l = srb_params.foot_length;
    double g = srb_params.g;
    double T = srb_params.T;
    double Kp_xR = srb_params.Kp_xR;
    double Kd_xR = srb_params.Kd_xR;
    double Kp_yR = srb_params.Kp_yR;
    double Kd_yR = srb_params.Kd_yR;
    VectorXd vx_des_t = srb_params.vx_des_t;
    VectorXd vx_des_vx = srb_params.vx_des_vx;
    int num_end_effector_pts = 4;
    int position_vec_size = 3;

    // Get trajectory planner data
    double W = traj_planner_dyn_data.step_width;
    int next_SSP = traj_planner_dyn_data.next_SSP;
    double t_sw_start = traj_planner_dyn_data.t_sw_start;
    double T_step = traj_planner_dyn_data.T_step;
    double xLIP0 = traj_planner_dyn_data.xLIP_init(0);
    double xd0 = traj_planner_dyn_data.xLIP_init(1);

    // Get SRB states
    VectorXd pc = x.segment(0, 3);
    double dx = x(3);
    double dy = x(4);

    // Calculations ------------------------------------------------------------

    // step time variable
    double t_step = t - t_sw_start;

    // compute phase variable
    double s = t_step / T_step;
    
    // get desired velocity in x-direction at end of next step
    // vx_des = f(sim_time)
    //dash_utils::start_timer();
    double vx_des = 0;
    // if((int)(1000*t) < vx_des_vx.size()) vx_des = vx_des_vx((int)(1000*t));
    if(PS4_connected){
        vx_des = vx_desired_ps4;
    }
    //dash_utils::end_timer();
    // LIP states
    MatrixXd lf2CoM_mat = pc.replicate(1, num_end_effector_pts) - lfv.transpose();

    double xLIP, yLIP;
    // calculate LIP states based on current and next footstep
    if(FSM == 1) // SSP_L
    {
        xLIP = lf2CoM_mat(0, 2) + 0.5*ft_l;
        yLIP = lf2CoM_mat(1, 2);
    }
    else if(FSM == -1) // SSP_R
    {
        xLIP = lf2CoM_mat(0, 0) + 0.5*ft_l;
        yLIP = lf2CoM_mat(1, 0);
    }
    else // DSP
    {
        if(next_SSP == 1) // next SSP is SSP_L
        {
            xLIP = lf2CoM_mat(0, 2) + 0.5*ft_l;
            yLIP = lf2CoM_mat(1, 2);
        }
        else if(next_SSP == -1) // next SSP is SSP_R
        {
            xLIP = lf2CoM_mat(0, 0) + 0.5*ft_l;
            yLIP = lf2CoM_mat(1, 0);
        }
        else // terminate walking
        {
            xLIP = 0.5*(lf2CoM_mat(0, 0) + lf2CoM_mat(0, 3)); 
            if(first_step_ylip)
            {
                yLIP = 0.5*(lf2CoM_mat(1, 0) + lf2CoM_mat(1, 2))-yLIP_offset;
                if(yLIP_offset<0.03 && t > 0.5)
                {
                    yLIP_offset = yLIP_offset+0.0001;
                }
                
            }
            else
            {
                yLIP = 0.5*(lf2CoM_mat(1, 0) + lf2CoM_mat(1, 2));
            }
        }
    }
    // printf("Next_SSP: %d\n", next_SSP);
    // printf("lf2CoM_mat:\n");
    // cout << lf2CoM_mat << endl;

    // compute LIP natural frequency
    double omega = sqrt(g/H);

    double sw2CoM_end_step_x;
    double sw2CoM_end_step_y;
    if (abs(FSM) == 1) // SSP
    {
        // compute angular momentum about contact point
        double Lx = m*H*dx;
        // cout << "dx: " << dx << endl;
        double Ly = m*H*dy;

        // compute angular momentum about contact point at the end of current step
        // estimate
        // Eq. 14 from Angular Momentum about the Contact Point for Control of
        // Bipedal Locomotion: Validation in a LIP-based Controller
        double Lxest_end_step = m*H*omega*sinh(omega*(T - t_step))*xLIP + cosh(omega*(T - t_step))*Lx;
        double Lyest_end_step = m*H*omega*sinh(omega*(T - t_step))*yLIP + cosh(omega*(T - t_step))*Ly;

        // compute desired angular momentum about contact point at the end of the
        // next step (periodically oscillating LIP)
        // Eq. 16 from Angular Momentum about the Contact Point for Control of
        // Bipedal Locomotion: Validation in a LIP-based Controller
        double Lxdes_end_next_step = 0;//m*H*vx_des;
        double Lydes_end_next_step_mag = (1.0/2.0)*m*H*W*(omega*sinh(omega*T)/(1.0 + cosh(omega*T)));
        double Lydes_end_next_step = 0.0;
        if (FSM == 1)
        {
            Lydes_end_next_step = 1.0*Lydes_end_next_step_mag;
        }
        else if (FSM == -1)
        {
            Lydes_end_next_step = -1.0*Lydes_end_next_step_mag;
        }

        // compute desired step length (s)
        // Eq. 15 from Angular Momentum about the Contact Point for Control of
        // Bipedal Locomotion: Validation in a LIP-based Controller
        sw2CoM_end_step_x = (Lxdes_end_next_step - cosh(omega*T)*Lxest_end_step)/(m*H*omega*sinh(omega*T));
        //cout << "end_step_x: " << sw2CoM_end_step_x << endl;
        sw2CoM_end_step_y = (Lydes_end_next_step - cosh(omega*T)*Lyest_end_step)/(m*H*omega*sinh(omega*T));
    }
    else // DSP
    {
        // no foot step can be taken
        sw2CoM_end_step_x = 0.0;
        sw2CoM_end_step_y = 0.0;
    }
    
    // sagittal plane control
    double xLIP_ref, xd_ref;
    if (abs(FSM) == 1) 
    {
        first_step_ylip = false;
        dash_utils::LIP_dyn_ref(t_step, omega, xLIP0, xd0, xLIP_ref,xd_ref);
        FxR = Kp_xR*(xLIP_ref - xLIP) + Kd_xR*(xd_ref - dx);
    }
    else
    {
        FxR = m*omega*omega*xLIP;
    }

    // frontal plane control
    if ( (FSM == 0 && next_SSP == 0) || t < srb_params.t_beg_stepping) // terminate walking 
    {
        FyR = -Kp_yR*yLIP - Kd_yR*dy;
        FxR = -Kp_xR*xLIP - Kd_xR*dx;
    }
    else // apply feedforward LIP force
    {
        // if(first_step_ylip)
        //     FyR = 0.9*m*omega*omega*yLIP;
        // else
            FyR = m*omega*omega*yLIP;
    }
    
    // desired step placement
    const Vector2d sw2CoM_end_step_des = {sw2CoM_end_step_x, sw2CoM_end_step_y};
    sw2CoM_end_step_strategy(lfv_comm, lfdv_comm, lfddv_comm, srb_params, traj_planner_dyn_data, FSM, s, x, lfv, lfdv, sw2CoM_end_step_des);

}

void dash_ctrl::sw2CoM_end_step_strategy(MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, const SRB_Params srb_params, const Traj_planner_dyn_data& traj_planner_dyn_data, const int FSM, 
                                const double s, const VectorXd& x, MatrixXd& lfv, MatrixXd& lfdv, const Vector2d& sw2CoM_end_step_des)
{

    // Parameters
    const double H = srb_params.hLIP;
    const double zcl = srb_params.zcl;
    const double ft_l = srb_params.foot_length;

    // Planner info
    const double sw2CoM_beg_step_x = traj_planner_dyn_data.sw2CoM_beg_step(0) + 0.5*ft_l;
    const double sw2CoM_beg_step_y = traj_planner_dyn_data.sw2CoM_beg_step(1);
    const double sw2CoM_beg_step_z = traj_planner_dyn_data.sw2CoM_beg_step(2);

    // Get SRB states
    const Vector3d pc = x.head<3>();

    // cout << "pc: " << pc.transpose() << endl;

    // Calculate the center of the right feet positions
    Eigen::Vector3d right_center = lfv.topRows(2).colwise().mean();

    // Calculate the center of the left feet positions
    Eigen::Vector3d left_center = lfv.bottomRows(2).colwise().mean();

    // Compute the new positions for each foot to align them properly
    Eigen::MatrixXd lfv_straight = lfv;
    lfv_straight.topRows(2).col(1).setConstant(right_center(1)); // Set y value for right feet
    lfv_straight.bottomRows(2).col(1).setConstant(left_center(1)); // Set y value for left feet
    lfv_straight(0,0) = (right_center(0) + 0.06 ); // Adjust x value for right feet
    lfv_straight(1,0) = (right_center(0) - 0.06 ); // Adjust x value for right feet
    lfv_straight(2,0) = (left_center(0)  + 0.06 ); // Adjust x value for left feet
    lfv_straight(3,0) = (left_center(0)  - 0.06 ); // Adjust x value for left feet

    lfv_comm = lfv_straight;//lfv_dsp_start;

    lfdv_comm.setZero(); // lfdv;
    lfddv_comm.setZero();

    double beg_step_z = traj_planner_dyn_data.sw_beg_step(2);

    // Swing-leg trajectories
    if (abs(FSM) == 1) // SSP
    {
        const VectorXd sw2CoM_traj_x = dash_utils::sw_leg_ref_xy(s, sw2CoM_beg_step_x, sw2CoM_end_step_des(0));
        const VectorXd sw2CoM_traj_y = dash_utils::sw_leg_ref_xy(s, sw2CoM_beg_step_y, sw2CoM_end_step_des(1));
        // const VectorXd sw2CoM_traj_z = dash_utils::sw_leg_ref_z_v2(s, beg_step_z, zcl);
        const VectorXd sw2CoM_traj_z = dash_utils::sw_leg_ref_z(s, zcl, H);

        // cout << "s: " << s << "\t H: " << H << "\t zcl: " << zcl << endl;
        
        if (FSM == 1) // SSP_L
        {
            // x-position trajectories
            lfv_comm(0, 0) = pc(0) - (sw2CoM_traj_x(0) - (1.0/2.0)*ft_l); lfv_comm(1, 0) = pc(0) - (sw2CoM_traj_x(0) + (1.0/2.0)*ft_l);
            // lfv_comm(0, 0) = pc(0) - (0 - (1.0/2.0)*ft_l); lfv_comm(1, 0) = pc(0) - (0 + (1.0/2.0)*ft_l);
            lfdv_comm(0, 0) = sw2CoM_traj_x(1); lfdv_comm(1, 0) = lfdv_comm(0, 0);
            lfddv_comm(0, 0) = sw2CoM_traj_x(2); lfddv_comm(1, 0) = lfddv_comm(0, 0);
            // y-position trajectories
            lfv_comm(0, 1) = pc(1) - sw2CoM_traj_y(0); lfv_comm(1, 1) = lfv_comm(0, 1);
            lfdv_comm(0, 1) = sw2CoM_traj_y(1); lfdv_comm(1, 1) = lfdv_comm(0, 1);
            lfddv_comm(0, 1) = sw2CoM_traj_y(2); lfddv_comm(1, 1) = lfddv_comm(0, 1);
            // z-position trajectories
            lfv_comm(0, 2) = pc(2) - sw2CoM_traj_z(0); lfv_comm(1, 2) = lfv_comm(0, 2);
            lfdv_comm(0, 2) = sw2CoM_traj_z(1); lfdv_comm(1, 2) = lfdv_comm(0, 2);
            lfddv_comm(0, 2) = sw2CoM_traj_z(2); lfddv_comm(1, 2) = lfddv_comm(0, 2);
            // cout << "traj_z(1): " << sw2CoM_traj_z(0) << endl;

        }
        else if (FSM == -1) // SSP_R
        {
            // x-position trajectories
            lfv_comm(2, 0) = pc(0) - (sw2CoM_traj_x(0) - (1.0/2.0)*ft_l); lfv_comm(3, 0) = pc(0) - (sw2CoM_traj_x(0) + (1.0/2.0)*ft_l);
            // lfv_comm(2, 0) = pc(0) - (0 - (1.0/2.0)*ft_l); lfv_comm(3, 0) = pc(0) - (0 + (1.0/2.0)*ft_l);
            lfdv_comm(2, 0) = sw2CoM_traj_x(1); lfdv_comm(3, 0) = lfdv_comm(2, 0);   
            lfddv_comm(2, 0) = sw2CoM_traj_x(2); lfddv_comm(3, 0) = lfddv_comm(2, 0);        
            // y-position trajectories
            lfv_comm(2, 1) = pc(1) - sw2CoM_traj_y(0); lfv_comm(3, 1) = lfv_comm(2, 1);
            lfdv_comm(2, 1) = sw2CoM_traj_y(1); lfdv_comm(3, 1) = lfdv_comm(2, 1);
            lfddv_comm(2, 1) = sw2CoM_traj_y(2); lfddv_comm(3, 1) = lfddv_comm(2, 1);
            // z-position trajectories
            lfv_comm(2, 2) = pc(2) - sw2CoM_traj_z(0); lfv_comm(3, 2) = lfv_comm(2, 2);
            lfdv_comm(2, 2) = sw2CoM_traj_z(1); lfdv_comm(3, 2) = lfdv_comm(2, 2);
            lfddv_comm(2, 2) = sw2CoM_traj_z(2); lfddv_comm(3, 2) = lfddv_comm(2, 2);
            // cout << "traj_z:(-1) " << sw2CoM_traj_z(0) << endl;

        }
    }

}

void dash_ctrl::sw_teleop_step_strategy(MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, const SRB_Params srb_params, const Human_params human_params, 
                            const Traj_planner_dyn_data& traj_planner_dyn_data, const Human_dyn_data& human_dyn_data, 
                            const int FSM, const double s, const double swxf, MatrixXd& lfv, MatrixXd& lfdv, VectorXd x) 
{

    // Swing-leg trajectory generation for teleoperation framework
    // Assumes the following:
    // x-direction final step placement is given based on dynamic
    // similarity between HWRM and robot
    // y-direction final step placement is based on maintaining normalized feet
    // width between robot and human
    // z-direction trajectory is based on human z-position serving as amplitude
    // of curve
    // this will fix step time because robot will touchdown at pre-determined
    // time, human should feel haptic force and step-down as well

    // Parameters (robot)
    const double hR = srb_params.hLIP;
    const double ft_l = srb_params.foot_length;

    const Vector3d pc = x.head<3>();

    // Parameters (human)
    const double hH = human_params.hLIP;    
    const double joystick_base_separation = 1.525;
    const double foot_center_to_joystick = FOOT_2_JOYSTICK;

    // Planner info
    const double sw_beg_step_x = traj_planner_dyn_data.sw_beg_step(0);
    const double sw_beg_step_y = traj_planner_dyn_data.sw_beg_step(1);
    const double sw_beg_step_z = traj_planner_dyn_data.sw_beg_step(2);   
    const double st_beg_step_y = traj_planner_dyn_data.st_beg_step(1);  
    const double fzH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step(2);

    // cout << "x: " << sw_beg_step_x << "   y: " << sw_beg_step_y << "   z: " << sw_beg_step_z << endl;

    // Human info (leg-joystick data)
    const double fyH_R = human_dyn_data.fyH_R;
    const double fzH_R = human_dyn_data.fzH_R;
    const double fyH_L = human_dyn_data.fyH_L;
    const double fzH_L = human_dyn_data.fzH_L;    

    // Initialize commanded end-effector positions (DSP)
    // lfv_comm = lfv_dsp_start; // lfv;
    // lfv_comm.col(2).setConstant(-srb_params.hLIP);
    // lfdv_comm.setZero(); // lfdv   
    // lfddv_comm.setZero();  

    VectorXd sw_traj_x(3); VectorXd sw_traj_y(3); VectorXd sw_traj_z(3);
    double swyf, AH;

    

    // Swing-leg trajectories
    if (abs(FSM) == 1) // SSP
    {

        // robot target foot width calculation (y-direction swing-leg end-of-step calculation)
        double human_foot_width = joystick_base_separation - (2.0 * foot_center_to_joystick) - fyH_R - fyH_L;
        double robot_target_foot_width = human_foot_width * (hR / hH);

        // initialize swing-leg trajectories
        // VectorXd sw_traj_x(3); VectorXd sw_traj_y(3); VectorXd sw_traj_z(3);
        
        // compute swing-leg trajectories
        if (FSM == 1) // SSP_L
        {
            // x-position trajectories
            sw_traj_x = dash_utils::sw_leg_ref_xy(s, sw_beg_step_x, swxf);
            lfv_comm(0, 0) = sw_traj_x(0) + (1.0/2.0) * ft_l; lfv_comm(1, 0) = sw_traj_x(0) - (1.0/2.0) * ft_l;
            lfdv_comm(0, 0) = sw_traj_x(1); lfdv_comm(1, 0) = lfdv_comm(0, 0);
            lfddv_comm(0, 0) = sw_traj_x(2); lfddv_comm(1, 0) = lfddv_comm(0, 0);

            // y-position trajectories
            swyf = st_beg_step_y - robot_target_foot_width;
            sw_traj_y = dash_utils::sw_leg_ref_xy(s, sw_beg_step_y, swyf);
            lfv_comm(0, 1) = sw_traj_y(0); lfv_comm(1, 1) = lfv_comm(0, 1);
            lfdv_comm(0, 1) = sw_traj_y(1); lfdv_comm(1, 1) = lfdv_comm(0, 1);
            lfddv_comm(0, 1) = sw_traj_y(2); lfddv_comm(1, 1) = lfddv_comm(0, 1);

            // z-position trajectories
            AH = (hR / hH) * (std::max(0.0, (fzH_R - fzH0)));
            AH = std::max(traj_planner_dyn_data.AH_step_predicted*(hR / hH),0.03);
            sw_traj_z = dash_utils::sw_leg_ref_z_v2(s, sw_beg_step_z, AH);
            // sw_traj_z = dash_utils::sw_leg_ref_z(s, AH,hR);
            lfv_comm(0, 2) = sw_traj_z(0); lfv_comm(1, 2) = lfv_comm(0, 2);
            // lfv_comm(0, 2) = pc(2) -  sw_traj_z(0); lfv_comm(1, 2) = lfv_comm(0, 2);
            lfdv_comm(0, 2) = sw_traj_z(1); lfdv_comm(1, 2) = lfdv_comm(0, 2);
            lfddv_comm(0, 2) = sw_traj_z(2); lfddv_comm(1, 2) = lfddv_comm(0, 2);
            // cout << sw_traj_z(1) << endl;

        }
        else if (FSM == -1) // SSP_R
        {
            // x-position trajectories
            sw_traj_x = dash_utils::sw_leg_ref_xy(s, sw_beg_step_x, swxf);
            lfv_comm(2, 0) = sw_traj_x(0) + (1.0/2.0) * ft_l; lfv_comm(3, 0) = sw_traj_x(0) - (1.0/2.0) * ft_l;
            lfdv_comm(2, 0) = sw_traj_x(1); lfdv_comm(3, 0) = lfdv_comm(2, 0);
            lfddv_comm(2, 0) = sw_traj_x(2); lfddv_comm(3, 0) = lfddv_comm(2, 0);       
               
            // y-position trajectories
            swyf = st_beg_step_y + robot_target_foot_width;
            sw_traj_y = dash_utils::sw_leg_ref_xy(s, sw_beg_step_y, swyf);
            lfv_comm(2, 1) = sw_traj_y(0); lfv_comm(3, 1) = lfv_comm(2, 1);
            lfdv_comm(2, 1) = sw_traj_y(1); lfdv_comm(3, 1) = lfdv_comm(2, 1);
            lfddv_comm(2, 1) = sw_traj_y(2); lfddv_comm(3, 1) = lfddv_comm(2, 1);

            // z-position trajectories
            AH = (hR / hH) * (std::max(0.0, (fzH_L - fzH0)));
            AH = std::max(traj_planner_dyn_data.AH_step_predicted*(hR / hH),0.03);
            sw_traj_z = dash_utils::sw_leg_ref_z_v2(s, sw_beg_step_z, AH);
            // sw_traj_z = dash_utils::sw_leg_ref_z(s, AH,hR);
            lfv_comm(2, 2) = sw_traj_z(0); lfv_comm(3, 2) = lfv_comm(2, 2);
            // lfv_comm(2, 2) = pc(2) - sw_traj_z(0); lfv_comm(3, 2) = lfv_comm(2, 2);
            lfdv_comm(2, 2) = sw_traj_z(1); lfdv_comm(3, 2) = lfdv_comm(2, 2);
            lfddv_comm(2, 2) = sw_traj_z(2); lfddv_comm(3, 2) = lfddv_comm(2, 2);

        }
        
    }
    // cout << "s: " << s << "   AH: " << AH << "   (fzH_R - fzH0): " << (fzH_R - fzH0) << "   fzH_R: " << fzH_R << "   fzH0: " << fzH0 << "   (fzH_L - fzH0): " << (fzH_L - fzH0) << "   fzH_L: " << fzH_L << endl;

}                             

void dash_ctrl::SRB_Balance_Controller(VectorXd& u, VectorXd& tau_legs, SRB_Params srb_params, int FSM, VectorXd x, MatrixXd lfv, MatrixXd qd, MatrixXd* Jv_mat, VectorXd u0, VectorXd SRB_wrench_ref)
{
    u = SRB_force_distribution_QP(srb_params, FSM, x, lfv, qd, Jv_mat, u0, SRB_wrench_ref);
    tau_legs = calc_joint_torques(x, Jv_mat, u);
}

// convert eigen vector to real_t vector
void VectorXd2real_t_vec(VectorXd vec, real_t* vec_realt)
{
    // number of elements
    int const num_elements = vec.size();

    // convert vector
    int i;
    for (i = 0; i < num_elements; i++) {
        vec_realt[i] = vec(i);
    }

}

// convert eigen matrix to real_t vector
void MatrixXd2real_t_vec(MatrixXd M, real_t* vec_realt)
{
    // number of rows and columns
    int const num_rows = M.rows();
    int const num_cols = M.cols();

    // create row-major vector
    int i, j;
    int idx = 0;
    for (i = 0; i < num_rows; i++) {
        for (j = 0; j < num_cols; j++) {
            vec_realt[idx] = M(i, j);
            idx++;
        }
    }

}

real_t* H_realt;
real_t* f_realt;;
real_t* A_realt;;
real_t* lb_realt;
real_t* ub_realt;;
real_t* Aeq_realt;
real_t* beq_realt; 
real_t* lbA_realt;
real_t* ubA_realt; 
bool real_allocated = false;

// resize optimization matrices
void resize_qp_mats(int num_opt_vars, int num_constraints)
{
    // free up memory for size change
    if(real_allocated == true) {

        free(H_realt);
        free(f_realt);
        free(A_realt);
        free(lb_realt);
        free(ub_realt);
        // free(Aeq_realt);
        // free(beq_realt);   
        free(lbA_realt);
        free(ubA_realt);       
    
    }

    // allocate new memory
    H_realt = (real_t*)malloc(num_opt_vars*num_opt_vars*sizeof(real_t));
    f_realt = (real_t*)malloc(num_opt_vars*sizeof(real_t));
    A_realt = (real_t*)malloc(num_constraints*num_opt_vars*sizeof(real_t));
    lb_realt = (real_t*)malloc(num_opt_vars*sizeof(real_t));
    ub_realt = (real_t*)malloc(num_opt_vars*sizeof(real_t));
    // Aeq_realt = (real_t*)malloc(num_constraints*sizeof(real_t));
    // beq_realt = (real_t*)malloc(num_constraints*sizeof(real_t));  
    ubA_realt = (real_t*)malloc(num_constraints*sizeof(real_t));
    lbA_realt = (real_t*)malloc(num_constraints*sizeof(real_t)); 

    // set flag
    real_allocated == true;  
 
}

VectorXd dash_ctrl::SRB_force_distribution_QP(SRB_Params srb_params,int FSM,VectorXd x,MatrixXd lfv,MatrixXd qd,MatrixXd* Jv_mat,VectorXd u0,VectorXd tau_SRB_des)
{
    // Parameters
    int QP_opt_sol_type = srb_params.QP_opt_sol_type;
    double W_wrench = srb_params.W_wrench;
    double W_u_minus_u0_norm = srb_params.W_u_minus_u0_norm;
    Eigen::VectorXi uix_idx(4);
    uix_idx << 0, 3, 6, 9;
    Eigen::VectorXi uiy_idx(4);
    uiy_idx << 1, 4, 7, 10;
    Eigen::VectorXi uiz_idx(4);
    uiz_idx << 2, 5, 8, 11;
    int num_end_effector_pts = 4;
    int position_vec_size = 3;
    
    // get SRB states
    Vector3d pc = x.segment<3>(0);

    // calculate position vectors from end-effector (s) to CoM
    MatrixXd r_mat(position_vec_size, num_end_effector_pts);
    for (int i = 0; i < num_end_effector_pts; i++) {
        r_mat = lfv.transpose() - pc.replicate(1,num_end_effector_pts);
    }
    
    // get rx, ry, and rz for each end-effector
    VectorXd rx = r_mat.row(0);
    VectorXd ry = r_mat.row(1);
    VectorXd rz = r_mat.row(2);
    
    // get desired net wrench component values
    double FxR = tau_SRB_des(0);
    double FyR = tau_SRB_des(1);
    double FzR = tau_SRB_des(2);
    double MxR = tau_SRB_des(3);
    double MyR = tau_SRB_des(4);
    double MzR = tau_SRB_des(5);
    
    // cost function: min_x (1/2)*x^T*H*x + f^T*x
    VectorXd dummy1 = VectorXd::Zero(4);
    VectorXd dummy2 = VectorXd::Zero(4);
    VectorXd dummy3 = VectorXd::Zero(4);
    VectorXd four_ones = VectorXd::Ones(4).transpose();
    // get individual contribution from each wrench component and penalize large
    // internal forces
    MatrixXd Fx_Q, Fy_Q, Fz_Q, Mx_Q, My_Q, Mz_Q, Q_intF_x, Q_intF_y, Q_intF_z;
    VectorXd Fx_f, Fy_f, Fz_f, Mx_f, My_f, Mz_f;
    cost_quadratic_coeff_matrices(four_ones, FxR, uix_idx, 4, Fx_Q, Fx_f); // forces in the x-direction
    cost_quadratic_coeff_matrices(four_ones, FyR, uiy_idx, 4, Fy_Q, Fy_f); // forces in the y-direction
    cost_quadratic_coeff_matrices(four_ones, FzR, uiz_idx, 4, Fz_Q, Fz_f); // forces in the z-direction
    cost_quadratic_coeff_matrices((VectorXd(8) << -1.0*rz, ry).finished(), MxR, (Eigen::VectorXi(8) << uiy_idx, uiz_idx).finished(), 8, Mx_Q, Mx_f); // moments in the x-direction
    cost_quadratic_coeff_matrices((VectorXd(8) << rz, -1.0*rx).finished(), MyR, (Eigen::VectorXi(8) << uix_idx, uiz_idx).finished(), 8, My_Q, My_f); // moments in the y-direction
    cost_quadratic_coeff_matrices((VectorXd(8) << -1.0*ry, rx).finished(), MzR, (Eigen::VectorXi(8) << uix_idx, uiy_idx).finished(), 8, Mz_Q, Mz_f); // moments in the z-direction


    
    // Calculate overall cost function with weights
    MatrixXd Q_wrench = Fx_Q + Fy_Q + Fz_Q + Mx_Q + My_Q + Mz_Q;
    MatrixXd Q_u_minus_u0_norm = Eigen::MatrixXd::Identity(12, 12);
    VectorXd f_wrench = Fx_f + Fy_f + Fz_f + Mx_f + My_f + Mz_f;
    VectorXd f_u_minus_u0_norm = -2.0*u0;
    MatrixXd Q = W_wrench*Q_wrench + W_u_minus_u0_norm*Q_u_minus_u0_norm;
    MatrixXd H = 2.0*Q;
    VectorXd f = W_wrench*f_wrench + W_u_minus_u0_norm*f_u_minus_u0_norm;

    // get QP linear constraints (inequality, equality, upper and lower bounds)
    MatrixXd Aeq, LB, UB, A;
    VectorXd b, beq;
    SRB_force_distribution_QP_constraints(A, b, Aeq, beq, LB, UB, srb_params, FSM, x, qd, Jv_mat);

    
    
    // solve QP to get GRFs

    const int nVar = H.rows();  // number of variables
    const int nCons = A.rows(); // number of constraints
    
    // Define options
    // qpOASES function call
    if(first_time_running_qp){
        GRFs_distribution_QP = QProblem(nVar, nCons, qpOASES::HST_POSDEF);
    
        Options options;
        options.setToReliable();
        options.printLevel = PL_NONE;
        GRFs_distribution_QP.setOptions(options);

    }

    MatrixXd A1; VectorXd lbA; VectorXd ubA;   
    lbA = -1000*VectorXd::Ones(b.size());

    // DEBUGGING
    // dash_utils::setOutputFolder("../../../Outputs/Debug/");
    // dash_utils::writeMatrixToCsv(H, "H.csv");
    // dash_utils::writeVectorToCsv(f, "f.csv");
    // dash_utils::writeMatrixToCsv(A, "A.csv");
    // dash_utils::writeMatrixToCsv(lbA, "lbA.csv");
    // dash_utils::writeVectorToCsv(b, "b.csv");
    // dash_utils::writeMatrixToCsv(LB, "LB.csv");
    // dash_utils::writeMatrixToCsv(UB, "UB.csv");
    // END DEBUGGING
    
    // Initialize arrays
    resize_qp_mats(nVar, nCons);
    
    MatrixXd2real_t_vec(H, H_realt);
    VectorXd2real_t_vec(f, f_realt);
    MatrixXd2real_t_vec(A, A_realt);
    MatrixXd2real_t_vec(lbA, lbA_realt); 
    VectorXd2real_t_vec(b, ubA_realt);
    MatrixXd2real_t_vec(LB, lb_realt); 
    MatrixXd2real_t_vec(UB, ub_realt); 
    
    int_t nWSR = 1000;     
    GRFs_distribution_QP.init(H_realt, f_realt, A_realt, lb_realt, ub_realt, lbA_realt, ubA_realt, nWSR); 
    GRFs_distribution_QP.getPrimalSolution(xOpt); 

    // convert solution
    VectorXd xOpt_VectorXd = VectorXd::Zero(nVar);
    int xOpt_comp_idx;
    for (xOpt_comp_idx = 0; xOpt_comp_idx < nVar; xOpt_comp_idx++) {
        xOpt_VectorXd(xOpt_comp_idx) = xOpt[xOpt_comp_idx];  
       //printf("%f, ",xOpt_VectorXd(xOpt_comp_idx));
    } 
    //printf("\n");
    // dash_utils::writeVectorToCsv(xOpt_VectorXd, "u.csv");
    // exit(0);
    return xOpt_VectorXd;
}

VectorXd dash_ctrl::calc_joint_torques(VectorXd x, MatrixXd* Jv_mat, VectorXd GRFs)
{
    // Calculate joint torques for each leg
    
    // get SRB states
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Matrix3d Rbw = R_curr.transpose();
    
    // JvT for each end-effector 
    MatrixXd JvT_contact_lf1R = Jv_mat[0].transpose();
    MatrixXd JvT_contact_lf2R = Jv_mat[1].transpose();
    MatrixXd JvT_contact_lf3L = Jv_mat[2].transpose();
    MatrixXd JvT_contact_lf4L = Jv_mat[3].transpose();
    
    // desired end-effector forces are opposite of ground reaction forces
    VectorXd GRF_opp = -GRFs;
    
    // get ground reaction forces (in world frame) for each contact point
    Vector3d f1 = GRF_opp.segment(0, 3);
    Vector3d f2 = GRF_opp.segment(3, 3);
    Vector3d f3 = GRF_opp.segment(6, 3);
    Vector3d f4 = GRF_opp.segment(9, 3);
    
    // convert GRFs (in world frame) to body frame
    Vector3d f1_b = Rbw * f1;
    Vector3d f2_b = Rbw * f2;
    Vector3d f3_b = Rbw * f3;
    Vector3d f4_b = Rbw * f4;
    
    // calculate joint torques for each leg
    VectorXd tau_leg_r = JvT_contact_lf1R * f1_b + JvT_contact_lf2R * f2_b;
    VectorXd tau_leg_l = JvT_contact_lf3L * f3_b + JvT_contact_lf4L * f4_b;
    
    // joint torques
    VectorXd tau_legs(10);
    tau_legs << tau_leg_r, tau_leg_l;
    
    return tau_legs;
}

void dash_ctrl::cost_quadratic_coeff_matrices(const VectorXd coeff, double C, const Eigen::VectorXi& idx, int num_opt_vars_act, MatrixXd& Q, VectorXd& f) 
{
    // initialize Q and f matrices
    Q = MatrixXd::Zero(12, 12);
    f = VectorXd::Zero(12);

    // update and Q and f matrices
    for (int col_idx = 0; col_idx < num_opt_vars_act; col_idx++) {
        // update f vector
        f(idx(col_idx)) = -2.0 * C * coeff(col_idx);
        for (int row_idx = 0; row_idx < num_opt_vars_act; row_idx++) {
            // update Q matrix
            Q(idx(row_idx), idx(col_idx)) = coeff(row_idx) * coeff(col_idx);
        }
    }
}

void dash_ctrl::SRB_force_distribution_QP_constraints(MatrixXd &A, VectorXd &b, MatrixXd &Aeq, VectorXd &beq, MatrixXd &LB, MatrixXd &UB, const SRB_Params &srb_params, const int FSM, const VectorXd &x, const MatrixXd &qd, const MatrixXd* Jv_mat) {
    // Compute linear constraints for force distribution QP
    // Constraints based on kinematic and actuation limits
    // Also need to satisfy friction cone

    // number of optimization variables = 12

    // inequality constraints: A*x <= b
    
    // friction cone constraints
    MatrixXd A_GRF_fric_nopull;
    VectorXd ubA_GRF_fric_nopull;
    int num_const_fric;
    friction_cone_constraints(srb_params, A_GRF_fric_nopull, ubA_GRF_fric_nopull, num_const_fric);
    
    // parallel actuation constraints
    // currently joint torque based constraints (default) are more stable than motor torque based constraints!!!
    MatrixXd A_GRF_torque_lim;
    VectorXd ubA_GRF_torque_lim;
    int num_const_parallel_act;
    parallel_actuation_constraints(srb_params, x, qd, Jv_mat, A_GRF_torque_lim, ubA_GRF_torque_lim, num_const_parallel_act);
    
    // total number of constaints
    int num_const_total = num_const_fric + num_const_parallel_act;

    // Combine all inequality constraints
    A.resize(num_const_total, 12);
    b.resize(num_const_total);
    A.topRows(num_const_fric) = A_GRF_fric_nopull;
    b.head(num_const_fric) = ubA_GRF_fric_nopull;
    A.bottomRows(num_const_parallel_act) = A_GRF_torque_lim;
    b.tail(num_const_parallel_act) = ubA_GRF_torque_lim;
    
    // equality constraints: Aeq*x = beq
    Aeq.resize(0,0);
    beq.resize(0);
    double grf_max = 1000;
    // upper and lower bounds: LB <= x <= UB
    LB = MatrixXd::Zero(12,1);
    UB = MatrixXd::Zero(12,1);
    if (FSM == 1) { // SSP_L
        // contact points 3 & 4 are active
        LB.block(6,0,6,1).setConstant(-grf_max);
        UB.block(6,0,6,1).setConstant(grf_max);
    } else if (FSM == -1) { // SSP_R
        // contact points 1 & 2 are active
        LB.block(0,0,6,1).setConstant(-grf_max);
        UB.block(0,0,6,1).setConstant(grf_max);
    } else { // FSM == 0 -- DSP
        // all contact points are active
        LB.setConstant(-grf_max);
        UB.setConstant(grf_max);
    }
    
}

void dash_ctrl::parallel_actuation_constraints(SRB_Params srb_params,const VectorXd& x,const MatrixXd& qd,const MatrixXd* Jv_mat,
                                    MatrixXd& A_GRF_torque_lim,VectorXd& ubA_GRF_torque_lim,int& num_const_total)
{
    int Act_const_type = srb_params.Act_const_type;
    double tau_m_max = srb_params.tau_m_max;
    double tau_m_stall = srb_params.tau_m_stall;
    double alpha_m = srb_params.alpha_m;
    double beta_trans = srb_params.beta_trans;
    double gamma_trans = srb_params.gamma_trans;
    int num_legs = 2;
    int num_GRFs = 4;
    int num_joints_per_leg = 5;
    int num_motors_per_leg = 5;
    
    // number of optimization variables
    int num_opt_variables = num_GRFs * 3;

    // total number of motors
    int num_motors = num_legs * num_motors_per_leg;

    // get SRB states
    Matrix3d Rbw = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(x.segment<9>(6).data()).transpose();
    
    // JvT for each end-effector
    MatrixXd JvT_contact_lf1R = Jv_mat[0].transpose();
    MatrixXd JvT_contact_lf2R = Jv_mat[1].transpose();
    MatrixXd JvT_contact_lf1L = Jv_mat[2].transpose();
    MatrixXd JvT_contact_lf2L = Jv_mat[3].transpose();

    // Define topology Jacobian for transmission dynamics
    MatrixXd J_topology(5, 5);
    J_topology << 1.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, beta_trans, -beta_trans, 0.0,          0.0,
                   0.0,        0.5,         0.5, 0.0,          0.0,
                   0.0,        0.0,         0.0, 0.5,  gamma_trans,
                   0.0,        0.0,         0.0, 0.5, -gamma_trans;
    
    // JvT_contact_Rbw calculation for each end-effector
    MatrixXd JvT_contact_Rbw_lf1R = JvT_contact_lf1R * Rbw;
    MatrixXd JvT_contact_Rbw_lf2R = JvT_contact_lf2R * Rbw;
    MatrixXd JvT_contact_Rbw_lf1L = JvT_contact_lf1L * Rbw;
    MatrixXd JvT_contact_Rbw_lf2L = JvT_contact_lf2L * Rbw;

    // matrix with JvT_contact_Rbw matrices (maps GRFs to joint torques)
    MatrixXd JvT_contact_Rbw_mat = MatrixXd::Zero(num_legs*num_joints_per_leg, num_opt_variables);
    JvT_contact_Rbw_mat.block(0, 0, 5, 3) = JvT_contact_Rbw_lf1R;
    JvT_contact_Rbw_mat.block(0, 3, 5, 3) = JvT_contact_Rbw_lf2R;
    JvT_contact_Rbw_mat.block(5, 6, 5, 3) = JvT_contact_Rbw_lf1L;
    JvT_contact_Rbw_mat.block(5, 9, 5, 3) = JvT_contact_Rbw_lf2L;
    
    MatrixXd tau_j_comp = -1.0 * JvT_contact_Rbw_mat;

    MatrixXd JT_topology = J_topology.transpose();
    MatrixXd Jinv_topology = J_topology.inverse();
    
    MatrixXd JT_topology_JvT_contact_Rbw_mat = MatrixXd::Zero(num_legs*num_joints_per_leg, num_opt_variables);
    JT_topology_JvT_contact_Rbw_mat.block(0, 0, 5, 3) = JT_topology * JvT_contact_Rbw_lf1R;
    JT_topology_JvT_contact_Rbw_mat.block(0, 3, 5, 3) = JT_topology * JvT_contact_Rbw_lf2R;
    JT_topology_JvT_contact_Rbw_mat.block(5, 6, 5, 3) = JT_topology * JvT_contact_Rbw_lf1L;
    JT_topology_JvT_contact_Rbw_mat.block(5, 9, 5, 3) = JT_topology * JvT_contact_Rbw_lf2L;
    
    MatrixXd omega_m_temp = MatrixXd::Zero(10, 10);
    omega_m_temp.block(0,0,5,5) = Jinv_topology;
    omega_m_temp.block(0,5,5,5) = MatrixXd::Zero(5, 5);
    omega_m_temp.block(5,0,5,5) = MatrixXd::Zero(5, 5);
    omega_m_temp.block(5,5,5,5) = Jinv_topology;
    VectorXd qdVec(10);
    qdVec << qd.row(0).transpose(), qd.row(1).transpose();
    VectorXd omega_m = omega_m_temp * qdVec;
    
    MatrixXd tau_m_comp = -1.0 * JT_topology_JvT_contact_Rbw_mat;

    if (Act_const_type == 0) {

        // parallel actuation torque based limits
        // |tau1| <= tau_m_max, |tau2 + tau3| <= 2*tau_m_max, |tau4 + tau5| <= 2*tau_m_max

        // initialize constraints
        int num_const_per_leg = 10;
        num_const_total = num_const_per_leg*num_legs;
        A_GRF_torque_lim = MatrixXd::Zero(num_const_total, num_opt_variables);

        ubA_GRF_torque_lim = VectorXd::Zero(num_const_total);

        // Right Leg

        // hip yaw motor torque constraint (right leg) -- |tau1| <= tau_m_max
        A_GRF_torque_lim.row(0) = tau_j_comp.row(0);
        A_GRF_torque_lim.row(1) = -1.0*tau_j_comp.row(0);
        ubA_GRF_torque_lim.segment(0,2) = tau_m_max*Eigen::VectorXd::Ones(2);
        // hip roll and hip pitch motor torques constraint (right leg) -- |tau2 + tau3| <= 2*tau_m_max
        A_GRF_torque_lim.row(2) = tau_j_comp.row(1) + tau_j_comp.row(2);
        A_GRF_torque_lim.row(3) = tau_j_comp.row(1) - tau_j_comp.row(2);
        A_GRF_torque_lim.row(4) = -1.0*tau_j_comp.row(1) + tau_j_comp.row(2);
        A_GRF_torque_lim.row(5) = -1.0*tau_j_comp.row(1) - tau_j_comp.row(2);
        ubA_GRF_torque_lim.segment(2,4) = 2.0*tau_m_max*Eigen::VectorXd::Ones(4);
        // knee pitch and ankle pitch motor torques constraint (right leg) -- |tau4 + tau5| <= 2*tau_m_max
        A_GRF_torque_lim.row(6) = tau_j_comp.row(3) + tau_j_comp.row(4);
        A_GRF_torque_lim.row(7) = tau_j_comp.row(3) - tau_j_comp.row(4);
        A_GRF_torque_lim.row(8) = -1.0*tau_j_comp.row(3) + tau_j_comp.row(4);
        A_GRF_torque_lim.row(9) = -1.0*tau_j_comp.row(3) - tau_j_comp.row(4);
        ubA_GRF_torque_lim.segment(6,4) = 2*tau_m_max*Eigen::VectorXd::Ones(4);

        // Left Leg
        // hip yaw motor torque constraint (left leg) -- |tau1| <= tau_m_max
        A_GRF_torque_lim.row(10) = tau_j_comp.row(5);
        A_GRF_torque_lim.row(11) = -1.0*tau_j_comp.row(5);
        ubA_GRF_torque_lim.segment(10,2) = tau_m_max*Eigen::VectorXd::Ones(2);
        // hip roll and hip pitch motor torques constraint (left leg) -- |tau2 + tau3| <= 2*tau_m_max
        A_GRF_torque_lim.row(12) = tau_j_comp.row(6) + tau_j_comp.row(7);
        A_GRF_torque_lim.row(13) = tau_j_comp.row(6) - tau_j_comp.row(7);
        A_GRF_torque_lim.row(14) = -1.0*tau_j_comp.row(6) + tau_j_comp.row(7);
        A_GRF_torque_lim.row(15) = -1.0*tau_j_comp.row(6) - tau_j_comp.row(7);
        ubA_GRF_torque_lim.segment(12,4) = 2.0*tau_m_max*VectorXd::Ones(4);

        // knee pitch and ankle pitch motor torques constraint (left leg) -- |tau4 + tau5| <= 2*tau_m_max
        A_GRF_torque_lim.row(16) = tau_j_comp.row(8) + tau_j_comp.row(9);
        A_GRF_torque_lim.row(17) = tau_j_comp.row(8) - tau_j_comp.row(9);
        A_GRF_torque_lim.row(18) = -1.0*tau_j_comp.row(8) + tau_j_comp.row(9);
        A_GRF_torque_lim.row(19) = -1.0*tau_j_comp.row(8) - tau_j_comp.row(9);
        ubA_GRF_torque_lim.segment(16,4) = 2.0*tau_m_max*VectorXd::Ones(4);
    }
    else if (Act_const_type == 1) {

        // parallel actuation torque based limits
        // tau_mi + alpha*omege_mi - tau_m_stall <= 0
        // tau_mi - tau_m_max <= 0
        // -tau_mi - alpha*omege_mi - tau_m_stall <= 0
        // -tau_mi - tau_m_max <= 0    

        // initialize constraints
        int num_const_per_motor = 4;
        num_const_total = num_const_per_motor*num_motors;
        A_GRF_torque_lim = MatrixXd::Zero(num_const_total, num_opt_variables);

        ubA_GRF_torque_lim = VectorXd::Zero(num_const_total);

        // loop through each motor and implement constraints (covers both legs)
        for (int mot_idx = 0; mot_idx < num_motors; mot_idx++) {
            // implement motor constraints
            A_GRF_torque_lim.block(4*mot_idx, 0, 1, num_opt_variables) = tau_m_comp.row(mot_idx);

            ubA_GRF_torque_lim(4*mot_idx) = tau_m_stall - alpha_m*omega_m(mot_idx);

            A_GRF_torque_lim.block(4*mot_idx+1, 0, 1, num_opt_variables) = tau_m_comp.row(mot_idx);

            ubA_GRF_torque_lim(4*mot_idx+1) = tau_m_max;

            A_GRF_torque_lim.block(4*mot_idx+2, 0, 1, num_opt_variables) = -1.0*tau_m_comp.row(mot_idx);

            ubA_GRF_torque_lim(4*mot_idx+2) = tau_m_stall + alpha_m*omega_m(mot_idx);

            A_GRF_torque_lim.block(4*mot_idx+3, 0, 1, num_opt_variables) = -1.0*tau_m_comp.row(mot_idx);

            ubA_GRF_torque_lim(4*mot_idx+3) = tau_m_max;
        }
    }

}


void dash_ctrl::friction_cone_constraints(SRB_Params srb_params, MatrixXd& A_GRF_fric_nopull, VectorXd& ubA_GRF_fric_nopull, int& num_const_total)
{
    // Parameters
    double mu = srb_params.mu;
    double Fz_min_QP = srb_params.Fz_min_QP;
    int num_GRFs = 4;
    int num_const_per_GRF = 5;
    int num_opt_variables = num_GRFs*3;
    // total number of constraints added
    num_const_total = num_GRFs * num_const_per_GRF;

    // inequality constraint A matrix and b vector for one GRF
    // 5 total constraints to enfore friction pyramid and no pulling on the
    // ground per GRF
    A_GRF_fric_nopull.resize(num_const_total, num_opt_variables);
    A_GRF_fric_nopull.setZero();
    ubA_GRF_fric_nopull.resize(num_const_total);
    ubA_GRF_fric_nopull.setZero();
    Eigen::Matrix<double, 5, 3> A_GRF_fric_nopull_comp;
    A_GRF_fric_nopull_comp << -1, 0, -mu,
                               1, 0, -mu,
                               0, -1, -mu,
                               0, 1, -mu,
                               0, 0, -1;
    VectorXd ubA_GRF_fric_nopull_comp(5);
    ubA_GRF_fric_nopull_comp << 0, 0, 0, 0, -Fz_min_QP;
    for (int GRF_idx = 0; GRF_idx < num_GRFs; GRF_idx++) {
        int row_idx = GRF_idx * num_const_per_GRF;
        int col_idx = GRF_idx * 3;
        A_GRF_fric_nopull.block(row_idx, col_idx, num_const_per_GRF, 3) = A_GRF_fric_nopull_comp;
        ubA_GRF_fric_nopull.segment(row_idx, num_const_per_GRF) = ubA_GRF_fric_nopull_comp;
    }
}


VectorXd dash_ctrl::SRB_PD_Wrench_Controller(SRB_Params srb_params, VectorXd x, MatrixXd SRB_state_ref, MatrixXd SRB_wrench_FF)
{
    // PD control of each SRB state based on given reference

    // Parameters
    double Kp_xR = srb_params.Kp_xR; // P gain for x-direction tracking
    double Kd_xR = srb_params.Kd_xR; // D gain for x-direction tracking
    double Kp_yR = srb_params.Kp_yR; // P gain for y-direction tracking
    double Kd_yR = srb_params.Kd_yR; // D gain for y-direction tracking
    double Kp_zR = srb_params.Kp_zR; // P gain for z-direction tracking
    double Kd_zR = srb_params.Kd_zR; // D gain for z-direction tracking
    double Kp_phiR = srb_params.Kp_phiR; // P gain for roll tracking
    double Kd_phiR = srb_params.Kd_phiR; // D gain for roll tracking
    double Kp_thetaR = srb_params.Kp_thetaR; // P gain for pitch tracking
    double Kd_thetaR = srb_params.Kd_thetaR; // D gain for pitch tracking
    double Kp_psiR = srb_params.Kp_psiR; // P gain for yaw tracking
    double Kd_psiR = srb_params.Kd_psiR; // D gain for yaw tracking

    // create PD gain matrices
    Eigen::Matrix<double, 6, 6> Kp;
    Kp <<   Kp_xR, 0, 0, 0, 0, 0,
                0, Kp_yR, 0, 0, 0, 0,
                0, 0, Kp_zR, 0, 0, 0,
                0, 0, 0, Kp_phiR, 0, 0,
                0, 0, 0, 0, Kp_thetaR, 0,
                0, 0, 0, 0, 0, Kp_psiR;

    Eigen::Matrix<double, 6, 6> Kd;
    Kd << Kd_xR, 0, 0, 0, 0, 0,
            0, Kd_yR, 0, 0, 0, 0,
            0, 0, Kd_zR, 0, 0, 0,
            0, 0, 0, Kd_phiR, 0, 0,
            0, 0, 0, 0, Kd_thetaR, 0,
            0, 0, 0, 0, 0, Kd_psiR;

    // get SRB states and create SRB state vectors
    Eigen::Vector3d pc = x.segment<3>(0);
    Eigen::Vector3d dpc = x.segment<3>(3);
    Matrix3d R = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Eigen::Vector3d wb = x.segment<3>(15);
    Eigen::Vector3d EA = x.segment<3>(18);
    Eigen::Vector3d dEA = dash_utils::calc_dEA(R, wb);
    Eigen::VectorXd SRB_pos(6);
    SRB_pos << pc, EA;
    Eigen::VectorXd SRB_vel(6);
    SRB_vel << dpc, dEA;

    // get SRB state references
    Eigen::VectorXd SRB_pos_ref = SRB_state_ref.col(0);
    Eigen::VectorXd SRB_vel_ref = SRB_state_ref.col(1);

    // PD control + feedforward term
    Eigen::VectorXd SRB_wrench_PD = Kp * (SRB_pos_ref - SRB_pos) + Kd * (SRB_vel_ref - SRB_vel) + SRB_wrench_FF;
    return SRB_wrench_PD;

}

void dash_ctrl::opt_stepping_controller(double& uk, VectorXd xk, VectorXd xk_des, double T_SSP, double T_DSP, double w) {

    // Optimal stepping controller (stabilizes velocity in 1 step, position in 2
    // steps if xk_des corresponds to a stable P1-orbit)    

    // Get states (Pre-Impact CoM-position and CoM-velocity)
    double p = xk(0);
    double v = xk(1);

    // Desired P-1 orbit states (Pre-Impact CoM-position and CoM-velocity)
    double p_star = xk_des(0);
    double v_star = xk_des(1);

    // Discrete control law
    uk = p + p_star + T_DSP*v + (1.0 / w)*(1.0 / tanh(T_SSP*w))*(v - v_star);

}

