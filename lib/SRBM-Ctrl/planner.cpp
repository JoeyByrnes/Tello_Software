#include "planner.h"

extern MatrixXd lfv0, lfdv0;

void dash_planner::SRB_6DoF_Test(std::string& recording_file_name, double& sim_time, SRB_Params& srb_params, MatrixXd lfv, char DoF, int num_tests)
{
    double amplitude;
    double omega;
    double phase;
    switch(DoF){
        case 'x':
            // leaning motion
            // sinusoidal trajectory parameters
            recording_file_name = "X";
            printf("Running X (Lean) Test\n");
            amplitude = (0.9*abs(lfv(0,0)))/(sqrt(2));
            omega = 1.0;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.x_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'y':
            // side to side motion
            // sinusoidal trajectory parameters
            recording_file_name = "Y";
            printf("Running Y (Side2Side) Test\n");
            amplitude = (0.9*abs(lfv(0,1)))/(sqrt(2));
            omega = 1.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.y_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'z':
            // squat motion
            // sinusoidal trajectory parameters
            recording_file_name = "Z";
            printf("Running Z (Squat) Test\n");
            amplitude = 0.06;
            omega = 1.0;
            phase = -M_PI;
            sim_time = num_tests*(M_PI/omega);
            srb_params.z_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'r':
            // roll motion
            // sinusoidal trajectory parameters
            recording_file_name = "Roll";
            printf("Running Roll Test\n");
            amplitude = 5.0*(M_PI/180.0);
            omega = 1.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.roll_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'p':
            // pitch motion
            // sinusoidal trajectory parameters
            recording_file_name = "Pitch";
            printf("Running Pitch Test\n");
            amplitude = 5.0*(M_PI/180.0);
            omega = 1.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.pitch_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'w':
            // yaw motion
            // sinusoidal trajectory parameters
            recording_file_name = "Yaw";
            printf("Running Yaw Test\n");
            amplitude = 15.0*(M_PI/180.0);
            omega = 1.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.yaw_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        default:
            // default motion (balance)
            // sinusoidal trajectory parameters
            recording_file_name = "Balance";
            printf("Running Balance Test\n");
            sim_time = 1.0*num_tests;
            break;
    }
}

int dash_planner::SRB_FSM(SRB_Params srb_params,Traj_planner_dyn_data traj_planner_dyn_data, int FSM_prev, double t, MatrixXd lfv, VectorXd u)
{
    // Finite State-Machine for SRBM reduced-order model
    // Possible states are double support phase (DSP = 0), single support phase
    // (SSP) for either the right (SSP_R = -1) or left leg (SSP_L = 1)

    // Parameters
    double Fz_min = srb_params.Fz_min_FSM; 
    double CoMz_init = srb_params.hLIP;
    double T = srb_params.T;

    // Planner info
    double t_sw_start;
    if (t != 0)
        t_sw_start = traj_planner_dyn_data.t_sw_start;
    else 
        t_sw_start = 0.0;

    double t_dsp_start;
    if (t != 0)
        t_dsp_start = traj_planner_dyn_data.t_dsp_start;
    else 
        t_dsp_start = 0.0;


    // get swing-foot z-position (front line foot pt) relative to ground --
    // assume constant height for now
    double lf1z = lfv(0,2) + CoMz_init;
    double lf3z = lfv(2,2) + CoMz_init;

    double lf2z = lfv(1,2) + CoMz_init;
    double lf4z = lfv(3,2) + CoMz_init;

    // get GRFs (front line foot pt)
    double u1z = u(2);
    double u3z = u(8);
    double u2z = u(5);
    double u4z = u(11);
    if(u1z < 0) u1z = 0;
    if(u3z < 0) u3z = 0;
    
    if(u2z < 0) u1z = 0;
    if(u4z < 0) u3z = 0;

    // compute phase variable
    double s = (t - t_sw_start) / T;

    double s_dsp = (t - t_dsp_start);

    // Finite-State Machine (FSM)    
    // From DSP can switch to SSP_L or SSP_R based on front toe z-direction GRFs
    // From SSP_L can switch to DSP based on right foot z-position
    // From SSP_R can switch to DSP based on left foot z-position
    
    int FSM_next;
    //cout << FSM_prev << "\t " << u1z << "\t " << u3z <<endl;
    if (FSM_prev == 0) // currently in DSP
    {
        if ( (u1z < Fz_min || u2z < Fz_min) && t > 0 && s_dsp > 0.06) // enter SSP_L
        {
            FSM_next = 1;
        }
        else if ( (u3z < Fz_min || u4z < Fz_min) && t > 0 && s_dsp > 0.06) // enter SSP_R 
        {
            FSM_next = -1;     
        }
        else // stay in DSP 
        {
            FSM_next = 0;
        }
    }
    else if (FSM_prev == 1) // currently in SSP_L
    {
        if ( (lf1z <= 0.0 || lf2z <= 0.0) && s > 0.6) // enter DSP
        {
            FSM_next = 0;
        }
        else // stay in SSP_L
        {
            FSM_next = 1;
        }
    }
    else if (FSM_prev == -1) // currently in SSP_R
    {
        if ( (lf3z <= 0.0 || lf3z <= 0.0) && s > 0.6) // enter DSP
        {
            FSM_next = 0;
        }
        else // stay in SSP_R
        {
            FSM_next = -1;
        } 
    }
    else // should not end up here
    {
        FSM_next = 0;
    }

    return FSM_next;

}

void dash_planner::SRB_Init_Traj_Planner_Data(Traj_planner_dyn_data& traj_planner_dyn_data, SRB_Params srb_params, Human_params human_params, VectorXd x0, MatrixXd lfv0)
{

   // This function initializes the trajectory planner data structure
    
    // Parameters
    double T_step_init = srb_params.T;
    double hH = human_params.hLIP;
    double g = srb_params.g;
    int num_end_effector_pts = 4;
    int position_vec_size = 3;

    // compute human LIP natural frequency
    double wH = sqrt(g/hH);
    
    // Calculate initial end-effector to CoM vector
    MatrixXd lf2CoM0_mat(position_vec_size, num_end_effector_pts);
    lf2CoM0_mat = x0.head(3).replicate(1, num_end_effector_pts) - lfv0.transpose();
    
    // Used by both planner_type = LIP_ang_mom_reg & planner_type = Human_Dyn_Telelocomotion
    traj_planner_dyn_data.stepping_flg = false; // flag to indicate stepping is enabled
    traj_planner_dyn_data.T_step = T_step_init; // step time (fixed most of the time)
    traj_planner_dyn_data.t_sw_start = 0.0; // SSP time (0 to T)
    traj_planner_dyn_data.t_dsp_start = 0.0; // DSP time (0 to T)
    
    // Only planner_type = LIP_ang_mom_reg
    traj_planner_dyn_data.next_SSP = 0; // next SSP (SSP_L = 1 or SSP_R = -1)
    traj_planner_dyn_data.step_width = (abs(lf2CoM0_mat(1, 0)) + abs(lf2CoM0_mat(1, 3))); // desired step width (updated at the start depending on initial feet width)
    cout << "step_width: " << traj_planner_dyn_data.step_width << endl;
    traj_planner_dyn_data.st2CoM_beg_step = VectorXd::Zero(position_vec_size); // stance-leg/foot position at the beginning-of-step relative to CoM
    traj_planner_dyn_data.sw2CoM_beg_step = VectorXd::Zero(position_vec_size); // swing-leg/foot position at the beginning-of-step relative to CoM
    traj_planner_dyn_data.xLIP_init = VectorXd::Zero(2); // initial conditions for sagittal plane LIP
    
    // Only planner_type = Human_Dyn_Telelocomotion
    traj_planner_dyn_data.sw_beg_step = VectorXd::Zero(position_vec_size); // swing-leg/foot position at the beginning-of-step
    traj_planner_dyn_data.human_leg_joystick_pos_beg_step = VectorXd::Zero(position_vec_size); // human leg joystick end-effector position at the beginning of step
    traj_planner_dyn_data.sigma1H = wH*(1.0/(tanh((T_step_init/2.0)*wH))); // human orbital line slope
    
}

void dash_planner::SRB_Traj_Planner(
    SRB_Params srb_params,
    Human_dyn_data &human_dyn_data,
    Traj_planner_dyn_data &traj_planner_dyn_data,
    Human_params human_params,
    int &FSM,
    int FSM_prev,
    double t,
    VectorXd x,
    MatrixXd lfv,
    MatrixXd lfdv,
    VectorXd u,
    VectorXd tau_ext,
    MatrixXd &SRB_state_ref,
    VectorXd &SRB_wrench_ref,
    MatrixXd &lfv_comm,
    MatrixXd &lfdv_comm)
{
    // Parameters
    double m = srb_params.m;
    double g = srb_params.g;
    int planner_type = srb_params.planner_type;
    VectorXd x_sinu_traj_params = srb_params.x_sinu_traj_params;
    VectorXd y_sinu_traj_params = srb_params.y_sinu_traj_params;
    VectorXd z_sinu_traj_params = srb_params.z_sinu_traj_params;
    VectorXd roll_sinu_traj_params = srb_params.roll_sinu_traj_params;
    VectorXd pitch_sinu_traj_params = srb_params.pitch_sinu_traj_params;
    VectorXd yaw_sinu_traj_params = srb_params.yaw_sinu_traj_params;
    int num_SRB_DoF = 6;
    
    // FSM
    FSM = SRB_FSM(srb_params, traj_planner_dyn_data, FSM_prev, t, lfv, u);
    
    // Update planner data
    traj_planner_dyn_data_gen(srb_params, human_params, traj_planner_dyn_data, human_dyn_data, t, FSM_prev, FSM, x, lfv);
    
    double FxR, FyR;
     // Control
    if (planner_type == 0)
    {
        // Basic balancing capability to track SRB state references
        
        // SRB state reference
        SRB_state_ref.row(0) = dash_utils::sinu_ref_traj(t, x_sinu_traj_params);
        SRB_state_ref.row(1) = dash_utils::sinu_ref_traj(t, y_sinu_traj_params);
        SRB_state_ref.row(2) = dash_utils::sinu_ref_traj(t, z_sinu_traj_params);
        SRB_state_ref.row(3) = dash_utils::sinu_ref_traj(t, roll_sinu_traj_params);
        SRB_state_ref.row(4) = dash_utils::sinu_ref_traj(t, pitch_sinu_traj_params);
        SRB_state_ref.row(5) = dash_utils::sinu_ref_traj(t, yaw_sinu_traj_params);

        // SRB wrench feedforward term (gravity compensation)
        VectorXd SRB_wrench_FF(num_SRB_DoF);
        SRB_wrench_FF << 0, 0, m * g, 0, 0, 0;

        // PD control to track desired SRB reference
        VectorXd SRB_wrench_PD = dash_ctrl::SRB_PD_Wrench_Controller(srb_params, x, SRB_state_ref, SRB_wrench_FF);

        // Final SRB wrench reference
        SRB_wrench_ref = SRB_wrench_PD;

        // Swing-leg control (step-placement strategy) - assume no step and stay
        // in DSP
        lfv_comm = lfv0;
        lfdv_comm = lfdv0;
    }
    else
    {
        // LIP-template based planner (s)
        if (planner_type == 1) { // Regulate angular momentum about contact point
            // LIP angular momentum regulation planner (stepping and walking)
            dash_ctrl::LIP_ang_mom_strat(FxR, FyR, lfv_comm, lfdv_comm, srb_params, traj_planner_dyn_data, FSM, t, x, lfv, lfdv);
        } else if (planner_type == 2) { // Human Whole-Body Dynamic Telelocomotion
            // Human pilot is the planner
            dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion(FxR, FyR, lfv_comm, lfdv_comm, human_dyn_data, srb_params, human_params, traj_planner_dyn_data, FSM, t, x, lfv, lfdv, tau_ext);  
        }
        
        // SRB state reference (regulate all around SRB states around zero)
        MatrixXd SRB_state_ref(num_SRB_DoF, 2);
        SRB_state_ref.setZero();

        // SRB wrench feedforward term (gravity compensation)
        VectorXd SRB_wrench_FF(num_SRB_DoF);
        SRB_wrench_FF << 0, 0, m * g, 0, 0, 0;
        
        // PD control to regulate remaining states around zero
        VectorXd SRB_wrench_PD = dash_ctrl::SRB_PD_Wrench_Controller(srb_params, x, SRB_state_ref, SRB_wrench_FF);
        
        // Final SRB wrench reference
        SRB_wrench_ref.setZero();
        SRB_wrench_ref(0) = FxR;
        SRB_wrench_ref(1) = FyR;
        SRB_wrench_ref.segment(2, num_SRB_DoF-2) = SRB_wrench_PD.segment(2, num_SRB_DoF-2);
        
    }

}


void dash_planner::traj_planner_dyn_data_gen(SRB_Params& srb_params, Human_params& human_params, Traj_planner_dyn_data& traj_planner_dyn_data, Human_dyn_data human_dyn_data,double t,int FSM_prev,int FSM, VectorXd x, MatrixXd lfv)
{
    // Parameters
    int planner_type = srb_params.planner_type;
    double t_beg_stepping = srb_params.t_beg_stepping;
    double t_end_stepping = srb_params.t_end_stepping;
    double ft_l = srb_params.foot_length;
    double hH = human_params.hLIP;
    double g = srb_params.g;

    // Get trajectory planner current data
    double t_sw_start = traj_planner_dyn_data.t_sw_start;
    double T_step = traj_planner_dyn_data.T_step;

    // Get SRB states
    VectorXd pc = x.head(3);
    double dx = x(3);

    // Get human leg joystick data
    double fxH_R = human_dyn_data.fxH_R;
    double fyH_R = human_dyn_data.fyH_R;
    double fzH_R = human_dyn_data.fzH_R;
    double fxH_L = human_dyn_data.fxH_L;
    double fyH_L = human_dyn_data.fyH_L;
    double fzH_L = human_dyn_data.fzH_L;
    VectorXd human_leg_joystick_data(6);
    human_leg_joystick_data << fxH_R, fyH_R, fzH_R, fxH_L, fyH_L, fzH_L;

    // compute human LIP natural frequency
    double wH = sqrt(g/hH);

    // Comment out stepping if planner_type = none
    if (planner_type == 0) {
        t_beg_stepping = 1e5;
        t_end_stepping = 1e6;
    }

    // Enable stepping/walking if planner_type = Human_Dyn_Telelocomotion
    if (planner_type == 2) {
        traj_planner_dyn_data.stepping_flg = true;
        t_end_stepping = 1e6;
    }

    // stepping dynamic data
    if (!traj_planner_dyn_data.stepping_flg && (t > t_beg_stepping && t < t_end_stepping)) { // command stepping (first step is with the right leg, i.e. FSM = SSP_L)

        // command stepping (start with SSP_L)
        traj_planner_dyn_data.stepping_flg = true;
        traj_planner_dyn_data.next_SSP = 1;

    }
    else if(traj_planner_dyn_data.stepping_flg) // during stepping
    {
        int next_SSP;
        int st_idx_R;
        int sw_idx_R;
        int sw_idx_H;
        if (FSM_prev == 0 && abs(FSM) == 1) { // DSP to SSP transition

            // initialize swing-phase time variable
            traj_planner_dyn_data.t_sw_start = t;

            // get correct swing and stance leg idx 
            // schedule next SSP
            if (FSM == 1) { // SSP_L

                // next SSP is SSP_R
                next_SSP = -1; 

                // stance is left leg swing is right leg
                st_idx_R = 2;
                sw_idx_R = 0; 
                sw_idx_H = 0; 

            } else if (FSM == -1) { // SSP_R

                // next SSP is SSP_L
                next_SSP = 1; 

                // stance is right leg swing is left leg
                st_idx_R = 0;
                sw_idx_R = 2;   
                sw_idx_H = 3; 

            }

            // beginning of step data
            // set next SSP for planner_type = LIP_ang_mom_reg 
            if (planner_type == 1) { // planner_type = LIP_ang_mom_reg 
                // update
                traj_planner_dyn_data.next_SSP = next_SSP;
                traj_planner_dyn_data.st2CoM_beg_step = pc - lfv.row(st_idx_R).transpose();
                traj_planner_dyn_data.sw2CoM_beg_step = pc - lfv.row(sw_idx_R).transpose();  
                traj_planner_dyn_data.xLIP_init << (pc(0) - lfv.row(st_idx_R)(0)) + (1.0/2.0)*ft_l, dx;
                
            } else if (planner_type == 2) { // planner_type = Human_Dyn_Telelocomotion

                // update
                traj_planner_dyn_data.sw_beg_step = lfv.row(sw_idx_R);
                traj_planner_dyn_data.sw_beg_step(0) = traj_planner_dyn_data.sw_beg_step(0) - (1.0/2.0)*ft_l;
                traj_planner_dyn_data.human_leg_joystick_pos_beg_step = human_leg_joystick_data.segment<3>(sw_idx_H);
                traj_planner_dyn_data.sigma1H = wH*(1.0/(tanh((T_step/2.0)*wH)));

            }

        } else if ((FSM_prev == 1 || FSM_prev == -1) && FSM == 0) { // SSP to DSP transition
            // update step time with previous step duration
            traj_planner_dyn_data.t_dsp_start = t;

            if (planner_type == 2) {
                traj_planner_dyn_data.T_step = t - traj_planner_dyn_data.t_sw_start;
            }

            // terminate walking/stepping
            if (t > t_end_stepping) {
                traj_planner_dyn_data.stepping_flg = false;
                traj_planner_dyn_data.next_SSP = 0;
            }
        }
    }
}

void dash_planner::gen_vel_trapz_traj(const VectorXd& t_waypts, const VectorXd& v_waypts, VectorXd& t_traj, VectorXd& v_traj)
{
    double dt = 0.001;
    int num_lines = t_waypts.size() - 1;

    // Time vector
    t_traj = VectorXd::LinSpaced(static_cast<int>((t_waypts[num_lines] - t_waypts[0])/dt) + 1, t_waypts[0], t_waypts[num_lines]);

    // Velocity vector
    v_traj = VectorXd::Zero(t_traj.size());
    int time_start_idx = 0;
    for (int line_idx = 0; line_idx < num_lines; line_idx++)
    {
        // Time vector for line segment
        VectorXd t_line = VectorXd::LinSpaced(static_cast<int>((t_waypts[line_idx + 1] - t_waypts[line_idx])/dt) + 1, t_waypts[line_idx], t_waypts[line_idx + 1]);

        // Velocity vector for line segment
        VectorXd v_line = VectorXd::LinSpaced(static_cast<int>((t_waypts[line_idx + 1] - t_waypts[line_idx])/dt) + 1, v_waypts[line_idx], v_waypts[line_idx + 1]);

        // Populate overall
        if (line_idx < num_lines - 1)
            v_traj.segment(time_start_idx, t_line.size() - 1) = v_line.segment(0, t_line.size() - 1);
        else
            v_traj.segment(time_start_idx, t_line.size()) = v_line.segment(0, t_line.size());

        // Update time start index
        time_start_idx += t_line.size() - 1;
    }
}

// void dash_planner::gen_smooth_traj(const VectorXd& t_waypts, const VectorXd& v_waypts, VectorXd& t_traj, VectorXd& v_traj)
// {
//     double dt = 0.001;
//     int num_lines = t_waypts.size() - 1;

//     // Time vector
//     t_traj = VectorXd::LinSpaced(static_cast<int>((t_waypts[num_lines] - t_waypts[0])/dt) + 1, t_waypts[0], t_waypts[num_lines]);

//     // Velocity vector
//     v_traj = VectorXd::Zero(t_traj.size());
//     int time_start_idx = 0;
//     for (int line_idx = 0; line_idx < num_lines; line_idx++)
//     {
//         // Time vector for line segment
//         VectorXd t_line = VectorXd::LinSpaced(static_cast<int>((t_waypts[line_idx + 1] - t_waypts[line_idx])/dt) + 1, t_waypts[line_idx], t_waypts[line_idx + 1]);

//         // Velocity vector for line segment
//         VectorXd v_line = (v_waypts[line_idx + 1] - v_waypts[line_idx]) / (1 + exp(-25 * (t_line.array() - (t_waypts[line_idx] + t_waypts[line_idx + 1])/2))) + v_waypts[line_idx];

//         // Populate overall
//         if (line_idx < num_lines - 1)
//             v_traj.segment(time_start_idx, t_line.size() - 1) = v_line.segment(0, t_line.size() - 1);
//         else
//             v_traj.segment(time_start_idx, t_line.size()) = v_line.segment(0, t_line.size());

//         // Update time start index
//         time_start_idx += t_line.size() - 1;
//     }
// }
void dash_planner::gen_smooth_traj(const VectorXd& t_waypts, const VectorXd& v_waypts, VectorXd& t_traj, VectorXd& v_traj)
{
    double dt = 0.001;
    int num_lines = t_waypts.size() - 1;

    // Time vector
    t_traj = VectorXd::LinSpaced(static_cast<int>((t_waypts[num_lines] - t_waypts[0])/dt) + 1, t_waypts[0], t_waypts[num_lines]);

    // Velocity vector
    v_traj = VectorXd::Zero(t_traj.size());
    int time_start_idx = 0;
    for (int line_idx = 0; line_idx < num_lines; line_idx++)
    {
        // Time vector for line segment
        VectorXd t_line = VectorXd::LinSpaced(static_cast<int>((t_waypts[line_idx + 1] - t_waypts[line_idx])/dt) + 1, t_waypts[line_idx], t_waypts[line_idx + 1]);

        // Velocity vector for line segment
        double vel_start = v_waypts[line_idx];
        double vel_end = v_waypts[line_idx + 1];
        double t_start = t_waypts[line_idx];
        double t_end = t_waypts[line_idx + 1];
        double vel_diff = vel_end - vel_start;
        double t_diff = t_end - t_start;
        VectorXd v_line(t_line.size());

        for (int i = 0; i < t_line.size(); i++) {
            double t = t_line[i];
            double sigmoid_arg = 14.0*(t - t_start)/t_diff - 7.0;
            v_line[i] = vel_start + vel_diff/(1.0 + exp(-sigmoid_arg));
        }

        // Populate overall
        if (line_idx < num_lines - 1)
            v_traj.segment(time_start_idx, t_line.size() - 1) = v_line.segment(0, t_line.size() - 1);
        else
            v_traj.segment(time_start_idx, t_line.size()) = v_line.segment(0, t_line.size());

        // Update time start index
        time_start_idx += t_line.size() - 1;
    }
}

void dash_planner::SRB_LIP_vel_traj(double des_walking_speed, VectorXd& t_traj, VectorXd& v_traj, double& t_beg_stepping_time, double& t_end_stepping_time)
{
    // Tunable waypoints (time in s)
    VectorXd t_waypts_0p1to3ms(6), t_waypts_0p4ms(6), t_waypts_0p5to6ms(6), t_waypts_0p7ms(6), t_waypts_0p8ms(6);
    t_waypts_0p1to3ms << 0.0, 1.0, 2.5, 5.5, 7.0, 8.0;
    t_waypts_0p4ms << 0.0, 1.0, 2.0, 4.0, 6.0, 7.0;
    t_waypts_0p5to6ms << 0.0, 1.0, 2.0, 4.0, 6.0, 7.5;
    t_waypts_0p7ms << 0.0, 1.0, 4.0, 6.0, 9.0, 13.0;
    t_waypts_0p8ms << 0.0, 1.0, 6.0, 8.0, 13.0, 17.0;

     // select based on desired top speed
    VectorXd t_waypts(6), v_waypts(6);
    if (des_walking_speed <= 0.3)
    {
        t_waypts = t_waypts_0p1to3ms;
    }
    else if (des_walking_speed <= 0.4)
    {
        t_waypts = t_waypts_0p4ms;
    }
    else if (des_walking_speed <= 0.6)
    {
        t_waypts = t_waypts_0p5to6ms;
    }
    else if (des_walking_speed <= 0.7)
    {
        t_waypts = t_waypts_0p7ms;
    }
    else if (des_walking_speed <= 0.8)
    {
        t_waypts = t_waypts_0p8ms;
    }
    v_waypts << 0.0, 0.0, des_walking_speed, des_walking_speed, 0.0, 0.0;

    // generate trajectory
    //gen_vel_trapz_traj(t_waypts, v_waypts, t_traj, v_traj);
    gen_smooth_traj(t_waypts, v_waypts, t_traj, v_traj);
    // cout << "V_TRAJ =============================" << endl;
    // cout << v_traj << endl;

    // time to command begin stepping
    t_beg_stepping_time = (t_waypts(1) - t_waypts(0))/2.0;

    // time to command end stepping
    int end = t_waypts.size() - 1;
    t_end_stepping_time = t_waypts[end-1] + (t_waypts[end] - t_waypts[end-1])/2.0;

}