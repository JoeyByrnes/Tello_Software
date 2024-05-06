#include "planner.h"
#include "dynamic_robot.h"

extern RoboDesignLab::DynamicRobot* tello;
extern MatrixXd lfv0, lfdv0;

extern bool en_v2_ctrl;
extern bool use_adaptive_step_time;

extern bool auto_mode;

extern double rfz;
extern double rbz;
extern double lfz;
extern double lbz;

extern double fzH0_min_L;
extern double fzH0_min_R;

double ssp_start_time = 0;
double prev_step_duration = 0.4;
double prev_step_amplitude = 0.03;

extern bool curve_fitting_complete;
extern bool data_ready_for_curve_fitting;
extern bool ready_for_new_curve_fit_data;

double curve_fit_offset = 0.0;

Eigen::VectorXd timevec(100);
Eigen::VectorXd AHvec(100);

MatrixXd lfv_dsp_start(4,3);

bool first_time_planner = true;

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
            amplitude = (0.7*abs(lfv(0,0)))/(sqrt(2));
            omega = 0.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.x_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'y':
            // side to side motion
            // sinusoidal trajectory parameters
            recording_file_name = "Y";
            printf("Running Y (Side2Side) Test\n");
            amplitude = (1.0*abs(lfv(0,1)))/(sqrt(2));
            omega = 0.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.y_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'z':
            // squat motion
            // sinusoidal trajectory parameters
            recording_file_name = "Z";
            printf("Running Z (Squat) Test\n");
            amplitude = 0.03;
            omega = 0.5;
            phase = -M_PI;
            sim_time = num_tests*(M_PI/omega);
            srb_params.z_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'r':
            // roll motion
            // sinusoidal trajectory parameters
            recording_file_name = "Roll";
            printf("Running Roll Test\n");
            amplitude = 8.0*(M_PI/180.0);
            omega = 0.5;
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
            omega = 0.5;
            phase = 0.0;
            sim_time = num_tests*(2.0*M_PI/omega);
            srb_params.pitch_sinu_traj_params = Eigen::Vector3d(omega, amplitude, phase);
            break;
        case 'w':
            // yaw motion
            // sinusoidal trajectory parameters
            recording_file_name = "Yaw";
            printf("Running Yaw Test\n");
            amplitude = 10.0*(M_PI/180.0);
            omega = 0.5;
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

int dash_planner::predict_ssp_params(Traj_planner_dyn_data& traj_planner_dyn_data, Human_dyn_data& human_dyn_data, double t)
{
    int ssp_data_len = traj_planner_dyn_data.curr_SSP_sample_count;
    int human_FSM = traj_planner_dyn_data.human_FSM;
    double AH;
    double end_time;
    if(human_FSM == 0 || ssp_data_len < 36)
    {
        // traj_planner_dyn_data.AH_step_predicted = 0;
        // traj_planner_dyn_data.T_step_predicted = 0;
        timevec.setConstant(traj_planner_dyn_data.T_step_actual);
        AHvec.setConstant(traj_planner_dyn_data.AH_step_actual);
        tello->controller->set_timevec(timevec);
        tello->controller->set_AHvec(AHvec);
        return 0;
    }
    else
    {
        VectorXd ydata_raw;
        if(human_FSM == 1) // Right Swing Leg
        {
            ydata_raw = tello->controller->getStepZHistoryR().tail(ssp_data_len);
        }
        else  // Left Swing Leg
        {
            ydata_raw = tello->controller->getStepZHistoryL().tail(ssp_data_len);
        }
        VectorXd tdata_raw = tello->controller->getStepTimeHistory().tail(ssp_data_len);
        double firstPoint = ydata_raw(0);
        double firstTime = tdata_raw(0);
        VectorXd ydata = ydata_raw.array() - firstPoint;
        curve_fit_offset = firstPoint;
        VectorXd xdata = tdata_raw.array() - firstTime;
        int index = -1;
        for (int i = 0; i < ydata.size(); i++) {
            if (ydata(i) > 0.004) {
                index = i-30;
                break;
            }
        }
        if(index > 0){
            int len = ydata_raw.size();
            ydata = ydata.tail(len-(index)).eval();
            xdata = xdata.tail(len-(index)).eval();
        }
        // if(human_FSM == 1) // Right Swing Leg
        // {
        //     dash_utils::writeVectorToCsv(ydata,"ydata_right.csv");
        // }
        // else  // Left Swing Leg
        // {
        //     dash_utils::writeVectorToCsv(ydata,"ydata_left.csv");
        // }
        firstTime = xdata(0);
        xdata = xdata.array() - firstTime;
        // dash_utils::writeVectorToCsv(xdata,"tdata.csv");
        if(ready_for_new_curve_fit_data)
        {
            tello->controller->set_xdata(xdata);
            tello->controller->set_ydata(ydata);
            data_ready_for_curve_fitting = true;
        }
        // double x0[] = { 0.04 , prev_step_duration };
        // double lb[] = { 0.000 , prev_step_duration - (((double)xdata.size()-35)/1000.0) };
        // double ub[] = { 1000 , prev_step_duration + (((double)xdata.size()-35)/1000.0) };
        // coder::array<double, 2U> x_data = dash_utils::eigenVectorToCoderArray(xdata);
        // coder::array<double, 2U> y_data = dash_utils::eigenVectorToCoderArray(ydata);
        // step_z_curve_fit(x_data, y_data, x0, lb, ub, &AH, &end_time);
        // traj_planner_dyn_data.AH_step_predicted = AH;
        
        // if(end_time < 0.05) end_time = prev_step_duration;
        // timevec.tail(99) = timevec.head(99).eval();
        // timevec[0] = end_time;
        // double timeval = dash_utils::smoothData(timevec, 0.8);

        // traj_planner_dyn_data.T_step_predicted = timeval;
    }

    return 0;
}

int dash_planner::SRB_FSM(SRB_Params srb_params,Traj_planner_dyn_data& traj_planner_dyn_data, Human_dyn_data& human_dyn_data, int FSM_prev, double t, MatrixXd lfv, VectorXd u)
{
    // Finite State-Machine for SRBM reduced-order model
    // Possible states are double support phase (DSP = 0), single support phase
    // (SSP) for either the right (SSP_R = -1) or left leg (SSP_L = 1)

    // Parameters
    int planner_type = srb_params.planner_type;

    double Fz_min = srb_params.Fz_min_FSM; 
    double CoMz_init = srb_params.hLIP;
    double T_DSP = srb_params.T_DSP;

    // Planner info
    double swing_T_scaler = srb_params.swing_time_scaler;
    double T = traj_planner_dyn_data.T_step;
    if(use_adaptive_step_time && !auto_mode) T = traj_planner_dyn_data.T_step_predicted*swing_T_scaler;
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

    int next_SSP = traj_planner_dyn_data.next_SSP;

    // get swing-foot z-position (front line foot pt) relative to ground --
    // assume constant height for now
    double lf1z = lfv(0,2) + CoMz_init;
    double lf3z = lfv(2,2) + CoMz_init;

    double lf2z = lfv(1,2) + CoMz_init;
    double lf4z = lfv(3,2) + CoMz_init;

    // cout << "lf1z: " << lf1z  << "   lf2z: " << lf2z << "   lf3z: " << lf3z  << "   lf4z: " << lf4z  << endl;

    // get GRFs (front line foot pt)
    double u1z = u(2);
    double u3z = u(8);
    double u2z = u(5);
    double u4z = u(11);
    if(u1z < 0) u1z = 0;
    if(u3z < 0) u3z = 0;
    
    if(u2z < 0) u2z = 0;
    if(u4z < 0) u4z = 0;

    rfz = u1z;
    rbz = u2z;
    lfz = u3z;
    lbz = u4z;

    // compute phase variable for SSP
    double s = (t - t_sw_start) / T;
    if(s < 0.0) s = 0.0;
    if(s > 1.5) s = 1.5;

    // compute time variable for DSP
    double t_dsp = (t - t_dsp_start);

    // Finite-State Machine (FSM)    
    // From DSP can switch to SSP_L or SSP_R based on front toe z-direction GRFs
    // From SSP_L can switch to DSP based on right foot z-position
    // From SSP_R can switch to DSP based on left foot z-position

    // cout << "next_SSP: " << next_SSP << "   u1z: " << u1z  << "   u2z: " << u2z << "   u3z: " << u3z  << "   u4z: " << u4z  << endl;
    double grf_rf = tello->_GRFs.right_front;
    double grf_rb = tello->_GRFs.right_back;
    double grf_lf = tello->_GRFs.left_front;
    double grf_lb = tello->_GRFs.left_back;

    double fzH0 = traj_planner_dyn_data.human_leg_joystick_pos_beg_step(2);
    
    double human_FSM = traj_planner_dyn_data.human_FSM;
    double human_next_SSP = traj_planner_dyn_data.human_next_SSP;
    double step_z_offset_R = traj_planner_dyn_data.step_z_offset_R;
    double step_z_offset_L = traj_planner_dyn_data.step_z_offset_L;
    
    double zHr = human_dyn_data.fzH_R - step_z_offset_R;
    double zHl = human_dyn_data.fzH_L - step_z_offset_L;

    if(zHr < 0.0) zHr = 0.0;
    if(zHl < 0.0) zHl = 0.0;


    // human FSM code:
    double step_threshold_up = 0.007;
    double step_threshold_down = 0.004;
    int num_init_samples = 100;
    
    if(planner_type == 2){
        if(human_FSM == 0)
        {
            if(zHr > step_threshold_up && human_next_SSP == 1)
            {
                human_FSM = 1;
                prev_step_amplitude = traj_planner_dyn_data.AH_step_actual;
                prev_step_duration = traj_planner_dyn_data.T_step_actual;
                tello->controller->set_prev_step_duration(prev_step_duration);
                tello->controller->set_prev_step_amplitude(prev_step_amplitude);
                ssp_start_time = t-0.065;
                traj_planner_dyn_data.T_step_actual = 0;
                traj_planner_dyn_data.AH_step_actual = 0;
                human_next_SSP = -1;
                traj_planner_dyn_data.curr_SSP_sample_count = num_init_samples;
            }
            else if(zHl > step_threshold_up && human_next_SSP == -1)
            {
                human_FSM = -1;
                prev_step_amplitude = traj_planner_dyn_data.AH_step_actual;
                prev_step_duration = traj_planner_dyn_data.T_step_actual;
                tello->controller->set_prev_step_duration(prev_step_duration);
                tello->controller->set_prev_step_amplitude(prev_step_amplitude);
                ssp_start_time = t-0.035;
                traj_planner_dyn_data.T_step_actual = 0;
                traj_planner_dyn_data.AH_step_actual = 0;
                human_next_SSP = 1;
                traj_planner_dyn_data.curr_SSP_sample_count = num_init_samples;
            }

        }
        else if(human_FSM == 1)
        {
            if(zHr < step_threshold_down)
            {
                human_FSM = 0;
                traj_planner_dyn_data.T_step_actual = (t+0.035)-ssp_start_time;
                traj_planner_dyn_data.curr_SSP_sample_count = 0;
                traj_planner_dyn_data.AH_step_actual = traj_planner_dyn_data.step_z_history_R.tail((traj_planner_dyn_data.T_step_actual*1000)).maxCoeff()-traj_planner_dyn_data.step_z_offset_R;
            }
        }
        else if(human_FSM == -1)
        {
            if(zHl < step_threshold_down)
            {
                human_FSM = 0;
                traj_planner_dyn_data.T_step_actual = (t+0.035)-ssp_start_time;
                traj_planner_dyn_data.curr_SSP_sample_count = 0;
                traj_planner_dyn_data.AH_step_actual = traj_planner_dyn_data.step_z_history_L.tail((traj_planner_dyn_data.T_step_actual*1000)).maxCoeff()-traj_planner_dyn_data.step_z_offset_L;
            }
        }
        traj_planner_dyn_data.human_FSM = human_FSM;
        traj_planner_dyn_data.human_next_SSP = human_next_SSP;
    }
    
    
    int FSM_next;
    // cout << "FSM: "<<(int)FSM_prev << "\t grf_check: " << (grf_rf < Fz_min && grf_rb < Fz_min ) << "\t next_SSP: " << next_SSP << "\t t_check: " << (t > 0 && t_dsp > 0.080) << "\t zHr Check: " << (zHr > 0.007) << endl;
    // cout << "grf_rf: " << grf_rf << " \t grf_rb: " << grf_rb << "\t grf_lf: " << grf_lf << "\t grf_lb: " << grf_lb << endl;
    if (FSM_prev == 0) // currently in DSP
    {
        if ( ((grf_rf + grf_rb) < 40 ) && t > 0.1 && t_dsp > 0.05 && (next_SSP==1) && ( (zHr > 0.006) || auto_mode) ) // enter SSP_L
        {
            cout << "Setting FSM from 0 to 1,   time: " << t << endl;
            FSM_next = 1;
            traj_planner_dyn_data.step_z_offset_L = human_dyn_data.fzH_L;
        }
        else if ( ((grf_lf + grf_lb) < 20 ) && t > 0.1 && t_dsp > 0.05 && (next_SSP==-1) && ( (zHl > 0.006) || auto_mode) ) // enter SSP_R 
        {
            cout << "Setting FSM from 0 to -1,   time: " << t << endl;
            FSM_next = -1;     
           traj_planner_dyn_data.step_z_offset_R = human_dyn_data.fzH_R;
        }
        else // stay in DSP 
        {
            FSM_next = 0;
        }
    }
    else if (FSM_prev == 1) // currently in SSP_L
    {
        if ( ( ((grf_rf + grf_rb) > 10 ) || (lf1z <= 0.005 || lf2z <= 0.005) ) && s > 0.8) // enter DSP
        {
            cout << "Setting FSM from 1 to 0,   time: " << t << endl;
            FSM_next = 0;
        }
        else // stay in SSP_L
        {
            FSM_next = 1;
        }
    }
    else if (FSM_prev == -1) // currently in SSP_R
    {
        if ( ( ((grf_lf + grf_lb) > 10 ) || (lf3z <= 0.005 || lf4z <= 0.005) ) && s > 0.8) // enter DSP
        {
            cout << "Setting FSM from -1 to 0,   time: " << t << endl;
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
    // cout << "INITIALIZING NEXT SSP TO ONE" << endl;
    traj_planner_dyn_data.step_width = (abs(lf2CoM0_mat(1, 0)) + abs(lf2CoM0_mat(1, 3))); // desired step width (updated at the start depending on initial feet width)
    traj_planner_dyn_data.st2CoM_beg_step = VectorXd::Zero(position_vec_size); // stance-leg/foot position at the beginning-of-step relative to CoM
    traj_planner_dyn_data.sw2CoM_beg_step = VectorXd::Zero(position_vec_size); // swing-leg/foot position at the beginning-of-step relative to CoM
    traj_planner_dyn_data.xLIP_init = VectorXd::Zero(2); // initial conditions for sagittal plane LIP
    
    // Only planner_type = Human_Dyn_Telelocomotion
    traj_planner_dyn_data.sw_beg_step = VectorXd::Zero(position_vec_size); // swing-leg/foot position at the beginning-of-step
    traj_planner_dyn_data.human_leg_joystick_pos_beg_step = VectorXd::Zero(position_vec_size); // human leg joystick end-effector position at the beginning of step
    traj_planner_dyn_data.sigma1H = wH*(1.0/(tanh((T_step_init/2.0)*wH))); // human orbital line slope
    traj_planner_dyn_data.x_HWRM = 0.0; // initial CoM position of HWRM-LIP
    traj_planner_dyn_data.dx_HWRM = 0.0; // initial CoM velocity of HWRM-LIP
    traj_planner_dyn_data.x_plus_HWRM = VectorXd::Zero(2); // initial HWRM-LIP state vector pre-phase (SSP or DSP)
    traj_planner_dyn_data.uk_HWRM = 0.0; // initial HWRM-LIP step placement
    ::lfv0 = lfv0;
    ::lfdv0.setZero();

    first_time_planner = true;
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
    MatrixXd &lfdv_comm,
    MatrixXd &lfddv_comm)
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

    tello->controller->set_lfv_dsp_start(lfv_dsp_start);
    
    // FSM
    FSM = SRB_FSM(srb_params, traj_planner_dyn_data,human_dyn_data, FSM_prev, t, lfv, u);
    // dash_utils::start_timer();
    if(planner_type == 2)
        predict_ssp_params(traj_planner_dyn_data,human_dyn_data, t);
    // dash_utils::print_timer();
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
            dash_ctrl::LIP_ang_mom_strat(FxR, FyR, lfv_comm, lfdv_comm, lfddv_comm, srb_params, traj_planner_dyn_data, FSM, t, x, lfv, lfdv);
        } else if (planner_type == 2) { // Human Whole-Body Dynamic Telelocomotion
            // Human pilot is the planner
            // if( en_v2_ctrl )
                dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion_v2(FxR, FyR, lfv_comm, lfdv_comm, lfddv_comm, human_dyn_data, srb_params, human_params, traj_planner_dyn_data, FSM, t, x, lfv, lfdv, tau_ext);  
            // else
            //     dash_ctrl::Human_Whole_Body_Dyn_Telelocomotion_v3(FxR, FyR, lfv_comm, lfdv_comm, lfddv_comm, human_dyn_data, srb_params, human_params, traj_planner_dyn_data, FSM, t, x, lfv, lfdv, tau_ext, u);
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

Vector3d st_beg_step_last;
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
     double swing_T_scaler = srb_params.swing_time_scaler;
    double T_step = traj_planner_dyn_data.T_step;
    if(use_adaptive_step_time && !auto_mode) T_step = traj_planner_dyn_data.T_step_predicted*swing_T_scaler;
    double x_HWRM = traj_planner_dyn_data.x_HWRM;
    double dx_HWRM = traj_planner_dyn_data.dx_HWRM;
    double uk_HWRM = traj_planner_dyn_data.uk_HWRM;
    // cout << "    Planner uk_HWRM: " << uk_HWRM;

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

    if(fzH_R < fzH0_min_R && fzH_R != 0) fzH0_min_R = fzH_R;
    if(fzH_L < fzH0_min_L && fzH_L != 0) fzH0_min_L = fzH_L;

    // compute human LIP natural frequency
    double wH = sqrt(g/hH);

    // Comment out stepping if planner_type = none
    if (planner_type == 0) {
        t_beg_stepping = 1e90;
        t_end_stepping = 1e100;
    }

    // Enable stepping/walking if planner_type = Human_Dyn_Telelocomotion
    if (planner_type == 2) {
        traj_planner_dyn_data.stepping_flg = true;
        t_end_stepping = 1e100;
        if(first_time_planner)
        {
            traj_planner_dyn_data.next_SSP = 1;
            lfv_dsp_start = lfv;
            first_time_planner = false;
            cout << "Setting Next_SSP to 1" << endl;
        }
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
        Vector2d x_plus_HWRM;
        Vector2d x_minus_HWRM;
        x_minus_HWRM << x_HWRM, dx_HWRM;

        if (FSM_prev == 0 && abs(FSM) == 1) { // DSP to SSP transition

            // initialize swing-phase time variable
            traj_planner_dyn_data.t_sw_start = t;

            // DSP to SSP reset map for HWRM dyn.
            dash_dyn::HLIP_Reset_Map_DSP_SSP(x_plus_HWRM, x_minus_HWRM, uk_HWRM);

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
                traj_planner_dyn_data.sw_beg_step = lfv.row(sw_idx_R);
                traj_planner_dyn_data.sw_beg_step(2) = -srb_params.hLIP ;//-0.0005;
                
            } else if (planner_type == 2) { // planner_type = Human_Dyn_Telelocomotion

                // update
                traj_planner_dyn_data.next_SSP = next_SSP;
                traj_planner_dyn_data.sw_beg_step = lfv.row(sw_idx_R);
                traj_planner_dyn_data.sw_beg_step(2) = -srb_params.hLIP ;
                traj_planner_dyn_data.st_beg_step = lfv.row(st_idx_R);
                traj_planner_dyn_data.sw_beg_step(0) = traj_planner_dyn_data.sw_beg_step(0) - (1.0/2.0)*ft_l;
                traj_planner_dyn_data.st_beg_step(0) = traj_planner_dyn_data.st_beg_step(0) - (1.0/2.0)*ft_l;
                st_beg_step_last = traj_planner_dyn_data.st_beg_step;
                traj_planner_dyn_data.human_leg_joystick_pos_beg_step = human_leg_joystick_data.segment<3>(sw_idx_H);
                traj_planner_dyn_data.sigma1H = wH*(1.0/(tanh((T_step/2.0)*wH)));
                traj_planner_dyn_data.x_plus_HWRM = x_plus_HWRM;

            }

        } else if ((FSM_prev == 1 || FSM_prev == -1) && FSM == 0) { // SSP to DSP transition

            // update step time with previous step duration
            traj_planner_dyn_data.t_dsp_start = t;
            lfv_dsp_start = lfv;

            traj_planner_dyn_data.y_LIP_offset = (lfv(0,1) + lfv(2,1))/2.0; 

            // final step time (store as assumed step time for next step)
            double T_step_final = t - t_sw_start;

             //update HWRM step placement (if necessary)
            if (T_step_final < T_step) {
                 uk_HWRM = (1.0/2.0) * (1.0 - cos(M_PI * (T_step_final/T_step))) * uk_HWRM;
                 traj_planner_dyn_data.uk_HWRM = uk_HWRM;
            }

            // DSP to SSP reset map for HWRM dyn.
            dash_dyn::HLIP_Reset_Map_SSP_DSP(x_plus_HWRM, x_minus_HWRM);            

            if (planner_type == 2) {
                // traj_planner_dyn_data.T_step = T_step_final;
                traj_planner_dyn_data.x_plus_HWRM = x_plus_HWRM;
            }

            // terminate walking/stepping
            if (t > t_end_stepping) {
                traj_planner_dyn_data.stepping_flg = false;
                traj_planner_dyn_data.next_SSP = 0;
                cout << "STEPPING OVER, SETTING NEXT_SSP TO ZERO" << endl;
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
    double step_end = 3;
    double step_begin = 2;
    t_waypts_0p1to3ms << 0.0, step_begin, step_begin+1, step_end, step_end+1, step_end+2;
    t_waypts_0p4ms << 0.0, 1.0, 2.0, 4.0, 6.0, 7.0;
    t_waypts_0p5to6ms << 0.0, 1.0, 2.0, 4.0, 6.0, 7.5;
    t_waypts_0p7ms << 0.0, 1.0, 4.0, 6.0, 9.0, 13.0;
    t_waypts_0p8ms << 0.0, 1.0, 6.0, 8.0, 13.0, 17.0;

     // select based on desired top speed
    VectorXd t_waypts(6), v_waypts(6);
    if (des_walking_speed <= 5)
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
    gen_vel_trapz_traj(t_waypts, v_waypts, t_traj, v_traj);
    // gen_smooth_traj(t_waypts, v_waypts, t_traj, v_traj);
    // cout << "V_TRAJ =============================" << endl;
    // cout << v_traj << endl;

    // time to command begin stepping
    t_beg_stepping_time = 1.0;//(t_waypts(1) - t_waypts(0))/2.0;

    // time to command end stepping
    int end = t_waypts.size() - 1;
    t_end_stepping_time = 5.0;//t_waypts[end-1] + (t_waypts[end] - t_waypts[end-1])/2.0;

}
