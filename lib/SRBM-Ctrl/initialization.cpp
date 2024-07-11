#include "initialization.h"
#include "../../include/user_config.h"

extern int simulation_mode;

#define M_PI 3.14159265358979323846

#define USING_ARMS 1

void dash_init::Human_Init(Human_params &Human_params, Human_dyn_data &Human_dyn_data) {
    // Human Parameters
    double joystick_base_separation = 1.525;
    // double foot_center_to_joystick = 0.0825;

    // Human_params.m = 75; // human weight in kg
    // Human_params.hLIP = 1.2; // nominal human LIP height

    // Human_params.m = 75; // human weight in kg
    // Human_params.hLIP = 1.2; // nominal human LIP height

    Human_params.m = 76; // human weight in kg //Guillermo = 79 //joao = 95
    Human_params.hLIP = 1.05; // nominal human LIP height //Guillermo = 1.15 // joao = 1.17

    Human_params.human_nom_ft_width = 0.1750; // nominal human feet width //was 0.175
    Human_params.fyH_home = (joystick_base_separation/2.0) - Human_params.human_nom_ft_width - FOOT_2_JOYSTICK; // joystick y width
    Human_params.foot_2_joystick = FOOT_2_JOYSTICK;
    // Note: HMI joystick bases are 1.465m apart
    //       distance from center of foot to joystick end is 0.0635

    // HMI data

    // human LIP models (frontal and sagittal plane)
    Human_dyn_data.xH = 0.0; // human x-CoM position
    Human_dyn_data.dxH = 0.0; // human x-CoM velocity
    Human_dyn_data.pxH = 0.0; // human x-CoP position
    Human_dyn_data.yH = 0.0; // human y-CoM position
    Human_dyn_data.dyH = 0.0; // human y-CoM velocity
    Human_dyn_data.pyH = 0.0; // human y-CoP position

    // lower-body joysticks
    Human_dyn_data.fxH_R = 0.0; // human right end effector (x-position)
    Human_dyn_data.fyH_R = 0.0; // human right end effector (y-position) //default is 0.494 from center
    Human_dyn_data.fzH_R = 0.0; // human right end effector (z-position)
    Human_dyn_data.fxH_L = 0.0; // human left end effector (x-position)
    Human_dyn_data.fyH_L = 0.0; // human left end effector (y-position)
    Human_dyn_data.fzH_L = 0.0; // human left end effector (z-position)
    Human_dyn_data.fdxH_R = 0.0; // human right end effector (x-velocity)
    Human_dyn_data.fdyH_R = 0.0; // human right end effector (y-velocity)
    Human_dyn_data.fdzH_R = 0.0; // human right end effector (z-velocity)
    Human_dyn_data.fdxH_L = 0.0; // human left end effector (x-velocity)
    Human_dyn_data.fdyH_L = 0.0; // human left end effector (y-velocity)
    Human_dyn_data.fdzH_L = 0.0; // human left end effector (z-velocity)

    // Haptic feedback to human pilot
    Human_dyn_data.FxH_hmi = 0.0;
    Human_dyn_data.FyH_hmi = 0.0;
    Human_dyn_data.FxH_spring = 0.0;

}

void dash_init::SRB_Init(VectorXd& x0, MatrixXd& q0, MatrixXd& qd0, MatrixXd& lfv0, MatrixXd& lfdv0, VectorXd& u0, SRB_Params srb_params, Human_params human_params)
{
    // This function initializes the robot dynamics for the SRB
    // This includes SRB states (x0), leg joints (q0, qd0), line foot dynamics
    // (lfv0, lfdv0), and control inputs (u0)

    // Parameters
    double hR = srb_params.hLIP;                    // nominal robot LIP height
    double CoM2H_z_dist = srb_params.CoM2H_z_dist;  // CoM to hip connection z-direction distance in m
    double thigh_length = srb_params.thigh_length;  // thigh length in m (L1)
    double calf_length = srb_params.calf_length;    // calf length in m (L2)
    double foot_length = srb_params.foot_length;    // foot length in m (L3)
    double heel_length = srb_params.heel_length;    // heel length in m (L4)
    double init_type = srb_params.init_type;        // initialization type

    // initialize CoM position and velocity to zero
    Vector3d pc_init = MatrixXd::Zero(3, 1);
    Vector3d dpc_init = MatrixXd::Zero(3, 1);

    //orientation of the torso is at the home position (relative to the world frame)
    Matrix3d R_init_mat = Matrix3d::Identity();
    VectorXd R_init = Eigen::Map<VectorXd>(R_init_mat.data(), R_init_mat.cols()*R_init_mat.rows());
    Vector3d EA_init = dash_utils::calc_EA(R_init_mat);
    
    // initialize angular velocity to zero
    Vector3d wb_init = MatrixXd::Zero(3, 1);

    // initial conditions vector
    x0.segment<3>(0) = pc_init;
    x0.segment<3>(3) = dpc_init;
    x0.segment<9>(6) = R_init;
    x0.segment<3>(15) = wb_init;
    x0.segment<3>(18) = EA_init;

    
    if(init_type == 0)
    {
        
        // default initialization places feet underneath hips
        // assumes no hip roll or yaw
        double hip_angle_yaw_init = 0;
        double hip_angle_roll_init = 0; // 0.0472665
        double HK_length = thigh_length;
        double KA_length = calf_length;
        double HA_length = hR - CoM2H_z_dist - heel_length;
        double HKA = acos((HA_length*HA_length - HK_length*HK_length - KA_length*KA_length)/(-2.0*HK_length*KA_length));
        double KHA = acos((KA_length*KA_length - HK_length*HK_length - HA_length*HA_length)/(-2.0*HK_length*HA_length));
        double HAK = acos((HK_length*HK_length - KA_length*KA_length - HA_length*HA_length)/(-2.0*KA_length*HA_length));
        double knee_angle_init = M_PI - HKA;
        double hip_angle_pitch_init = -1.0*KHA;
        double ankle_angle_init = -1.0*HAK;
        VectorXd qr(5);
        qr(0) = hip_angle_yaw_init;
        qr(1) =  hip_angle_roll_init;
        qr(2) = hip_angle_pitch_init;
        qr(3) = knee_angle_init;
        qr(4) = ankle_angle_init;
        VectorXd ql = qr;
        q0.row(0) = qr;
        q0.row(1) = ql;
        q0(1,1) = -q0(1,1);
        
    }
    else if(init_type == 1)
    {
        // teleoperation set up matches normalized feet position of human
        // use IK to generate initial joint angles
        double hH = human_params.hLIP; 
        double pyH_lim_des = human_params.human_nom_ft_width;
        Vector3d lf1R(foot_length/2.0, -1.0*(hR/hH)*pyH_lim_des, -1.0*hR);
        Vector3d lf2R(-1.0*(foot_length/2.0), -1.0*(hR/hH)*pyH_lim_des, -1.0*hR);
        Vector3d lf1L(foot_length/2.0, (hR/hH)*pyH_lim_des, -1.0*hR);
        Vector3d lf2L(-1.0*(foot_length/2.0), (hR/hH)*pyH_lim_des, -1.0*hR);

        lfv0.row(0) = lf1R.transpose(); 
        lfv0.row(1) = lf2R.transpose(); 
        lfv0.row(2) = lf1L.transpose();
        lfv0.row(3) = lf2L.transpose();

        cout << "Initializing Legs for Teleoperation" << endl;
        
        q0 = dash_kin::SRB_IK(srb_params, pc_init, R_init_mat, lfv0);  

    }
    else
    {
        
        q0 = MatrixXd::Zero(2,5);
        cout <<"ERROR: INVALID INIT TYPE" << endl;
    }
    
    // initial joint velocities
    qd0 =  MatrixXd::Zero(2, 5);

    // SRB FK
    MatrixXd torso_vertices(3,8);
    MatrixXd right_leg(5,1);
    MatrixXd left_leg(5,1);
    
    dash_kin::SRB_FK(torso_vertices, right_leg, left_leg, lfv0, srb_params, x0, q0);
    
    // line foot (end-effector) linear velocities (assume no rotation)
    // assume we start at rest
    lfdv0 = MatrixXd::Zero(4, 3); 
    
    // initialize GRFs
    u0 = VectorXd::Zero(12);
}

void dash_init::init_foot_width(double width, Human_params human_params, SRB_Params srb_params, MatrixXd& lfv0, MatrixXd& q0)
{
    // teleoperation set up matches normalized feet position of human
    // use IK to generate initial joint angles
    double foot_length = srb_params.foot_length;    // foot length in m (L3)
    double hH = human_params.hLIP; 
    double hR = srb_params.hLIP; 

    double pyH_lim_des = human_params.human_nom_ft_width;
    Vector3d lf1R(foot_length/2.0, -1.0*(width/2.0), -1.0*hR);
    Vector3d lf2R(-1.0*(foot_length/2.0), -1.0*(width/2.0), -1.0*hR);
    Vector3d lf1L(foot_length/2.0, (width/2.0), -1.0*hR);
    Vector3d lf2L(-1.0*(foot_length/2.0), (width/2.0), -1.0*hR);

    lfv0.row(0) = lf1R.transpose(); 
    lfv0.row(1) = lf2R.transpose(); 
    lfv0.row(2) = lf1L.transpose();
    lfv0.row(3) = lf2L.transpose();

    cout << "Initializing foot width to: " << width << "m "<< endl;

    Vector3d pc_init = MatrixXd::Zero(3, 1);
    Matrix3d R_init_mat = Matrix3d::Identity();
    
    q0 = dash_kin::SRB_IK(srb_params, pc_init, R_init_mat, lfv0);  
}

void dash_init::SRB_params_tello(SRB_Params& srb_params)
{
    if(simulation_mode == 1)
    {
        // simulation 
        srb_params.dt = 0.001; // simulation time step
        srb_params.init_type = 0; // default = 0, teleop_setup = 1

        // physical constants
        srb_params.g = 9.81; // acceleration due to gravity in m/s^2
        srb_params.mu = 1.0; // coefficient of friction value

        // SRB specific
        srb_params.m = 12; // robot mass in kg // was 23 for mujoco // real robot is 15.8Kg
        srb_params.hLIP = 0.58; // nominal robot LIP height // was 0.66 for mujoco
        srb_params.Ib = Matrix3d::Identity();
        srb_params.Ib(0,0) = 0.4874;
        srb_params.Ib(1,1) = 0.3081;
        srb_params.Ib(2,2) = 0.2390; // robot fixed moment of inertia tensor in {B} - Ixx, Iyy, Izz in kg-m^2

        // kinematics
        srb_params.W = 0.252; // body width in m (along y-direction)
        srb_params.L = 0.0840; // body length in m (along x-direction) -- visualization only
        srb_params.H = 0.252; // body height in m (along z-direction) -- visualization only
        srb_params.thigh_length = 0.2286; // thigh length in m (L1)
        srb_params.calf_length = 0.26; // calf length in m (L2)
        srb_params.foot_length = 0.12; // foot length in m (L3)
        srb_params.heel_length = 0.0576; // heel length in m (L4)
        srb_params.CoM2H_z_dist = 0.088; // CoM to hip connection z-direction distance in m was 0.18 for mujoco

        // planner (all) -- perhaps move to a separate data structure later
        srb_params.planner_type = 0; // none = 0, LIP_ang_mom_reg = 1, Human_Dyn_Telelocomotion = 2
        srb_params.T = 0.25; // single-support-phase time (step time)
        // planner_type = none
        srb_params.x_sinu_traj_params << 0.0, 0.0, 0.0; // x-direction sinusoidal reference parameters
        srb_params.y_sinu_traj_params << 0.0, 0.0, 0.0; // y-direction sinusoidal reference parameters
        srb_params.z_sinu_traj_params << 0.0, 0.0, 0.0; // z-direction sinusoidal reference parameters
        srb_params.roll_sinu_traj_params << 0.0, 0.0, 0.0; // roll sinusoidal reference parameters 
        srb_params.pitch_sinu_traj_params << 0.0, 0.0, 0.0; // pitch sinusoidal reference parameters
        srb_params.yaw_sinu_traj_params << 0.0, 0.0, 0.0; // yaw sinusoidal reference parameters
        // planner_type = LIP_ang_mom_reg 
        std::vector<double> vx_des_t;
        for (double t = 0; t <= 5.0; t += 0.1) {
            vx_des_t.push_back(t);
        } 
        Eigen::VectorXd eigen_vx_des_t(vx_des_t.size());
        for (int i = 0; i < vx_des_t.size(); ++i) {
            eigen_vx_des_t[i] = vx_des_t[i];
        }
        srb_params.vx_des_t = eigen_vx_des_t;  // time signal for desired x-direction velocity (end-of-next-step)
        std::vector<double> vx_des_vx;
        for (int i = 0; i < vx_des_t.size(); ++i) {
            double t = vx_des_t[i];
            vx_des_vx.push_back((1.0 / 8.0) * (-1 * (t - 2) * (t - 2) + 4));
        }
        Eigen::VectorXd eigen_vx_des_vx(vx_des_vx.size());
        for (int i = 0; i < vx_des_vx.size(); ++i) {
            eigen_vx_des_vx[i] = vx_des_vx[i];
        }
        srb_params.vx_des_vx = eigen_vx_des_vx; // desired x-direction velocity (end-of-next-step) 
        srb_params.t_beg_stepping = 5; // time to initiate stepping in s
        srb_params.t_end_stepping = 4.5; // time to end stepping in s
        srb_params.zcl = 0.04; // swing-leg max height in m
        // planner_type = Human_Dyn_Telelocomotion 
        srb_params.xDCMH_deadband = 0.10; // deadband for applying gain for human DCM in m
        srb_params.KxDCMH = 1.5; // gain for human DCM
        srb_params.Kx_DCM_mult = 3.0; // multiplier of K_DCM for sagittal plane control
        srb_params.Ky_DCM_mult = 1.5; // multiplier of K_DCM for frontal plane control
        srb_params.T_DSP = 0.0750; // assumed duration of DSP in s
        srb_params.lmaxR = 0.5; // maximum step length in m

        // controller 
        srb_params.Kp_xR = 180.8; // P gain for x-direction tracking
        srb_params.Kd_xR = 121.0; // D gain for x-direction tracking
        srb_params.Kp_yR = 1.8; // P gain for y-direction tracking
        srb_params.Kd_yR = 0.0; // D gain for y-direction tracking
        srb_params.Kp_zR = 180.8; // P gain for z-direction tracking
        srb_params.Kd_zR = 121.0; // D gain for z-direction tracking
        srb_params.Kp_phiR = 11.95; // P gain for roll tracking
        srb_params.Kd_phiR = 4.93; // D gain for roll tracking
        srb_params.Kp_thetaR = 11.23; // P gain for pitch tracking
        srb_params.Kd_thetaR = 3.82; // D gain for pitch tracking
        srb_params.Kp_psiR = 10.95; // P gain for yaw tracking
        srb_params.Kd_psiR = 3.34; // D gain for yaw tracking
        srb_params.QP_opt_sol_type = 2; // quadprog = 0, quadprog (active-set) = 1, qpOASES = 2 --> DEFAULT
        srb_params.W_wrench = 100.0; // cost function weight for satisfying desired net wrench
        srb_params.W_u_minus_u0_norm = 15.0; // cost function weight for penalizing large GRFs that differ too much from the previous (helps with large internal forces that cancel)
        srb_params.Act_const_type = 0; // joint torque based = 0 or motor torque based = 1
        srb_params.tau_m_max = 15.0; // maximum motor torque (15-20)
        srb_params.tau_m_stall = 15.0; // motor stall torque
        srb_params.alpha_m = 0.358; // motor dynamics alpha term
        srb_params.beta_trans = 0.465; // transmission kinematics beta term
        srb_params.gamma_trans = 0.5; // transmission kinematics gamma term
        srb_params.Fz_min_QP = 0.0; // vertical force min to make sure no pulling on the ground (force distribution QP)
        srb_params.Fz_min_FSM = 15.0; // vertical force min to detect when foot breaks contact (FSM) 

        // joint limits
        srb_params.q1_lim << -M_PI/9, M_PI/9;
        srb_params.q2_lim << -M_PI/12, M_PI/9;

        srb_params.swing_time_scaler = 0.6;
        
    }
    else if(simulation_mode == 2)
    {
        // simulation 
        srb_params.dt = 0.002; // simulation time step
        srb_params.init_type = 0; // default = 0, teleop_setup = 1

        // physical constants
        srb_params.g = 9.81; // acceleration due to gravity in m/s^2
        srb_params.mu = 1.0; // coefficient of friction value

        // SRB specific
        srb_params.m = 20.2; // robot mass in kg // was 23 for mujoco
        srb_params.hLIP = 0.58; // nominal robot LIP height // was 0.66 for mujoco
        srb_params.Ib = Matrix3d::Identity();
        srb_params.Ib(0,0) = 0.4874;
        srb_params.Ib(1,1) = 0.3081;
        srb_params.Ib(2,2) = 0.2390; // robot fixed moment of inertia tensor in {B} - Ixx, Iyy, Izz in kg-m^2

        // kinematics
        srb_params.W = 0.252; // body width in m (along y-direction)
        srb_params.L = 0.0840; // body length in m (along x-direction) -- visualization only
        srb_params.H = 0.252; // body height in m (along z-direction) -- visualization only
        srb_params.thigh_length = 0.2286; // thigh length in m (L1)
        srb_params.calf_length = 0.26; // calf length in m (L2)
        srb_params.foot_length = 0.12; // foot length in m (L3)
        srb_params.heel_length = 0.0576; // heel length in m (L4)
        srb_params.CoM2H_z_dist = 0.088; // CoM to hip connection z-direction distance in m

        // planner (all) -- perhaps move to a separate data structure later
        srb_params.planner_type = 0; // none = 0, LIP_ang_mom_reg = 1, Human_Dyn_Telelocomotion = 2
        srb_params.T = 0.25; // single-support-phase time (step time)
        // planner_type = none
        srb_params.x_sinu_traj_params << 0.0, 0.0, 0.0; // x-direction sinusoidal reference parameters
        srb_params.y_sinu_traj_params << 0.0, 0.0, 0.0; // y-direction sinusoidal reference parameters
        srb_params.z_sinu_traj_params << 0.0, 0.0, 0.0; // z-direction sinusoidal reference parameters
        srb_params.roll_sinu_traj_params << 0.0, 0.0, 0.0; // roll sinusoidal reference parameters 
        srb_params.pitch_sinu_traj_params << 0.0, 0.0, 0.0; // pitch sinusoidal reference parameters
        srb_params.yaw_sinu_traj_params << 0.0, 0.0, 0.0; // yaw sinusoidal reference parameters
        // planner_type = LIP_ang_mom_reg 
        std::vector<double> vx_des_t;
        for (double t = 0; t <= 5.0; t += 0.1) {
            vx_des_t.push_back(t);
        } 
        Eigen::VectorXd eigen_vx_des_t(vx_des_t.size());
        for (int i = 0; i < vx_des_t.size(); ++i) {
            eigen_vx_des_t[i] = vx_des_t[i];
        }
        srb_params.vx_des_t = eigen_vx_des_t;  // time signal for desired x-direction velocity (end-of-next-step)
        std::vector<double> vx_des_vx;
        for (int i = 0; i < vx_des_t.size(); ++i) {
            double t = vx_des_t[i];
            vx_des_vx.push_back((1.0 / 8.0) * (-1 * (t - 2) * (t - 2) + 4));
        }
        Eigen::VectorXd eigen_vx_des_vx(vx_des_vx.size());
        for (int i = 0; i < vx_des_vx.size(); ++i) {
            eigen_vx_des_vx[i] = vx_des_vx[i];
        }
        srb_params.vx_des_vx = eigen_vx_des_vx; // desired x-direction velocity (end-of-next-step) 
        srb_params.t_beg_stepping = 5; // time to initiate stepping in s
        srb_params.t_end_stepping = 4.5; // time to end stepping in s
        srb_params.zcl = 0.04; // swing-leg max height in m
        // planner_type = Human_Dyn_Telelocomotion 
        srb_params.xDCMH_deadband = 0.05; // deadband for applying gain for human DCM in m
        srb_params.KxDCMH = 1.0; // gain for human DCM
        srb_params.Kx_DCM_mult = 3.0; // multiplier of K_DCM for sagittal plane control
        srb_params.Ky_DCM_mult = 3.0; // multiplier of K_DCM for frontal plane control
        srb_params.T_DSP = 0.005; // assumed duration of DSP in s
        srb_params.lmaxR = 0.5; // maximum step length in m

        // controller 
        srb_params.Kp_xR = 180.8; // P gain for x-direction tracking
        srb_params.Kd_xR = 121.0; // D gain for x-direction tracking
        srb_params.Kp_yR = 180.8; // P gain for y-direction tracking
        srb_params.Kd_yR = 121.0; // D gain for y-direction tracking
        srb_params.Kp_zR = 180.8; // P gain for z-direction tracking
        srb_params.Kd_zR = 121.0; // D gain for z-direction tracking
        srb_params.Kp_phiR = 11.95; // P gain for roll tracking
        srb_params.Kd_phiR = 4.93; // D gain for roll tracking
        srb_params.Kp_thetaR = 11.23; // P gain for pitch tracking
        srb_params.Kd_thetaR = 3.82; // D gain for pitch tracking
        srb_params.Kp_psiR = 10.95; // P gain for yaw tracking
        srb_params.Kd_psiR = 3.34; // D gain for yaw tracking
        srb_params.QP_opt_sol_type = 2; // quadprog = 0, quadprog (active-set) = 1, qpOASES = 2 --> DEFAULT
        srb_params.W_wrench = 100.0; // cost function weight for satisfying desired net wrench
        srb_params.W_u_minus_u0_norm = 1.0; // cost function weight for penalizing large GRFs that differ too much from the previous (helps with large internal forces that cancel)
        srb_params.Act_const_type = 0; // joint torque based = 0 or motor torque based = 1
        srb_params.tau_m_max = 15.0; // maximum motor torque (15-20)
        srb_params.tau_m_stall = 15.0; // motor stall torque
        srb_params.alpha_m = 0.358; // motor dynamics alpha term
        srb_params.beta_trans = 0.465; // transmission kinematics beta term
        srb_params.gamma_trans = 0.5; // transmission kinematics gamma term
        srb_params.Fz_min_QP = 0.0; // vertical force min to make sure no pulling on the ground (force distribution QP)
        srb_params.Fz_min_FSM = 0.1; // vertical force min to detect when foot breaks contact (FSM) 

        // joint limits
        srb_params.q1_lim << -M_PI/9, M_PI/9;
        srb_params.q2_lim << -M_PI/12, M_PI/9;

    }
    else // Hardware mode
    {
        // simulation 
        srb_params.dt = 0.001; // simulation time step
        srb_params.init_type = 0; // default = 0, teleop_setup = 1

        // physical constants
        srb_params.g = 9.81; // acceleration due to gravity in m/s^2
        srb_params.mu = 1.0; // coefficient of friction value

        // SRB specific
        if(USING_ARMS)
            srb_params.m = 16.8+5.85;//6.85; // robot mass in kg // was 23 for mujoco // real robot is 16.6Kg
        else
            srb_params.m = 16.8; // robot mass in kg // was 23 for mujoco // real robot is 16.6Kg


        if(USING_ARMS)
            srb_params.hLIP = 0.627; // nominal robot LIP height //COM_HEIGHT
        else
            srb_params.hLIP = 0.58; // nominal robot LIP height //COM_HEIGHT
        
        srb_params.Ib = Matrix3d::Identity();
        srb_params.Ib(0,0) = 0.4874;
        srb_params.Ib(1,1) = 0.3081;
        srb_params.Ib(2,2) = 0.2390; // robot fixed moment of inertia tensor in {B} - Ixx, Iyy, Izz in kg-m^2

        // kinematics
        srb_params.W = 0.252; // body width in m (along y-direction)
        srb_params.L = 0.0840; // body length in m (along x-direction) -- visualization only
        srb_params.H = 0.252; // body height in m (along z-direction) -- visualization only
        srb_params.thigh_length = 0.2286; // thigh length in m (L1)
        srb_params.calf_length = 0.26; // calf length in m (L2)
        srb_params.foot_length = 0.12; // foot length in m (L3)
        srb_params.heel_length = 0.0576; // heel length in m (L4)
        if(USING_ARMS)
            srb_params.CoM2H_z_dist = 0.135;// 0.135 <--with arms //0.088; // CoM to hip connection z-direction distance in m was 0.18 for mujoco
        else
            srb_params.CoM2H_z_dist = 0.088;
        // planner (all) -- perhaps move to a separate data structure later
        srb_params.planner_type = 0; // none = 0, LIP_ang_mom_reg = 1, Human_Dyn_Telelocomotion = 2
        srb_params.T = 0.250; // single-support-phase time (step time)
        // planner_type = none
        srb_params.x_sinu_traj_params << 0.0, 0.0, 0.0; // x-direction sinusoidal reference parameters
        srb_params.y_sinu_traj_params << 0.0, 0.0, 0.0; // y-direction sinusoidal reference parameters
        srb_params.z_sinu_traj_params << 0.0, 0.0, 0.0; // z-direction sinusoidal reference parameters
        srb_params.roll_sinu_traj_params << 0.0, 0.0, 0.0; // roll sinusoidal reference parameters 
        srb_params.pitch_sinu_traj_params << 0.0, 0.0, 0.0; // pitch sinusoidal reference parameters
        srb_params.yaw_sinu_traj_params << 0.0, 0.0, 0.0; // yaw sinusoidal reference parameters
        // planner_type = LIP_ang_mom_reg 
        std::vector<double> vx_des_t;
        for (double t = 0; t <= 5.0; t += 0.1) {
            vx_des_t.push_back(t);
        } 
        Eigen::VectorXd eigen_vx_des_t(vx_des_t.size());
        for (int i = 0; i < vx_des_t.size(); ++i) {
            eigen_vx_des_t[i] = vx_des_t[i];
        }
        srb_params.vx_des_t = eigen_vx_des_t;  // time signal for desired x-direction velocity (end-of-next-step)
        std::vector<double> vx_des_vx;
        for (int i = 0; i < vx_des_t.size(); ++i) {
            double t = vx_des_t[i];
            vx_des_vx.push_back((1.0 / 8.0) * (-1 * (t - 2) * (t - 2) + 4));
        }
        Eigen::VectorXd eigen_vx_des_vx(vx_des_vx.size());
        for (int i = 0; i < vx_des_vx.size(); ++i) {
            eigen_vx_des_vx[i] = vx_des_vx[i];
        }
        srb_params.vx_des_vx = eigen_vx_des_vx; // desired x-direction velocity (end-of-next-step) 
        srb_params.t_beg_stepping = 5; // time to initiate stepping in s
        srb_params.t_end_stepping = 4.5; // time to end stepping in s
        srb_params.zcl = 0.02; // swing-leg max height in m
        
        
        // planner_type = Human_Dyn_Telelocomotion 
        srb_params.xDCMH_deadband = 0.10; // deadband for applying gain for human DCM in m
        srb_params.KxDCMH = 2.0; // gain for human DCM
        srb_params.Kx_DCM_mult = 1.5; // multiplier of K_DCM for sagittal plane control
        srb_params.Ky_DCM_mult = 1.0; // multiplier of K_DCM for frontal plane control
        srb_params.T_DSP = 0.0750; // assumed duration of DSP in s
        srb_params.lmaxR = 0.5; // maximum step length in m

        if(USING_ARMS)
        {
            srb_params.Kp_xR = 1000.0; // P gain for x-direction tracking
            srb_params.Kd_xR = 10.0; // D gain for x-direction tracking
            srb_params.Kp_yR = 1500.0; // P gain for y-direction tracking
            srb_params.Kd_yR = 50.0; // D gain for y-direction tracking
            srb_params.Kp_zR = 3500.0; // P gain for z-direction tracking
            srb_params.Kd_zR = 1.0; // D gain for z-direction tracking

            srb_params.Kp_phiR = 300.00; // P gain for roll tracking
            srb_params.Kd_phiR = 2.0; // D gain for roll tracking
            srb_params.Kp_thetaR = 400.00; // P gain for pitch tracking
            srb_params.Kd_thetaR = 2.0; // D gain for pitch tracking
            srb_params.Kp_psiR = 50.00; // P gain for yaw tracking
            srb_params.Kd_psiR = 2.0; // D gain for yaw tracking // TODO: unstable, increase?
        }
        else
        {
            srb_params.Kp_xR = 500.0; // P gain for x-direction tracking
            srb_params.Kd_xR = 5.0; // D gain for x-direction tracking
            srb_params.Kp_yR = 1500.0; // P gain for y-direction tracking
            srb_params.Kd_yR = 50.0; // D gain for y-direction tracking
            srb_params.Kp_zR = 3500.0; // P gain for z-direction tracking
            srb_params.Kd_zR = 1.0; // D gain for z-direction tracking

            srb_params.Kp_phiR = 500.00; // P gain for roll tracking
            srb_params.Kd_phiR = 5.0; // D gain for roll tracking
            srb_params.Kp_thetaR = 600.00; // P gain for pitch tracking
            srb_params.Kd_thetaR = 5.0; // D gain for pitch tracking
            srb_params.Kp_psiR = 50.00; // P gain for yaw tracking
            srb_params.Kd_psiR = 5.0; // D gain for yaw tracking // TODO: unstable, increase?
        }

        // controller 
        
  
        // srb_params.xDCMH_deadband = 0.10; // deadband for applying gain for human DCM in m
        // srb_params.KxDCMH = 2.0; // gain for human DCM
        // srb_params.Kx_DCM_mult = 1.0; // multiplier of K_DCM for sagittal plane control
        // srb_params.Ky_DCM_mult = 1.0; // multiplier of K_DCM for frontal plane control
        // srb_params.T_DSP = 0.0750; // assumed duration of DSP in s
        // srb_params.lmaxR = 0.5; // maximum step length in m

        // // controller 
        // srb_params.Kp_xR = 200.0; // P gain for x-direction tracking
        // srb_params.Kd_xR = 2.0; // D gain for x-direction tracking
        // srb_params.Kp_yR = 600.0; // P gain for y-direction tracking
        // srb_params.Kd_yR = 2.0; // D gain for y-direction tracking
        // srb_params.Kp_zR = 2000.0; // P gain for z-direction tracking
        // srb_params.Kd_zR = 10.0; // D gain for z-direction tracking

        // srb_params.Kp_phiR = 100.00; // P gain for roll tracking
        // srb_params.Kd_phiR = 2.0; // D gain for roll tracking
        // srb_params.Kp_thetaR = 400.00; // P gain for pitch tracking
        // srb_params.Kd_thetaR = 2.0; // D gain for pitch tracking
        // srb_params.Kp_psiR = 50.00; // P gain for yaw tracking
        // srb_params.Kd_psiR = 1.0; // D gain for yaw tracking

        srb_params.QP_opt_sol_type = 2; // quadprog = 0, quadprog (active-set) = 1, qpOASES = 2 --> DEFAULT
        srb_params.W_wrench = 100.0; // cost function weight for satisfying desired net wrench
        srb_params.W_u_minus_u0_norm = 1.0; // cost function weight for penalizing large GRFs that differ too much from the previous (helps with large internal forces that cancel)
        srb_params.Act_const_type = 0; // joint torque based = 0 or motor torque based = 1
        srb_params.tau_m_max = 15.0; // maximum motor torque (15-20)
        srb_params.tau_m_stall = 15.0; // motor stall torque
        srb_params.alpha_m = 0.358; // motor dynamics alpha term
        srb_params.beta_trans = 0.465; // transmission kinematics beta term
        srb_params.gamma_trans = 0.5; // transmission kinematics gamma term
        srb_params.Fz_min_QP = 0.0; // vertical force min to make sure no pulling on the ground (force distribution QP)
        srb_params.Fz_min_FSM = 10; // vertical force min to detect when foot breaks contact (FSM) 

        // joint limits
        srb_params.q1_lim << -M_PI/9, M_PI/9;
        srb_params.q2_lim << -M_PI/12, M_PI/9;

        srb_params.swing_time_scaler = 0.6;
        
    }
}