#define WITHOUT_NUMPY
#define GL_GLEXT_PROTOTYPES
#include "mujoco_main.h"
#include "mujoco_utilities.h"
#include "state_estimator.h"
#include "utilities.h"
#include <X11/Xlib.h>
#include <regex>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include "mocap.h"

pthread_mutex_t plotting_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sim_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sim_step_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tello_ctrl_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tau_share_mutex = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t EKF_mutex;

namespace plt = matplotlibcpp;
GLFWwindow* window;
int windowWidth, windowHeight;
extern float separator_thickness;

FILE* screen_record_pipe;
pid_t screen_rec_pid = -1;
bool recording_in_progress = false;
bool usb_recording_in_progress = false;
bool usb_recording_HW_in_progress = false;

extern struct termios originalSettings;

simConfig sim_conf;
int plot_width = 0;

double vx_desired_ps4 = 0;
double vy_desired_ps4 = 0;
bool PS4_connected = false;
bool zero_human = false;
bool zero_arms = false;
float master_gain = 0;
bool screen_recording = false;
bool usbcam_recording = false;
bool usbcam_hw_recording = false;
bool en_v2_ctrl = false;
bool en_safety_monitor = false;
bool bookmarked = false;
bool showCopyErrorPopup = false;
bool init_foot_width = false;
bool playback_error = false;
userProfile activeUser;
int hdd_cnt=0; // human_playback counter
std::string active_playback_log;
std::string active_playback_log_png;
int active_playback_log_index = 0;
std::vector<std::string> hddFiles;
bool showPlotMenu = false;
bool showTuningMenu = false;
bool playback_changed = false;
bool playback_chosen = false;
bool auto_mode = false;
bool start_target_motion = false;
bool ramp_toggle = false;

double human_x_zero = 0;

double playback_foot_width = 0.175;

bool sim_window_close_requested = false;

bool cd_shared_data_ready = false;

extern double FxH_hmi_out, FxH_spring_out, FyH_hmi_out;

//logging
bool log_data_ready = false;
bool sim_step_completed = false;
std::string log_folder;
VectorXd x_out, u_out, q_out, qd_out,full_tau_out, tau_out, tau_ext_out, lfv_out, lfdv_out,lfv_comm_out,lfdv_comm_out, t_n_FSM_out, impulse_out, meas_grf_out, xDCM_out, yDCM_out;
VectorXd target_pos_out = Vector3d(0,0,0);
VectorXd target_vel_out = Vector3d(0,0,0);
double last_log_time = -1;
Human_dyn_data hdd_out;
Traj_planner_dyn_data tpdd_out;


extern RoboDesignLab::DynamicRobot* tello;
inekf::RobotState filter_state;
extern MatrixXd lfv_dsp_start;
Matrix3d rotation_mat;
MatrixXd mujoco_lfv(4,3);
VectorXd EA_curr;
VectorXd dEA_curr;
VectorXd pc_curr;
VectorXd dpc_curr;
VectorXd kp_vec_joint_swing(10);
VectorXd kd_vec_joint_swing(10);
VectorXd kp_vec_joint_posture(10);
VectorXd kd_vec_joint_posture(10);
Joint_PD_config swing_conf, posture_conf;
RoboDesignLab::TaskPDConfig swing_pd_config, posture_pd_config;
extern bool filter_data_ready;
int render_cnt = 0;
int pos_sample_index = 0;

VectorXd left_toe_vel_mj(6), right_toe_vel_mj(6), left_heel_vel_mj(6), right_heel_vel_mj(6);

uint64_t render_counter = 0;

int camera_cnt = 0;

char error[1000];

double sim_time;
bool pause_sim = true;
double stepping_in_progress = true;
double CoM_z_last = 0;
Vector4d z_plotting;
double x_prev = 0;
double x_vel;
VectorXd x_vels = VectorXd(100);
double y_prev = 0;
double y_vel;
VectorXd y_vels = VectorXd(100);
double z_prev = 0;
double z_vel;
int vel_index = 0;
VectorXd z_vels = VectorXd(100);
double dx_smoothed;
double dy_smoothed;
double dz_smoothed;
double smoothFactor = 4;

VectorXd torso_vels = VectorXd(2000);

VectorXd gnd_contacts(4);
VectorXd z_forces(4);

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mjModel* m_shared = NULL;                  // MuJoCo model
// mjData* d_shared = NULL;                   // MuJoCo data
VectorXd tau_shared = VectorXd::Zero(10);
bool tau_ready = false;
ctrlData cd_shared;

double push_force_x = 0;
double push_force_y = 0;
double push_force_z = 0;
mjtNum push_force[3];
bool impulse_scheduled = false;
bool impulse_active = false;
double impulse_start_time = 0;
bool ball_throw_scheduled = false;
double impulse_force_newtons = 10;

bool sim_was_restarted = false;
extern int simulation_mode;

bool curve_fitting_complete = false;
bool data_ready_for_curve_fitting = false;
bool ready_for_new_curve_fit_data = true;
bool use_adaptive_step_time = true;
double dtime = 0;

extern int sockfd_tx;
extern char hmi_tx_buffer[100];
extern struct sockaddr_in servaddr_tx;

extern double robot_init_foot_width;

Vector3d CoM_pos = Vector3d::Zero();
Vector3d CoM_rpy = Vector3d::Zero();
Vector3d CoM_vel = Vector3d::Zero();
Vector3d CoM_rpy_vel = Vector3d::Zero();
Vector3d CoM_acc = Vector3d::Zero();
VectorXd CoM_quat = VectorXd::Zero(4);

extern long long hmi_comms_counter;
bool hmi_connected = false;

// begin SRBM-Ctrl Variables here ================================================================

SRBMController* controller;
MatrixXd lfv0(4,3), lfdv0(4,3); // global so planner can access them for now

// end SRBM-Ctrl Variables here ============================================================

VectorXd arm_joint_pos_desired(8);
extern VectorXd R_joystick_enc;
extern VectorXd L_joystick_enc;

// Define variables here so code runs faster:

void set_mujoco_state(VectorXd x)
{
    Vector3d pc = x.segment(0, 3);
    Vector3d dpc = x.segment(3, 3);
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d EA = dash_utils::calc_EA(R_curr);
    Vector3d wb_curr = x.segment(15, 3);
    Vector3d dEA = dash_utils::calc_dEA(R_curr,wb_curr);
    MatrixXd q = tello->controller->get_q();
    MatrixXd qd = tello->controller->get_qd();

    // Get robot states
    d->qpos[torso_x_idx] = pc(0);
    d->qvel[torso_x_idx] = dpc(0);
    d->qpos[torso_y_idx] = pc(1);
    d->qvel[torso_y_idx] = dpc(1);
    d->qpos[torso_z_idx] = pc(2);
    d->qvel[torso_z_idx] = dpc(2);

    d->qpos[torso_roll_idx] = EA(0);
    d->qvel[torso_roll_idx] = dEA(0);
    d->qpos[torso_pitch_idx] = EA(1);
    d->qvel[torso_pitch_idx] = dEA(1);
    d->qpos[torso_yaw_idx] = EA(2);
    d->qvel[torso_yaw_idx] = dEA(2);

    d->qpos[hip_yaw_l_idx] = q(1,0);
    d->qpos[hip_roll_l_idx] = q(1,1);
    d->qpos[hip_pitch_l_idx] = q(1,2);
    d->qpos[knee_pitch_l_idx] = q(1,3);
    d->qpos[ankle_pitch_l_idx] = q(1,4);
    d->qpos[hip_yaw_r_idx] = q(0,0);
    d->qpos[hip_roll_r_idx] = q(0,1);
    d->qpos[hip_pitch_r_idx] = q(0,2);
    d->qpos[knee_pitch_r_idx] = q(0,3);
    d->qpos[ankle_pitch_r_idx] = q(0,4);

    d->qvel[hip_yaw_l_idx] = qd(1,0);
    d->qvel[hip_roll_l_idx] = qd(1,1);
    d->qvel[hip_pitch_l_idx] = qd(1,2);             
    d->qvel[knee_pitch_l_idx] = qd(1,3);   
    d->qvel[ankle_pitch_l_idx] = qd(1,4);    
    d->qvel[hip_yaw_r_idx] = qd(0,0);
    d->qvel[hip_roll_r_idx] = qd(0,1);
    d->qvel[hip_pitch_r_idx] = qd(0,2);             
    d->qvel[knee_pitch_r_idx] = qd(0,3);   
    d->qvel[ankle_pitch_r_idx] = qd(0,4);
}

void dummy_callback(const mjModel* m, mjData* d)
{
}

ctrlData copyMjData(const mjModel* m, mjData*d)
{
    ctrlData cd;
    cd.t = d->time;
    cd.xR = d->qpos[torso_x_idx];
    cd.xdR = d->qvel[torso_x_idx];    
    cd.yR = d->qpos[torso_y_idx];
    cd.ydR = d->qvel[torso_y_idx];
    cd.zR = d->qpos[torso_z_idx];
    cd.zdR = d->qvel[torso_z_idx];
    cd.phiR = d->qpos[torso_roll_idx];
    cd.phidR = d->qvel[torso_roll_idx];   
    cd.thetaR = d->qpos[torso_pitch_idx];
    cd.thetadR = d->qvel[torso_pitch_idx];
    cd.psiR = d->qpos[torso_yaw_idx];
    cd.psidR = d->qvel[torso_yaw_idx];  
    cd.q1l = d->qpos[hip_yaw_l_idx];
    cd.q2l = d->qpos[hip_roll_l_idx];
    cd.q3l = d->qpos[hip_pitch_l_idx];             
    cd.q4l = d->qpos[knee_pitch_l_idx];   
    cd.q5l = d->qpos[ankle_pitch_l_idx];    
    cd.q1r = d->qpos[hip_yaw_r_idx];
    cd.q2r = d->qpos[hip_roll_r_idx];
    cd.q3r = d->qpos[hip_pitch_r_idx];             
    cd.q4r = d->qpos[knee_pitch_r_idx];   
    cd.q5r = d->qpos[ankle_pitch_r_idx];  
    cd.qd1l = d->qvel[hip_yaw_l_idx];
    cd.qd2l = d->qvel[hip_roll_l_idx];
    cd.qd3l = d->qvel[hip_pitch_l_idx];             
    cd.qd4l = d->qvel[knee_pitch_l_idx];   
    cd.qd5l = d->qvel[ankle_pitch_l_idx];    
    cd.qd1r = d->qvel[hip_yaw_r_idx];
    cd.qd2r = d->qvel[hip_roll_r_idx];
    cd.qd3r = d->qvel[hip_pitch_r_idx];             
    cd.qd4r = d->qvel[knee_pitch_r_idx];   
    cd.qd5r = d->qvel[ankle_pitch_r_idx];

    mjtNum left_foot_toe[3];
    mjtNum left_foot_heel[3];
    mjtNum right_foot_toe[3];
    mjtNum right_foot_heel[3];
    const char* lft = "lft";
    const char* lfh = "lfh";
    const char* rft = "rft";
    const char* rfh = "rfh";
    int geom_id = mj_name2id(m, mjOBJ_GEOM, lft);
    memcpy(left_foot_toe, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
    geom_id = mj_name2id(m, mjOBJ_GEOM, lfh);
    memcpy(left_foot_heel, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
    geom_id = mj_name2id(m, mjOBJ_GEOM, rft);
    memcpy(right_foot_toe, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
    geom_id = mj_name2id(m, mjOBJ_GEOM, rfh);
    memcpy(right_foot_heel, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));

    // cd.mujoco_lfv.row(0) = Vector3d(right_foot_toe[0],right_foot_toe[1],right_foot_toe[2]);
    // cd.mujoco_lfv.row(1) = Vector3d(right_foot_heel[0],right_foot_heel[1],right_foot_heel[2]);
    // cd.mujoco_lfv.row(2) = Vector3d(left_foot_toe[0],left_foot_toe[1],left_foot_toe[2]);
    // cd.mujoco_lfv.row(3) = Vector3d(left_foot_heel[0],left_foot_heel[1],left_foot_heel[2]);

    int accel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "torso-linear-acceleration");
    int gyro_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "toso-angular-velocity");

    mju_copy3(cd.acceleration, &d->sensordata[accel_sensor_id]);
    mju_copy3(cd.angular_velocity, &d->sensordata[gyro_sensor_id]);

    return cd;
}

void setNaNtoZeroSim(Eigen::VectorXd& vec) {
    for (int i = 0; i < vec.size(); ++i) {
        if (std::isnan(vec(i))) {
            vec(i) = 0.0;
        }
    }
}

// void TELLO_locomotion_ctrl(const mjModel* m, mjData* d)
void TELLO_locomotion_ctrl(ctrlData cd)
{
    tello->_GRFs.right_front = cd.grf_rf;
    tello->_GRFs.right_back = cd.grf_rb;
    tello->_GRFs.left_front = cd.grf_lf;
    tello->_GRFs.left_back = cd.grf_lb;

    double t_end_stepping;
    controller->set_time(cd.t);

    // Vector3d acc_no_g = subtractG(Vector3d(psiR,thetaR,phiR),Vector3d(acceleration[0],acceleration[1],acceleration[2]));
    Vector3d imu_acc = Vector3d(cd.acceleration[0],cd.acceleration[1],cd.acceleration[2]);
    Vector3d imu_gyro = Vector3d(cd.angular_velocity[0],cd.angular_velocity[1],cd.angular_velocity[2]);

	// Torso state vectors
    VectorXd SRB_q(6);
    VectorXd SRB_qd(6);
    SRB_q << cd.xR, cd.yR, cd.zR, cd.phiR, cd.thetaR, cd.psiR;
    SRB_qd << cd.xdR, cd.ydR, cd.zdR, cd.phidR, cd.thetadR, cd.psidR;
    //pthread_mutex_lock(&plotting_mutex);
    // CoM vector
    pc_curr = SRB_q.head(3);
    dpc_curr = SRB_qd.head(3);
    MatrixXd pcom_row(1, 3);
    pcom_row = pc_curr.transpose();
    MatrixXd pcom_mat(4, 3);
    pcom_mat = pcom_row.replicate(4, 1);     

    // Leg joints and parameters vectors
    VectorXd qLeg_l(5);
    qLeg_l << cd.q1l, cd.q2l, cd.q3l, cd.q4l, cd.q5l;
    VectorXd qLeg_r(5);
    qLeg_r << cd.q1r, cd.q2r, cd.q3r, cd.q4r, cd.q5r;

    VectorXd qdLeg_l(5);
    qdLeg_l << cd.qd1l, cd.qd2l, cd.qd3l, cd.qd4l, cd.qd5l;
    VectorXd qdLeg_r(5);
    qdLeg_r << cd.qd1r, cd.qd2r, cd.qd3r, cd.qd4r, cd.qd5r;

    // Compute rotation matrix from world to body frame
    Matrix3d Rwb;
    Rwb = AngleAxisd(cd.phiR, Vector3d::UnitX())
        * AngleAxisd(cd.thetaR, Vector3d::UnitY())
        * AngleAxisd(cd.psiR, Vector3d::UnitZ());
    rotation_mat = Rwb;

    // add gravity to IMU:
    VectorXd imu_acc_world = Rwb*imu_acc;
    imu_acc_world += Vector3d(0,0,9.81);

    tello->controller->set_ddpc_world(imu_acc_world);

    imu_acc = Rwb.transpose()*imu_acc_world;

    tello->_acc = imu_acc;
    tello->_gyro = imu_gyro;

	// Fill SRBM-Ctrl variables with Mujoco Variables here:
    MatrixXd q(2,5);
    MatrixXd qd(2,5);
	q.row(0) = qLeg_r;
	q.row(1) = qLeg_l;
    qd.row(0) = qdLeg_r;
	qd.row(1) = qdLeg_l;
	Eigen::Map<Eigen::VectorXd> R_curr(Rwb.data(), Rwb.size());
	EA_curr = Vector3d(cd.phiR,cd.thetaR,cd.psiR);
    dEA_curr = Vector3d(cd.phidR,cd.thetadR,cd.psidR);
    VectorXd wb_curr = dash_utils::calc_wb(dEA_curr,EA_curr);

    VectorXd qVec(10), qdVec(10);
    qVec << q.row(1).transpose(), q.row(0).transpose();
    qdVec << qd.row(1).transpose(), qd.row(0).transpose();

    tello->sim_joint_pos << qVec;
    tello->sim_joint_vel << qdVec;

    VectorXd task_velocities = tello->joint_vel_to_task_vel(qdVec,tello->getJointPositions());

    MatrixXd lfdv_hip(4,3);
    lfdv_hip.row(2) = task_velocities.segment<3>(0);
    lfdv_hip.row(3) = task_velocities.segment<3>(3);
    lfdv_hip.row(0) = task_velocities.segment<3>(6);
    lfdv_hip.row(1) = task_velocities.segment<3>(9);
    if(simulation_mode == 1)
    {
        controller->set_lfdv_hip(lfdv_hip);
    }

	// call SRBM-Ctrl here ======================================================================================
    double CoM_z = controller->get_CoM_z(controller->get_lfv_hip(),gnd_contacts,EA_curr); 
    if(CoM_z == -1) CoM_z = CoM_z_last;
    CoM_z_last = CoM_z;

    // Populate Controller inputs with estimated data instead of mujoco given data:
    Vector3d pc_curr_plot = pc_curr;

    Vector3d estimated_pc(filter_state.getPosition()(0),filter_state.getPosition()(1),CoM_z);

    x_vel = (estimated_pc(0) - x_prev)/0.001;
    x_prev = estimated_pc(0);
    y_vel = (estimated_pc(1) - y_prev)/0.001;
    y_prev = estimated_pc(1);
    z_vel = (estimated_pc(2) - z_prev)/0.001;
    z_prev = estimated_pc(2);

    x_vels.tail(99) = x_vels.head(99).eval();
    x_vels[0] = x_vel;
    y_vels.tail(99) = y_vels.head(99).eval();
    y_vels[0] = y_vel;
    z_vels.tail(99) = z_vels.head(99).eval();
    z_vels[0] = z_vel;

    dx_smoothed = smoothData(x_vels,3);
    dy_smoothed = smoothData(y_vels,3);
    dz_smoothed = smoothData(z_vels,3);

    // Vector3d estimated_dpc(dx_smoothed,dy_smoothed,dz_smoothed);
    
    VectorXd tau = controller->update(pc_curr, dpc_curr, EA_curr, dEA_curr,q ,qd ,cd.t);

    t_end_stepping = controller->get_SRB_params().t_end_stepping;
    if(tello->controller->get_sim_mode() == 2)
    {
        // do nothing for srbm sim
    }
    else
    {

        Vector3d target_front_left = controller->get_lfv_comm_hip().row(2);
        Vector3d target_back_left = controller->get_lfv_comm_hip().row(3);
        Vector3d target_front_right = controller->get_lfv_comm_hip().row(0);
        Vector3d target_back_right = controller->get_lfv_comm_hip().row(1);

        Vector3d target_front_left_vel = controller->get_lfdv_comm_hip().row(2);
        Vector3d target_back_left_vel = controller->get_lfdv_comm_hip().row(3);
        Vector3d target_front_right_vel = controller->get_lfdv_comm_hip().row(0);
        Vector3d target_back_right_vel = controller->get_lfdv_comm_hip().row(1);

        Vector3d target_front_left_accel = controller->get_lfddv_comm_hip().row(2);
        Vector3d target_back_left_accel = controller->get_lfddv_comm_hip().row(3);
        Vector3d target_front_right_accel = controller->get_lfddv_comm_hip().row(0);
        Vector3d target_back_right_accel = controller->get_lfddv_comm_hip().row(1);

        VectorXd vel_desired(12);
        vel_desired  << target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel;

        VectorXd pos_desired(12);
        pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

        VectorXd leg_inertia(5);
        leg_inertia <<  0.0111, // straight leg (hip + thigh + shin + foot) inertia seen by hip yaw
                        0.0926, // straight leg (thigh + shin + foot) inertia seen by hip roll
                        0.0913, // straight leg (thigh + shin + foot) inertia seen by hip pitch
                        0.0174, // straight leg (shin + foot) inertia seen by knee
                        0.0030; // foot inertia seen by ankle
        double inertia_accel_gain = 0.0;

        VectorXd swing_leg_torques = VectorXd::Zero(10);
        // if(abs(tello->controller->get_FSM()) == 1)
        // {
            // Set up configuration struct for Task Space Controller
            swing_pd_config.use_single_jacoian = false;
            swing_pd_config.side = BOTH_LEGS;
            // if(tello->controller->get_FSM() == 1)
            //     swing_pd_config.side = RIGHT_LEG;
            // if(tello->controller->get_FSM() == -1)
            //     swing_pd_config.side = LEFT_LEG;

            swing_pd_config.ignore_joint_velocity = false;
            swing_pd_config.task_ff_force = VectorXd::Zero(12);
            swing_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
            swing_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
            swing_pd_config.setTaskKp(0,0,0);
            swing_pd_config.setTaskKd(0,0,0);
            // swing_pd_config.setJointKa(swing_conf.hip_yaw_Ka,swing_conf.hip_roll_Ka,swing_conf.hip_pitch_Ka,swing_conf.knee_Ka,swing_conf.ankle_Ka);
            swing_pd_config.setJointKa(0/*leg_inertia*inertia_accel_gain*/);
            swing_pd_config.setFFAccel(target_front_left_accel,target_back_left_accel,target_front_right_accel,target_back_right_accel);
            swing_pd_config.setJointKp(kp_vec_joint_swing);
            swing_pd_config.setJointKd(kd_vec_joint_swing);
            swing_pd_config.motor_kp = VectorXd::Zero(10);
            swing_pd_config.motor_kd = VectorXd::Zero(10);
            swing_leg_torques = tello->taskPD2(swing_pd_config);

            // if( abs(tello->controller->get_FSM()) == 1)
            // {
            //     cout << "comm: " << endl;
            //     cout << tello->controller->get_lfv_comm_world() << endl;
            //     cout << endl;
            //     cout << "=============================================================================" << endl;
            //     cout << endl;
            //     cout << "real: " << endl;
            //     cout << tello->controller->get_lfv_world() << endl;
            //     cout << endl;
            //     cout << "=============================================================================" << endl;
            //     cout << endl;

            // }
            

            // if( abs(tello->controller->get_FSM()) == 1)
            // {
            //     printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
            //             swing_leg_torques(0), swing_leg_torques(1), swing_leg_torques(2),
            //             swing_leg_torques(3), swing_leg_torques(4), swing_leg_torques(5),
            //             swing_leg_torques(6), swing_leg_torques(7), swing_leg_torques(8),
            //             swing_leg_torques(9));
            // }

            // setNaNtoZeroSim(swing_leg_torques);
        // }

        posture_pd_config = swing_pd_config;
        posture_pd_config.ignore_joint_velocity = true;
        posture_pd_config.side = BOTH_LEGS;
        // if(tello->controller->get_FSM() == -1)
        //     posture_pd_config.side = RIGHT_LEG;
        // if(tello->controller->get_FSM() == 1)
        //     posture_pd_config.side = LEFT_LEG;
        // Re-using
        posture_pd_config.task_ff_force = VectorXd::Zero(12);
        posture_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
        posture_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
        posture_pd_config.setTaskKp(0,0,0);
        posture_pd_config.setTaskKd(0,0,0);
        posture_pd_config.setTaskKa(0,0,0);
        posture_pd_config.setJointKp(kp_vec_joint_posture);
        posture_pd_config.setJointKd(kd_vec_joint_posture);

        VectorXd posture_ctrl_torques = tello->taskPD2(posture_pd_config);

        // setNaNtoZeroSim(posture_ctrl_torques);
        
        // END TASK PD CODE ======================================+++++++++++++++++
        VectorXd tau_LR(10);
        tau_LR << tau.tail(5), tau.head(5);

        tau_LR = tau_LR + posture_ctrl_torques;

        // printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
        //     tau_LR(0), tau_LR(1), tau_LR(2),
        //     tau_LR(3), tau_LR(4), tau_LR(5),
        //     tau_LR(6), tau_LR(7), tau_LR(8),
        //     tau_LR(9));

        // printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
        //     posture_ctrl_torques(0), posture_ctrl_torques(1), posture_ctrl_torques(2),
        //     posture_ctrl_torques(3), posture_ctrl_torques(4), posture_ctrl_torques(5),
        //     posture_ctrl_torques(6), posture_ctrl_torques(7), posture_ctrl_torques(8),
        //     posture_ctrl_torques(9));


        VectorXd torques_left  = tello->swing_stance_mux(tau_LR.head(5), swing_leg_torques.head(5),
                                                            0.01,controller->get_isSwingToStanceRight(), 
                                                            cd.t-controller->get_transitionStartRight(), 
                                                            0);
        VectorXd torques_right = tello->swing_stance_mux(tau_LR.tail(5), swing_leg_torques.tail(5),
                                                            0.01,controller->get_isSwingToStanceLeft(),
                                                            cd.t-controller->get_transitionStartLeft(), 
                                                            1);
        VectorXd tau_LR_muxed(10);
        tau_LR_muxed << torques_left,torques_right;
        tau_ready = false;
        pthread_mutex_lock(&tau_share_mutex);
        tau_shared = tau_LR_muxed;
        pthread_mutex_unlock(&tau_share_mutex);
        tau_ready = true;
        
        // tau is now ready to use with either the sim or hardware

    }
    
}
VectorXd x0;
MatrixXd q0;
std::string recording_file_name;

void initializeSRBMCtrl()
{
    controller = tello->controller;
    SRB_Params srb_params = controller->get_SRB_params();
    Traj_planner_dyn_data traj_planner_dyn_data = controller->get_traj_planner_dyn_data();
    Human_params human_params = controller->get_human_params();
    x0 = controller->get_x0();
    q0 = controller->get_q0();
    lfv0 = controller->get_lfv0();
    lfdv0 = controller->get_lfdv0();
    lfv_dsp_start = lfv0;

    if (std::filesystem::is_directory("/home/telloHW")) {
        //std::cout << "The directory /home/tello exists!" << std::endl;
        if(simulation_mode == 1)
        {
            cout << "RUNNING ON TELLO" << endl;
            dash_utils::parse_json_to_srb_params("tello_files/srb_pd_config.json",srb_params);
            dash_utils::parse_json_to_pd_params("tello_files/srb_pd_config.json",swing_conf,posture_conf);
        }
        if(simulation_mode == 2)
        {
            dash_utils::parse_json_to_srb_params("tello_files/srb_pd_config_SRBsim.json",srb_params);
            dash_utils::parse_json_to_pd_params("tello_files/srb_pd_config_SRBsim.json",swing_conf,posture_conf);
        }
    }
    else {
        //std::cout << "The directory /home/tello does not exist!" << std::endl;
        if(simulation_mode == 1)
        {
            dash_utils::parse_json_to_srb_params(activeUser.config_filename,srb_params);
            dash_utils::parse_json_to_pd_params(activeUser.config_filename,swing_conf,posture_conf);
            copyFile(activeUser.config_filename,log_folder);
        }
        if(simulation_mode == 2)
        {
            dash_utils::parse_json_to_srb_params("/home/tello/Documents/GIT/Tello_Software/include/srb_pd_config_SRBsim.json",srb_params);
            dash_utils::parse_json_to_pd_params("/home/tello/Documents/GIT/Tello_Software/include/srb_pd_config_SRBsim.json",swing_conf,posture_conf);
            copyFile("/home/tello/Documents/GIT/Tello_Software/include/srb_pd_config_SRBsim.json",log_folder);
        }
            
    }
    
    //dash_utils::parse_json_to_srb_params("tello_files/srb_pd_config.json",srb_params);
    // dash_utils::parse_json_to_pd_params("tello_files/srb_pd_config.json",swing_conf,posture_conf);

    kp_vec_joint_swing <<   swing_conf.hip_yaw_Kp, swing_conf.hip_roll_Kp,swing_conf.hip_pitch_Kp, swing_conf.knee_Kp, swing_conf.ankle_Kp,
                            swing_conf.hip_yaw_Kp, swing_conf.hip_roll_Kp,swing_conf.hip_pitch_Kp, swing_conf.knee_Kp, swing_conf.ankle_Kp;
    kd_vec_joint_swing <<   swing_conf.hip_yaw_Kd, swing_conf.hip_roll_Kd,swing_conf.hip_pitch_Kd, swing_conf.knee_Kd, swing_conf.ankle_Kd,
                            swing_conf.hip_yaw_Kd, swing_conf.hip_roll_Kd,swing_conf.hip_pitch_Kd, swing_conf.knee_Kd, swing_conf.ankle_Kd;
    kp_vec_joint_posture << posture_conf.hip_yaw_Kp, posture_conf.hip_roll_Kp,posture_conf.hip_pitch_Kp, posture_conf.knee_Kp, posture_conf.ankle_Kp,
                            posture_conf.hip_yaw_Kp, posture_conf.hip_roll_Kp,posture_conf.hip_pitch_Kp, posture_conf.knee_Kp, posture_conf.ankle_Kp;
    kd_vec_joint_posture << posture_conf.hip_yaw_Kd, posture_conf.hip_roll_Kd,posture_conf.hip_pitch_Kd, posture_conf.knee_Kd, posture_conf.ankle_Kd,
                            posture_conf.hip_yaw_Kd, posture_conf.hip_roll_Kd,posture_conf.hip_pitch_Kd, posture_conf.knee_Kd, posture_conf.ankle_Kd;
    cout << "------------------------------------------------------------------------------------------------------------------------------" << endl;
    cout << "Swing Kp Gains: " << kp_vec_joint_swing.transpose() << endl;
    cout << "------------------------------------------------------------------------------------------------------------------------------" << endl;
    cout << "Swing Kd Gains: " << kd_vec_joint_swing.transpose() << endl;
    cout << "------------------------------------------------------------------------------------------------------------------------------" << endl;
    cout << "Posture Kp Gains: " << kp_vec_joint_posture.transpose() << endl;
    cout << "------------------------------------------------------------------------------------------------------------------------------" << endl;
    cout << "Posture Kd Gains: " << kd_vec_joint_posture.transpose() << endl;
    cout << "------------------------------------------------------------------------------------------------------------------------------" << endl;
    // printf("Choose a Test:\n\n");
    // printf("x: lean\n");
    // printf("y: side2side\n");
    // printf("z: squat\n");
    // printf("r: roll\n");
    // printf("p: pitch\n");
    // printf("w: yaw\n");
    // printf("b: balance\n");
    // printf("s: stepping/walking\n");
    // cin.get();
    // char DoF;
    // cin.get(DoF);
    // if(DoF != 's')
    // {
    //     dash_planner::SRB_6DoF_Test(recording_file_name,sim_time,srb_params,lfv0,DoF,1);
    // }
    // else{
    // }
    if(sim_conf.en_autonomous_mode_on_boot)
    {
        auto_mode = true;
        printf("Walking Selected\n\n");
        // Option 2: Walking using LIP angular momentum regulation about contact point
        // user input (walking speed and step frequency)
        double des_walking_speed = srb_params.des_walking_speed;
        // double des_walking_step_period = 0.2;<---- CHANGE IN JSON, NOT HERE
        // end user input
        recording_file_name = "Walking";
        srb_params.planner_type = 1; 
        // srb_params.T = des_walking_step_period;<---- CHANGE IN JSON, NOT HERE
        VectorXd t_traj, v_traj;
        double t_beg_stepping_time, t_end_stepping_time;
        dash_planner::SRB_LIP_vel_traj(des_walking_speed,t_traj,v_traj,t_beg_stepping_time,t_end_stepping_time);
        srb_params.vx_des_t = t_traj;
        srb_params.vx_des_vx = v_traj;
        srb_params.t_beg_stepping = t_beg_stepping_time;
        srb_params.t_end_stepping = t_end_stepping_time;
        printf("T_beg_stepping: %f, T_end_Stepping: %f \n", t_beg_stepping_time, t_end_stepping_time);
        sim_time = srb_params.vx_des_t(srb_params.vx_des_t.size()-1);
        // sim_time = 1e5;
    }
    else
    {
        printf("Telelop Selected\n\n");
        recording_file_name = "Telelop";
        srb_params.planner_type = 2; 
        srb_params.init_type = 1;
        sim_time = 1e5;
    }
    

	// Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

    controller->set_SRB_params(srb_params);
    controller->set_traj_planner_dyn_data(traj_planner_dyn_data);    


}

void initializeLegs()
{
    d->qpos[hip_yaw_l_idx] = q0.row(1)(0);
    d->qpos[hip_roll_l_idx] = q0.row(1)(1);
    d->qpos[hip_pitch_l_idx] = q0.row(1)(2);
    d->qpos[knee_pitch_l_idx] = q0.row(1)(3);
    d->qpos[ankle_pitch_l_idx] = q0.row(1)(4);
    d->qpos[hip_yaw_r_idx] = q0.row(0)(0);
    d->qpos[hip_roll_r_idx] = q0.row(0)(1);
    d->qpos[hip_pitch_r_idx] = q0.row(0)(2);
    d->qpos[knee_pitch_r_idx] = q0.row(0)(3);
    d->qpos[ankle_pitch_r_idx] = q0.row(0)(4);

    // Set joint angles for the left and right arms
    // Assuming the indices of the joints for the arms are known, replace with actual indices
    int left_shoulder_roll_joint_id = mj_name2id(m, mjOBJ_JOINT, "left_shoulder_roll");
    int right_shoulder_roll_joint_id = mj_name2id(m, mjOBJ_JOINT, "right_shoulder_roll");
    int left_shoulder_pitch_joint_id = mj_name2id(m, mjOBJ_JOINT, "left_shoulder_pitch");
    int right_shoulder_pitch_joint_id = mj_name2id(m, mjOBJ_JOINT, "right_shoulder_pitch");
    int left_shoulder_yaw_joint_id = mj_name2id(m, mjOBJ_JOINT, "left_shoulder_yaw");
    int right_shoulder_yaw_joint_id = mj_name2id(m, mjOBJ_JOINT, "right_shoulder_yaw");
    int left_elbow_joint_id = mj_name2id(m, mjOBJ_JOINT, "left_elbow");
    int right_elbow_joint_id = mj_name2id(m, mjOBJ_JOINT, "right_elbow");

    // Set shoulder pitch angles to zero
    d->qpos[left_shoulder_roll_joint_id] = 0;//0.15;
    d->qpos[right_shoulder_roll_joint_id] = 0;//-0.15;
    d->qpos[left_shoulder_pitch_joint_id] = 0;//0.2;
    d->qpos[right_shoulder_pitch_joint_id] = 0;//0.2;
    d->qpos[left_shoulder_yaw_joint_id] = 0;//-0.3;
    d->qpos[right_shoulder_yaw_joint_id] = 0;//0.3;

    // Set elbow angles to 90 degrees (converted to radians)
    d->qpos[left_elbow_joint_id] = 0;
    d->qpos[right_elbow_joint_id] = 0;
}

ImVec4 hex2ImVec4(int hex)
{
    return ImVec4(((hex & 0xFF0000) >> 16)/ 255.0f, ((hex & 0xFF00) >> 8)/ 255.0f, (hex & 0xFF)/255.0f, 1.0f);
}
double last_Xf = 0;
double last_Yf = 0;
double last_springf = 0;
bool controller_unstable = false;
bool realtime_enabled = true;
bool simulation_ready_to_run = false;

VectorXd y_forces(100);
VectorXd x_forces(100);
VectorXd s_forces(100);
double y_force, x_force, s_force;
int force_idx = 0;

double sin_cnt = 0;
double sin_val = 0;
int video_pulse_indicator_cnt = 0;


bool sim_ready_for_control = false;
void* tello_controller( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
	// Print the core and priority of the thread
	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Controller thread running on core %d, with priority %d\n", core, priority);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(!sim_ready_for_control || !cd_shared_data_ready) usleep(100);

    while(1)
    {
        handle_start_of_periodic_task(next);
        
        pthread_mutex_lock(&sim_step_mutex);
        ctrlData cd_local;
        cd_local = cd_shared;
        pthread_mutex_unlock(&sim_step_mutex);
        pthread_mutex_lock(&tello_ctrl_mutex);
        if(simulation_mode == 1)
        {
            if(!pause_sim){
                    TELLO_locomotion_ctrl(cd_local);
            } 
        }
        if(simulation_mode == 2)
        {
            if(!pause_sim){
                    TELLO_locomotion_ctrl(cd_local);
                    set_mujoco_state(tello->controller->get_x());
            } 
        }
        // if(simulation_mode == 3)
        // {
        //     if(!pause_sim){
        //          set_mujoco_state(tello->controller->get_x());
        //     } 
        // }
        pthread_mutex_unlock(&tello_ctrl_mutex);
        
        handle_end_of_periodic_task(next, period);
    }

}



void* hmi_hw_monitor( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
	// Print the core and priority of the thread
	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Controller thread running on core %d, with priority %d\n", core, priority);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while(1)
    {
        handle_start_of_periodic_task(next);
        
       hmi_comms_counter++;
       if(hmi_comms_counter > 1000)
       {
            hmi_connected = false;
       }
       else{
            hmi_connected = true;
       }
        
        handle_end_of_periodic_task(next, period);
    }

}

extern VectorXd th_R_Arm;
extern VectorXd th_L_Arm;

void runArmControl()
{

    float RA_pose_des = 0.0;
    float LA_pose_des = 0.0;
    float human_ef_log[3];

    for(int i = 0; i < 2; i++){
        float th_j1 = 0;
        float th_j2 = 0;
        float th_j3 = 0;
        float th_j4 = 0;
        if(i==0){
        //Joystick right joint angles (input)
            th_j1 =(R_joystick_enc[0]/180)*3.14159;
            th_j2 =(R_joystick_enc[1]/180)*3.14159;
            th_j3 =(R_joystick_enc[2]/180)*3.14159;
            th_j4 =(R_joystick_enc[3]/180)*3.14159;
        }
        else{
            th_j1 = -(L_joystick_enc[0]/180)*3.14159;
            th_j2 = -(L_joystick_enc[1]/180)*3.14159;
            th_j3 = -(L_joystick_enc[2]/180)*3.14159;
            th_j4 = -(L_joystick_enc[3]/180)*3.14159;
        }
        //Physical lengths
        float L_joy_shoulder_from_body = 0.2105; //middle to shoulder pivot (to the right)
        float L_joy_shoulder_x = 0.1035; //to the right
        float L_joy_shoulder_y = 0.168; //downwards
        float L_joy_arm = 0.313; //upper arm length
        float L_joy_forearm = 0.339; //lower arm length
        float L_joy_hand = 0.078; //width of the hand that goes inwards
        float L_sat_shoulder_from_body = 0.2297;
        float L_sat_forearm = 0.2277;
        float L_sat_arm = 0.1790;
        //HTM of end effector in base frame
        float joy_eff[12];
        float s1 = sin(th_j1);
        float c1 = cos(th_j1);
        float s2 = sin(th_j2);
        float c2 = cos(th_j2);
        float c3 = cos(th_j3);
        float s34 = sin(th_j3 - th_j4);
        float c34 = cos(th_j3 - th_j4);
        joy_eff[0] = -s1*s2*s34 + c1*c34;
        joy_eff[1] = -s1*c2;
        joy_eff[2] = s1*s2*c34 + s34*c1;
        joy_eff[3] = -L_joy_arm*(s1*s2*c3 + sin(th_j3)*c1) - L_joy_forearm*(s1*s2*c34 + s34*c1) - L_joy_hand*s1*c2 + L_joy_shoulder_x*c1 + L_joy_shoulder_y*s1*c2;
        joy_eff[4] = s1*c34 + s2*s34*c1;
        joy_eff[5] = c1*c2;
        joy_eff[6] = s1*s34 - s2*c1*c34;
        joy_eff[7] = -L_joy_arm*(s1*sin(th_j3) - s2*c1*c3) - L_joy_forearm*(s1*s34 - s2*c1*c34) + L_joy_hand*c1*c2 - L_joy_shoulder_from_body + L_joy_shoulder_x*s1 - L_joy_shoulder_y*c1*c2;
        joy_eff[8] = -s34*c2;
        joy_eff[9] = s2;
        joy_eff[10] = c2*c34;
        joy_eff[11] = -L_joy_arm*c2*c3 - L_joy_forearm*c2*c34 + L_joy_hand*s2 - L_joy_shoulder_y*s2;
        float human_ef[3];
        human_ef[0] = joy_eff[3];
        human_ef[1] = joy_eff[7];
        human_ef[2] = joy_eff[11];
        if(i == 0){
            human_ef_log[0] = human_ef[0];
            human_ef_log[1] = human_ef[1];
            human_ef_log[2] = human_ef[2];
        }
        //Direction of satyrr's elbow
        float sat_elb_dir[3];
        sat_elb_dir[0] = L_joy_forearm*joy_eff[2] - L_joy_shoulder_x + joy_eff[3];
        sat_elb_dir[1] = L_joy_forearm*joy_eff[6] - L_joy_hand + L_joy_shoulder_from_body + L_joy_shoulder_y + joy_eff[7];
        sat_elb_dir[2] = L_joy_forearm*joy_eff[10] + joy_eff[11];
        //Rz is the unit vector from satyrr's elbow to shoulder
        float Rz[3];
        Rz[0] = -sat_elb_dir[0];
        Rz[1] = -sat_elb_dir[1];
        Rz[2] = -sat_elb_dir[2];
        float Rz_mag = sqrt(pow(Rz[0], 2) + pow(Rz[1], 2) + pow(Rz[2], 2));
        Rz[0] = Rz[0]/Rz_mag;
        Rz[1] = Rz[1]/Rz_mag;
        Rz[2] = Rz[2]/Rz_mag;
        //Ry is the component of joystick's elbow axis perpendicular to Rz (as a unit vector)
        //Calculated with: norm3d(yaxis_joy_end - Rz*yaxis_joy_end.dot(Rz))
        float Ry[3];
        float ydotRz = Rz[0]*joy_eff[1] + Rz[1]*joy_eff[5] + Rz[2]*joy_eff[9];
        Ry[0] = -Rz[0]*ydotRz + joy_eff[1];
        Ry[1] = -Rz[1]*ydotRz + joy_eff[5];
        Ry[2] = -Rz[2]*ydotRz + joy_eff[9];
        float Ry_mag = sqrt(pow(Ry[0], 2) + pow(Ry[1], 2) + pow(Ry[2], 2));
        Ry[0] = Ry[0]/Ry_mag;
        Ry[1] = Ry[1]/Ry_mag;
        Ry[2] = Ry[2]/Ry_mag;
        //Rx is the remaining axis to define satyrr's elbow frame, Rx = Ry x Rz
        float Rx[3];
        Rx[0] = Ry[1]*Rz[2] - Ry[2]*Rz[1];
        Rx[1] = -Ry[0]*Rz[2] + Ry[2]*Rz[0];
        Rx[2] = Ry[0]*Rz[1] - Ry[1]*Rz[0];
        //Inverse kinematics of a spherical wrist
        float th_s1 = M_PI - (fmod(atan2(Rz[2], Rz[0]) + M_PI_2, 2.0*M_PI));
    float th_s2 = M_PI - (fmod(atan2(Rz[1], sqrt(1 - pow(Rz[1], 2.0))) + M_PI, 2.0*M_PI));
    float th_s3 = M_PI - (fmod(atan2(-Rx[1], Ry[1]) + M_PI, 2.0*M_PI));
        //angle between satyrr's upper arm and joystick forearm about Ry
        float th_s4 = -acos(Rz[0]*joy_eff[2] + Rz[1]*joy_eff[6] + Rz[2]*joy_eff[10]);
        if(-Ry[0]*(Rz[1]*joy_eff[10] - Rz[2]*joy_eff[6]) + Ry[1]*(Rz[0]*joy_eff[10] - Rz[2]*joy_eff[2]) - Ry[2]*(Rz[0]*joy_eff[6] - Rz[1]*joy_eff[2]) < 0){
            th_s4 = -1*th_s4;
        }
        //Flipping angle convention to match model -> robot
        th_s1 = -th_s1;
        th_s2 =  th_s2;
        th_s3 =  th_s3;
        th_s4 = -th_s4;
        //Flipping angle convention to matc right or left arm
        float arm_temp[4];
        if(i==0){
            arm_temp[0] = th_s1;
            arm_temp[1] = th_s2;
            arm_temp[2] = th_s3;
            arm_temp[3] = th_s4;
            for(int j = 0; j < 4; j++){
                //Safety condition bounding change in RARM joint angle
                if(abs(th_R_Arm[j] - arm_temp[j]) < 1.57){
                    th_R_Arm[j] = arm_temp[j];
                }
                else{
                    th_R_Arm[j] = th_R_Arm[j];
                }
            }
            RA_pose_des = (207*cos(th_s1))/50000.0 - (10396091193541362039.0*cos(th_s1)*cos(th_s4))/225179981368524800000.0 - (2389.0*cos(th_s2)*sin(th_s1))/20000.0 - (3808975302672035301.0*cos(th_s1)*sin(th_s4))/18014398509481984000.0 + (10396091193541362039.0*cos(th_s2)*sin(th_s1)*sin(th_s4))/225179981368524800000.0 - (3808975302672035301.0*cos(th_s2)*cos(th_s4)*sin(th_s1))/18014398509481984000.0;
            RA_pose_des = -RA_pose_des; // To get angle conventions to match of robot
        }
        else{
            arm_temp[0] = -th_s1;
            arm_temp[1] = -th_s2;
            arm_temp[2] = -th_s3;
            arm_temp[3] = -th_s4;
            for(int j = 0; j < 4; j++){
                //Safety condition bounding change in LARM joint angle
                if(abs(th_L_Arm[j] - arm_temp[j]) < 1.57){
                    th_L_Arm[j] = arm_temp[j];
                }
                else{
                    th_L_Arm[j] = th_L_Arm[j];
                }
            }
            LA_pose_des = (207.0*cos(th_L_Arm[0]))/50000.0 - (2559.0*cos(th_L_Arm[3])*(cos(th_L_Arm[0])*cos(th_L_Arm[2]) + sin(th_L_Arm[0])*sin(th_L_Arm[1])*sin(th_L_Arm[2])))/50000.0 - L_sat_forearm*(sin(th_L_Arm[3])*(cos(th_L_Arm[0])*cos(th_L_Arm[2]) + sin(th_L_Arm[0])*sin(th_L_Arm[1])*sin(th_L_Arm[2])) + cos(th_L_Arm[1])*cos(th_L_Arm[3])*sin(th_L_Arm[0])) + (2559.0*cos(th_L_Arm[1])*sin(th_L_Arm[0])*sin(th_L_Arm[3]))/50000.0 - L_sat_arm*cos(th_L_Arm[1])*sin(th_L_Arm[0]);
        }
    }

    RoboDesignLab::JointPDConfig arm_pd;

    arm_pd.joint_pos_desired = VectorXd::Zero(8);
    arm_pd.joint_vel_desired = VectorXd::Zero(8);
    arm_pd.joint_pos_desired << th_L_Arm(0), th_L_Arm(1), th_L_Arm(2), th_L_Arm(3), -th_R_Arm(0), th_R_Arm(1), th_R_Arm(2), -th_R_Arm(3);
    arm_pd.joint_vel_desired << 0, 0, 0, 0, 0, 0, 0, 0;

    // cout << arm_pd.joint_pos_desired.transpose() << endl;
    
    VectorXd arm_kp(8);
    VectorXd arm_kd(8);
    arm_kp << 100,100,100,100,    100,100,100,100;
    arm_kd << 0.5,0.5,0.5,0.5,    0.5,0.5,0.5,0.5;
    arm_pd.joint_kp = arm_kp.asDiagonal();
    arm_pd.joint_kd = arm_kd.asDiagonal();
    arm_pd.motor_kp = VectorXd::Zero(8);
    arm_pd.motor_kd = VectorXd::Zero(8);
    arm_pd.joint_ff_torque  = VectorXd::Zero(8);
    VectorXd arm_torques = tello->arm_jointPD(arm_pd);

    // cout << arm_torques.transpose() << endl;

    applyArmJointTorquesMujoco(m,d,arm_torques);
}


char notes[100]; // String variable to store the entered notes
void* mujoco_Update_1KHz( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
	// Print the core and priority of the thread
	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Mujoco thread running on core %d, with priority %d\n", core, priority);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    log_folder = createLogFolder("/home/tello/Desktop/tello_outputs/Logs/");
    dash_utils::setOutputFolder(log_folder);

    sim_conf = readSimConfigFromFile("/home/tello/Documents/GIT/Tello_Software/include/sim_config.json");

    std::vector<userProfile> profiles;
    std::string active_user;
    readProfilesFromJson("/home/tello/Documents/GIT/Tello_Software/include/user_profiles.json",profiles, active_user);
    for(int i=0;i<profiles.size();i++)
    {
        if(active_user.compare(profiles[i].name) == 0)
        {
            activeUser = profiles[i];
            Human_params human_params = tello->controller->get_human_params();
            human_params.hLIP = activeUser.lip_height;
            human_params.m = activeUser.weight;
        }
    }


    // INITIALIZE SRBM CONTROLLER ========================================================

    initializeSRBMCtrl();

    dash_utils::writeSRBParamsToTxt(tello->controller->get_SRB_params(),"srb_params.csv");
    dash_utils::writeHumanParamsToTxt(tello->controller->get_human_params(),"human_params.csv");

    // BEGIN SETUP CODE FOR MUJOCO ======================================================================
    
    mj_activate("./lib/Mujoco/mjkey.txt");
    if(simulation_mode == 1)
    {
        // m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-6-16-23.xml", NULL, error, 1000);
        // m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-with-arms.xml", NULL, error, 1000);
        m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-with-arms-fake.xml", NULL, error, 1000);
        // m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
    }
    else if(simulation_mode == 3)
    {
        m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-visualization.xml", NULL, error, 1000);
        // m_shared = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
    }
    else if(simulation_mode == 4)
    {
        m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-hardware-visualization.xml", NULL, error, 1000);
        // m_shared = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
    }
    else
    {
        m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-blue-grey.xml", NULL, error, 1000);
        // m_shared = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-blue-grey.xml", NULL, error, 1000);
    }

	if (!m)
    {
        printf('r',"%s\n", error);
        exit(1);
    }
	// make data
    d = mj_makeData(m);
    // d_shared = mj_makeData(m_shared);

    initializeLegs();

    // install control callback
    // mjcb_control = dummy_callback;//TELLO_locomotion_ctrl;

	// init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

		// create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1920, 1080, "Tello Mujoco Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    // cam.elevation = -18;
    // cam.distance = 1.8;
    // cam.azimuth = 135;
    // cam.lookat[0] = 0;
    // cam.lookat[1] = 0;
    // cam.lookat[2] = -0.2;

    cam.elevation = -8;
    cam.distance = 2.0;
    cam.azimuth = -135;
    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = -0.1;

    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetWindowCloseCallback(window,window_close_callback);

    m->opt.timestep = 0.002;

    // set up UDP transmit here: =======================================================================
	// Creating socket file descriptor
	if ( (sockfd_tx = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
	memset(&servaddr_tx, 0, sizeof(servaddr_tx));
	// Filling server information
	servaddr_tx.sin_family = AF_INET;
	servaddr_tx.sin_port = htons(UDP_TRANSMIT_PORT);
	servaddr_tx.sin_addr.s_addr = inet_addr(HMI_IP_ADDRESS);
	// ens UDP setup here: =============================================================================

    // Plotting:

	// END SETUP CODE FOR MUJOCO ========================================================================
    pthread_mutex_lock(&sim_mutex);
    mj_forward(m, d);
    pthread_mutex_unlock(&sim_mutex);
    simulation_ready_to_run = true;
    // initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::StyleColorsDark();


    float baseFontSize = 60.0f; // 13.0f is the size of the default font. Change to the font size you use.
    float iconFontSize = baseFontSize * 2.25f / 3.0f; // FontAwesome fonts need to have their sizes reduced by 2.0f/3.0f in order to align correctly


    ImFont* font;
    ImFont* fontSmall;
    if (std::filesystem::is_directory("/home/telloHW")) {
        //std::cout << "The directory /home/tello exists!" << std::endl;
        font = io.Fonts->AddFontFromFileTTF("./tello_files/fonts/roboto/Roboto-Light.ttf", baseFontSize);
    }
    else {
        //std::cout << "The directory /home/tello does not exist!" << std::endl;
        fontSmall = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/roboto/Roboto-Light.ttf", baseFontSize/1.75);
        font = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/roboto/Roboto-Light.ttf", baseFontSize);
        // font = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/seguisym.ttf", 80);
        
    }
    
    // merge in icons from Font Awesome
    static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_16_FA, 0 };
    ImFontConfig icons_config; 
    icons_config.MergeMode = true; 
    icons_config.PixelSnapH = true; 
    icons_config.GlyphMinAdvanceX = iconFontSize;
    io.Fonts->AddFontFromFileTTF( "../../../lib/imGUI/fonts/fa-solid-900.ttf", iconFontSize, &icons_config, icons_ranges );


    // create a vector of all the plot pngs for selecting playback option
    std::string plotfolderPath = "/home/tello/Desktop/tello_outputs/Favorite_Logs/Plots";
    std::vector<std::string> pngFiles;
    std::vector<GLuint> image_textures;
    std::vector<std::string> image_names;
    GLuint active_texture;
    int im_width = 888;
    int im_height = 500;

    for (const auto& entry : std::filesystem::directory_iterator(plotfolderPath)) {
        if (entry.path().extension() == ".png") {
            std::string name = entry.path().filename().string();
            pngFiles.push_back((plotfolderPath+"/"+name));
            GLuint tex = 0;
            LoadTextureFromFile((plotfolderPath+"/"+name).c_str(), &tex, &im_width, &im_height);
            image_textures.push_back(tex);
            image_names.push_back(name.substr(0, name.length() - 4));
            hddFiles.push_back("/home/tello/Desktop/tello_outputs/Favorite_Logs/"+name.substr(0, name.length() - 4)+"/human_dyn_data.csv");
        }
    }

    // // Print the filenames
    // for (const auto& filename : hddFiles) {
    //     std::cout << filename << std::endl;
    // }

    // dash_utils::start_sim_timer();

	while(!glfwWindowShouldClose(window))
    {
        handle_start_of_periodic_task(next);
		// BEGIN LOOP CODE FOR MUJOCO ===================================================================
        
        // set local tello object here:
        pthread_mutex_lock(&tello_ctrl_mutex);
        RoboDesignLab::DynamicRobot* telloLocal = new RoboDesignLab::DynamicRobot(*tello);
        pthread_mutex_unlock(&tello_ctrl_mutex);

        // Get body ID
        pthread_mutex_lock(&sim_step_mutex);
        int body_id = mj_name2id(m, mjOBJ_BODY, "torso");

        // Apply force-torque to body
        // mjtNum force[3] = {push_force_x, push_force_y, -push_force_z}; // x, y, z components of the force
        mjtNum torque[3] = {0.0, 0.0, 0.0}; // x, y, z components of the torque
        mjtNum point[3] = {0.0, 0.0, 0.0}; 
        pthread_mutex_lock(&sim_mutex);
        if(impulse_scheduled)
        {
            mj_applyFT(m, d, push_force, torque, point, body_id, d->qfrc_applied);
            impulse_scheduled = false;
            impulse_active = true;
            // cout << "Impulse Applied" << endl;
            // dash_utils::start_timer();
        }
        if(d->time - impulse_start_time > 0.15 && impulse_active)
        {
            mju_zero(d->qfrc_applied, m->nv);
            push_force[0] = 0;
            push_force[1] = 0;
            push_force[2] = 0;
            impulse_active = false;
            // cout << "Impulse ended. ";
            // dash_utils::print_timer();
        }

        pthread_mutex_unlock(&sim_mutex);

        // ctrlData cd_local;
        // cd_local = cd_shared;
        pthread_mutex_unlock(&sim_step_mutex);


        std::string text = "\tTello Mujoco Simulation    |    Test: " + recording_file_name + "    |    Time: " + std::to_string(d->time)+ "\t";
        glfwSetWindowTitle(window, text.c_str());

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        if(!pause_sim)
        {
            torso_vels.tail(1999) = torso_vels.head(1999).eval();
            torso_vels[0] = telloLocal->controller->get_x()(3);
        }
        double torso_vel_smooth = torso_vels.mean();

        cam.lookat[0] = d->qpos[torso_x_idx];//-torso_vel_smooth/2.0;
        cam.lookat[1] = d->qpos[torso_y_idx];
        
        // set the background color to white
        
        // update scene and render
        if(render_counter%33 == 0)
        {
            pthread_mutex_lock(&sim_mutex);
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            pthread_mutex_unlock(&sim_mutex);
            mjr_render(viewport, &scn, &con);
        }
        
        sim_ready_for_control = true;
        // render ImGui GUI
        ImVec4 dark_navy = hex2ImVec4(0x082032);
        ImVec4 med_navy = hex2ImVec4(0x2C394B);
        ImVec4 light_navy = hex2ImVec4(0x334756);
        ImVec4 lighter_navy = hex2ImVec4(0x54748c);
        
        ImVec4 grey1 = hex2ImVec4(0xd9d9d9);
        ImVec4 grey2 = hex2ImVec4(0xadadad);
        ImVec4 grey3 = hex2ImVec4(0x7d7d7d);
        ImVec4 grey4 = hex2ImVec4(0x424242);
        ImVec4 grey5 = hex2ImVec4(0x1f1f1f);

        ImVec4 black = hex2ImVec4(0x000000);
        ImVec4 white = hex2ImVec4(0xFFFFFF);
        ImVec4 blue1 = hex2ImVec4(0x0d75bf);
        ImVec4 red = hex2ImVec4(0xb51021);
        ImVec4 redHover = hex2ImVec4(0xd92133);
        ImVec4 redActive = hex2ImVec4(0x7a0b16);
        // ImVec4 grey2 = hex2ImVec4(0x7d7d7d);
        // ImVec4 grey2 = hex2ImVec4(0x7d7d7d);
        glfwGetWindowSize(window, &windowWidth, &windowHeight);
        double screenScale = ((double)windowWidth/3840.0)*1.2;
        if(screenScale > 1.0) screenScale = 1.0;
        io.FontGlobalScale = screenScale;
        separator_thickness = 15.0*screenScale;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::PushFont(font);
        ImGui::PushStyleColor(ImGuiCol_MenuBarBg, grey5);
        ImGui::SetNextWindowSizeConstraints(ImVec2(-1, 35+60.0*screenScale), ImVec2(-1, FLT_MAX));
        ImGui::BeginMainMenuBar();
        ImGui::EndMainMenuBar();
        ImGui::PopStyleColor();
        ImGui::SetNextWindowPos(ImVec2(0, 15));
        ImGui::SetNextWindowSizeConstraints(ImVec2(-1, 10*60.0*screenScale), ImVec2(-1, FLT_MAX));
        // add toolbar with button
        ImGui::PushStyleColor(ImGuiCol_MenuBarBg, grey4);
        ImGui::PushStyleColor(ImGuiCol_Button, med_navy);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, light_navy);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, lighter_navy);
        
        ImGui::BeginMainMenuBar();

        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();

        bool dummy;
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);        
        if (ImGui::BeginMenu(" " ICON_FA_BARS " "))
        {
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_FILE_CSV "  Enable Data Logging   ", &(sim_conf.en_data_logging));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_PLAY "  Enable Playback Mode   ", &(sim_conf.en_playback_mode));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_DICE_TWO "  Enable V2 Controller   ", &(sim_conf.en_v2_controller));
            en_v2_ctrl = sim_conf.en_v2_controller;
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_USER_ALT_SLASH "  Boot to Auto Mode (Restart Required)   ", &(sim_conf.en_autonomous_mode_on_boot));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            // std::string human_label;
            // if(sim_conf.en_playback_mode) human_label = " " ICON_FA_CHILD "  Enable HMI Playback   ";
            // else human_label = " " ICON_FA_CHILD "  Enable HMI Communication   ";
            // ImGui::Checkbox(human_label.c_str(), &(sim_conf.en_human_control));
            // tello->controller->enable_human_dyn_data = sim_conf.en_human_control;
            // ImGui::Separator();  
            ImGui::Checkbox(" " ICON_FA_SIGN_IN_ALT "  Enable X Haptic Force   ", &(sim_conf.en_x_haptic_force));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_ARROWS_ALT_H "  Enable Force Feedback   ", &sim_conf.en_force_feedback);      // Edit bools storing our window open/close state
            ImGui::Separator();//init_foot_width
            ImGui::Checkbox(" " ICON_FA_HARD_HAT "  Enable Safety Monitor   ", &sim_conf.en_safety_monitor);      // Edit bools storing our window open/close state
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_GAMEPAD "  Enable PS4 Controller   ", &sim_conf.en_ps4_controller);      // Edit bools storing our window open/close state
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_CAMERA "  Enable HMI Recording   ", &(sim_conf.en_HMI_recording));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_TV "  Enable Mujoco Recording   ", &(sim_conf.en_screen_recording));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_VIDEO "  Auto-Record Sessions   ", &(sim_conf.en_auto_record));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_LAPTOP_CODE "  Show Live Variable View  ", &(sim_conf.en_live_variable_view));
            ImGui::Separator();
            // ImGui::Checkbox(" " ICON_FA_SLIDERS_H "  Show Full HMI Controls   ", &(sim_conf.en_full_hmi_controls));
            // ImGui::Separator();
            // ImGui::Checkbox(" " ICON_FA_CHART_LINE "  Display Realtime Plot   ", &(sim_conf.en_realtime_plot));
            // ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            if (ImGui::MenuItem(" " ICON_FA_SAVE "  Save Configuration"))
            { 
                writeSimConfigToFile(sim_conf, "/home/tello/Documents/GIT/Tello_Software/include/sim_config.json");
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();
        char notes[1001]; // String variable to store the entered notes
        
        if (ImGui::BeginMenu((" " ICON_FA_USER " " + activeUser.name + " ").c_str()))
        {
            ImGui::Separator();
            ImGui::Text(" Select User Profile:   ");
            ImGui::Separator();
            for(int p=0;p<profiles.size();p++)
            {
                userProfile user = profiles[p];
                ImGui::SetNextItemWidth(300);
                if(ImGui::Button((" " + user.name + " ").c_str(),ImVec2(500, 0)))
                {
                    activeUser = profiles[p];
                    updateActiveUserInJson("/home/tello/Documents/GIT/Tello_Software/include/user_profiles.json",activeUser.name);
                    initializeSRBMCtrl();
                }
                ImGui::Separator();
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if(bookmarked)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.84f, 0.0f, 1.0f)); // Set text color to gold
        }
        if (ImGui::BeginMenu(" " ICON_FA_STAR " "))
        {
            if(bookmarked) ImGui::PopStyleColor();
            ImGui::Separator();
            if(!pause_sim || screen_recording || usbcam_recording || usbcam_hw_recording)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // Set button color to grey
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // Set button color to grey
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // Set button color to grey
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f); // Dim the button
                ImGui::Button(" " ICON_FA_COPY " Copy log folder to favorites ");
                ImGui::PopStyleVar();
                ImGui::PopStyleColor(3);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Please stop simulation and recording to copy"); // Display tooltip on hover
                }
            }
            else
            {
                if (ImGui::Button(" " ICON_FA_COPY " Copy log folder to favorites ")) // Submit button
                {
                    if(pause_sim && !screen_recording && !usbcam_recording && ! usbcam_hw_recording)
                    {
                        std::string command = "cp -R " + log_folder + " /home/tello/Desktop/tello_outputs/Favorite_Logs/";
                        system(command.c_str());
                    }
                    else
                    {
                        showCopyErrorPopup = true;
                    }
                }
            }
            ImGui::Separator();
            if(bookmarked)
            {
                if (ImGui::Button(" " ICON_FA_TIMES " Undo Bookmark                    ")) // Submit button
                {
                    bookmarked = false;
                    dash_utils::deleteLogFile("bookmarked.txt");
                }
            }
            else
            {
                if (ImGui::Button(" " ICON_FA_BOOKMARK " Bookmark log folder             ")) // Submit button
                {
                    bookmarked = true;
                    dash_utils::writeStringToFile("","bookmarked.txt");
                }
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        
        else if(bookmarked) ImGui::PopStyleColor();

        ImGui::Separator();
        if (ImGui::BeginMenu(" " ICON_FA_PENCIL_ALT " "))
        {
            ImGui::Separator();
            ImGui::Separator();
            ImGui::AlignTextToFramePadding(); // Align label to top
            ImGui::Text("  Notes  "); // Label text
            ImGui::SameLine(); // Place the text entry box next to the label
            ImGui::InputText("  ", notes,1000); // Text entry box for notes                                                                   ");
            ImGui::SameLine();
            if (ImGui::Button(" Save ")) // Submit button
            {
                std::string submittedNotes = notes;
                dash_utils::writeStringToFile(submittedNotes+"\n\n","notes.txt");
                notes[0] = '\0';
            }
            ImGui::SameLine();
            ImGui::Text("  ");
            ImGui::Separator();
            ImGui::Separator();
            ImGui::EndMenu();
        }

        ImGui::PopStyleColor();
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();

        if(pause_sim)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.4f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 0.6f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
            if (ImGui::Button(" " ICON_FA_PLAY_CIRCLE " ")) {
                pause_sim = false;
            }
            ImGui::PopStyleColor(3);
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.0f, 0.45f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.65f, 0.0f, 0.65f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.25f, 0.0f, 0.25f, 1.0f));
            if (ImGui::Button(" " ICON_FA_PAUSE_CIRCLE " ") || controller_unstable) {
                pause_sim = true;
                master_gain = 0.0;
                tello->controller->disable_human_ctrl();
                // screen_recording = false;
                // usbcam_recording = false;
                // system("killall -2 ffmpeg");
                
                recording_in_progress = false;
                usb_recording_in_progress = false;
                usb_recording_HW_in_progress = false;
            }
            ImGui::PopStyleColor(3);
        }
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();
        if(!screen_recording)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, red);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redActive);
            if (ImGui::Button(" " ICON_FA_VIDEO " ")) {
                screen_recording = true;
                usbcam_recording = true;
                usbcam_hw_recording = true;
            }
            ImGui::PopStyleColor(3);
        }
        else
        {
            double color_offset = 20 * std::sin(video_pulse_indicator_cnt / 1000.0);  // Adjust frequency by dividing counter
            video_pulse_indicator_cnt +=5;
            if(video_pulse_indicator_cnt > 6283) video_pulse_indicator_cnt = 0;
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4((181.0f+color_offset) / 255.0f, 16.0f / 255.0f, 33.0f / 255.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redActive);
            if (ImGui::Button(" " ICON_FA_VIDEO_SLASH " ")) {
                screen_recording = false;
                usbcam_recording = false;
                usbcam_hw_recording = false;
                system("killall -2 ffmpeg");
                
                recording_in_progress = false;
                usb_recording_in_progress = false;
                usb_recording_HW_in_progress = false;
                
                usleep(2000);
            }
            ImGui::PopStyleColor(3);
        }
        
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::PushStyleColor(ImGuiCol_CheckMark,black);
        ImGui::PushStyleColor(ImGuiCol_FrameBg,grey1);
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,white);
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,grey2);

        ImGui::Separator();
        if(!(sim_conf.en_autonomous_mode_on_boot))
        {
            // ImGui::Separator();
            // ImGui::Checkbox(" Real-Time   ", &realtime_enabled);
            if(sim_conf.en_full_hmi_controls || sim_conf.en_playback_mode){

                std::string human_label;
                if(sim_conf.en_playback_mode) human_label = "  Enable HMI Playback   ";
                else
                {
                    if(hmi_connected)
                        human_label = "  Enable HMI Communication   ";
                    else{
                        human_label = "  HMI Not Connected          ";
                    }
                }
                ImGui::Checkbox(human_label.c_str(), &sim_conf.en_human_control);
                

                if(!(sim_conf.en_playback_mode))
                {
                    ImGui::Separator();//init_foot_width
                    ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                    if (ImGui::Button(" " ICON_FA_WEIGHT " Tare CoM  ")) {
                        zero_human = true;
                        human_x_zero = tello->controller->get_human_dyn_data().xH;
                    }
                    else{
                        zero_human = false;
                    }
                    ImGui::PopStyleColor(3);
                    ImGui::Separator();//init_foot_width
                    ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                    if (ImGui::Button(" " ICON_FA_WEIGHT " Tare Arms  ")) {
                        zero_arms = true;
                    }
                    else{
                        zero_arms = false;
                    }
                    ImGui::PopStyleColor(3);
                }
                
                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_RULER " Init Foot Width  ")) {
                    init_foot_width = true;
                    MatrixXd new_lfv0(4,3);
                    MatrixXd new_q0(2,5);
                    dash_init::init_foot_width(robot_init_foot_width,telloLocal->controller->get_human_params(),
                                                telloLocal->controller->get_SRB_params(),new_lfv0,q0);
                    Traj_planner_dyn_data tpdd_foot_z = tello->controller->get_traj_planner_dyn_data();
                    tpdd_foot_z.step_z_offset_R = tello->controller->get_human_dyn_data().fzH_R;
                    tpdd_foot_z.step_z_offset_L = tello->controller->get_human_dyn_data().fzH_L;
                    tello->controller->set_traj_planner_human_foot_z_offsets(tpdd_foot_z);
                    tello->controller->set_lfv0(new_lfv0);
                    tello->controller->set_q0(q0);
                    initializeLegs();
                    if(pause_sim)
                        mj_forward(m,d);
                }
                else{
                    init_foot_width = false;
                }
                
                ImGui::PopStyleColor(3);
                ImGui::Separator();
                if(!(sim_conf.en_playback_mode))
                {
                    ImGui::SetNextItemWidth(250.0f*screenScale);
                    ImGui::PushStyleColor(ImGuiCol_FrameBg,grey2);
                    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,grey2);
                    ImGui::PushStyleColor(ImGuiCol_SliderGrab,black);
                    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,black);
                    ImGui::SliderFloat(" HMI Gain", &master_gain, 0.0f, 1.0f);
                    ImGui::Separator();
                    ImGui::PopStyleColor(4);
                }
            }
            else{
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button("  " ICON_FA_RULER " Initialize Human & Robot   ")) {
                    init_foot_width = true;
                    MatrixXd new_lfv0(4,3);
                    MatrixXd new_q0(2,5);
                    dash_init::init_foot_width(robot_init_foot_width,telloLocal->controller->get_human_params(),
                                                telloLocal->controller->get_SRB_params(),new_lfv0,q0);
                    Traj_planner_dyn_data tpdd_foot_z = telloLocal->controller->get_traj_planner_dyn_data();
                    tpdd_foot_z.step_z_offset_R = telloLocal->controller->get_human_dyn_data().fzH_R;
                    tpdd_foot_z.step_z_offset_L = telloLocal->controller->get_human_dyn_data().fzH_L;
                    tello->controller->set_traj_planner_human_foot_z_offsets(tpdd_foot_z);
                    tello->controller->set_lfv0(new_lfv0);
                    tello->controller->set_q0(q0);
                    initializeLegs();
                    master_gain = 1.0;
                    zero_human = true;
                    if(pause_sim)
                        mj_forward(m,d);
                }
                ImGui::PopStyleColor(3);
                ImGui::Separator();//init_foot_width
            }
            if(sim_conf.en_playback_mode)
            {
                if(!showPlotMenu)
                {
                    if (ImGui::Button(" " ICON_FA_PLAY " Show Playback Menu  ")) 
                    {
                        showPlotMenu = true;
                    }
                }
                else
                {
                    if (ImGui::Button(" " ICON_FA_PLAY " Hide Playback Menu  ")) 
                    {
                        showPlotMenu = false;
                    }
                }
            }
        }
        else{
            // ImGui::Separator();
            // ImGui::Checkbox(" Real-Time   ", &realtime_enabled);
            ImGui::Separator();
            ImGui::Text("Autonomous Mode");
        }
        
        ImGui::PopStyleColor(5);
        ImGui::EndMainMenuBar();
        // ImGui::PopFont();
        ImGui::PopStyleColor(4);

        // Load images
        // ImPlot::StyleColorsLight();
        if(showPlotMenu)
        {
            
            double plot_width = (double)windowWidth/5.0;
            double plot_height = plot_width*(9.0/16.0);
            double headerHeight = plot_height+120*screenScale+150*screenScale;
            ImGui::SetNextWindowSize(ImVec2((double)windowWidth/5.0,headerHeight));
            ImGui::SetNextWindowPos(ImVec2( (windowWidth - (double)windowWidth/5.0), (35+60*screenScale)) );
            ImGui::Begin("plotHeader",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

            if(ImGui::Button(" " ICON_FA_TIMES_CIRCLE " "))
            {
                showPlotMenu = false;
            }
            ImGui::SameLine();
            ImGui::Text("  Selected Log:");
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::PushFont(fontSmall);
            ImGui::Text((image_names[active_playback_log_index]).c_str());
            ImGui::PopFont();
            ImGui::Image((void*)(intptr_t)(image_textures[active_playback_log_index]), ImVec2(plot_width, plot_height));
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("  Available Logs:");
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::End();

            ImGui::SetNextWindowSize(ImVec2((double)windowWidth/5.0,windowHeight-headerHeight-(35+60*screenScale)));
            ImGui::SetNextWindowPos(ImVec2( (windowWidth - (double)windowWidth/5.0), (35+60*screenScale+headerHeight)) );
            ImGui::Begin("plotSelect",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);
            ImGui::PushFont(fontSmall);
            for(int i=0;i<pngFiles.size();i++)
            {
                ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
                ImGui::Separator();
                ImGui::PopStyleColor();
                ImGui::Text((image_names[i]).c_str());
                if (ImGui::ImageButton((void*)(intptr_t)(image_textures[i]), ImVec2(plot_width, plot_height)))
                {
                    active_playback_log_png = pngFiles[i];
                    active_playback_log_index = i;
                    playback_changed = true;
                    playback_chosen = true;
                    writeActivePlaybackLog(hddFiles[i],"/home/tello/Documents/GIT/Tello_Software/include/active_playback_log.json");
                }
            }
            ImGui::PopFont();
            
            ImGui::End();
        }
        if(showTuningMenu)
        {
            
            double tuning_width = (double)windowWidth*0.9;
            double tuning_height = windowHeight*0.9- (35+60*screenScale);
            ImGui::SetNextWindowSize(ImVec2(tuning_width,tuning_height));
            ImGui::SetNextWindowPos(ImVec2(windowWidth*0.05, (35+60*screenScale)+windowHeight*0.05));
            ImGui::Begin("plotHeader",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

            if(ImGui::Button(" " ICON_FA_TIMES_CIRCLE " "))
            {
                showTuningMenu = false;
            }
            ImGui::SameLine();
            ImGui::Text("  Tuning Menu:");
            
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::PushStyleColor(ImGuiCol_FrameBg,grey4);
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,grey4);
            ImGui::PushStyleColor(ImGuiCol_SliderGrab,black);
            ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,black);
            float f1,f2,f3,f4,f5,f6,f7,f8,f9,f0;
            ImGui::SliderFloat(" Kp1", &f1, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp2", &f2, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp3", &f3, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp4", &f4, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp5", &f5, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp6", &f6, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp7", &f7, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            
            ImGui::PopStyleColor(5);

            // Reset the ImGui style to its original value
            io.FontGlobalScale = screenScale;

            ImGui::End();
        }
        if(sim_conf.en_live_variable_view)
        {
            ImGui::SetNextWindowSize(ImVec2(800*screenScale, 10*60*screenScale + 10*15*screenScale +65*screenScale));
            ImGui::SetNextWindowPos(ImVec2(50, (35+60*screenScale)+50));
            ImGui::Begin("DebugView",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);
            
            ImGui::Text(" Live Variable View:");
            ImGui::Separator();

            // ImGui::Text("FSM: %d", telloLocal->controller->get_FSM());
            // ImGui::Separator();
            // ImGui::Text("Next SSP: %d", telloLocal->controller->get_traj_planner_dyn_data().next_SSP);
            // ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm R 0: %.2f", R_joystick_enc[0]);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm R 1: %.2f", R_joystick_enc[1]);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm R 2: %.2f", R_joystick_enc[2]);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm R 3: %.2f", R_joystick_enc[3]);

                        ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm L 0: %.2f", L_joystick_enc[0]);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm L 1: %.2f", L_joystick_enc[1]);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm L 2: %.2f", L_joystick_enc[2]);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("Arm L 3: %.2f", L_joystick_enc[3]);
            // ImGui::Text("Impulse Force: %dN", (int)impulse_force_newtons);
            // ImGui::Separator();
            // ImGui::Text("LF: %s \tRF %s", grf_lf.c_str(),grf_rf.c_str());
            // ImGui::Text("LB: %s \tRB %s", grf_lb.c_str(),grf_rb.c_str());
            // VectorXd u(12);
            // u << 0,0,tello->_GRFs.right_front,0,0,tello->_GRFs.right_back,0,0,tello->_GRFs.left_front,0,0,tello->_GRFs.left_back;
            // VectorXd CoP = dash_utils::compute_robot_CoP(tello->controller->get_lfv_world(),u);
            // cout << "CoP Size: " << CoP.size() << endl;
            // ImGui::Text("CoP from Mujoco GRF: ");
            // ImGui::Separator();
            // ImGui::PushFont(fontSmall);
            // std::string copviz = "L"+dash_utils::visualizeCoP(tello->controller->get_lfv_world()(2,1),CoP(1),tello->controller->get_lfv_world()(0,1))+"R";
            // ImGui::Text(copviz.c_str());
            // ImGui::Separator();
            // ImGui::PopFont();
            // ImGui::Text("CoP from QP GRF: ");
            // ImGui::PushFont(fontSmall);
            // VectorXd u_qp = tello->controller->get_GRFs();
            // VectorXd CoP_qp = dash_utils::compute_robot_CoP(tello->controller->get_lfv_world(),u_qp);
            // std::string copviz_qp = "L"+dash_utils::visualizeCoP(tello->controller->get_lfv_world()(2,1),CoP_qp(1),tello->controller->get_lfv_world()(0,1))+"R";
            // ImGui::Text(copviz_qp.c_str());
            // ImGui::PopFont();
            ImGui::End();
        }
        
        // Create the popup dialog
        if (playback_error) {
            ImGui::OpenPopup("Playback Error");

            // Display the popup dialog
            if (ImGui::BeginPopupModal("Playback Error", &playback_error, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::Text("Error Opening Playback File. Restart Program.");
                ImGui::Separator();

                if (ImGui::Button("Close"))
                {
                    playback_error = false;
                    ImGui::CloseCurrentPopup();
                }
                    

                ImGui::EndPopup();
            }
        }
        
        ImGui::PopFont();
        if(render_counter%33 == 0)
        {
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // swap OpenGL buffers (blocking call due to v-sync)
            
            glfwSwapBuffers(window);
            
            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }
        else{
            ImGui::EndFrame();
        }
        render_counter++;

        //handle UDP transmit here:
		Human_dyn_data hdd = telloLocal->controller->get_human_dyn_data();
        if(sim_conf.en_safety_monitor)
        {
            if( (fabs(hdd.FxH_hmi - last_Xf) > 100) || (fabs(hdd.FyH_hmi - last_Yf) > 100) || (fabs(hdd.FxH_spring - last_springf) > 100)){
                controller_unstable = true;
            }
        }
        // if( (fabs(hdd.FyH_hmi - last_Yf) > 100) ){
        //     controller_unstable = true;
        // }
        last_Xf = FxH_hmi_out;
        last_Yf = FyH_hmi_out;
        last_springf = FxH_spring_out;

        // hdd.FyH_hmi = 20*sin(sin_cnt);
        // sin_cnt+= 0.016;
        // if(sin_cnt >= 2*M_PI) sin_cnt = 0;
        // sin_val = hdd.FyH_hmi;
        x_forces.tail(99) = x_forces.head(99).eval();
        x_forces[0] = FxH_hmi_out;
        y_forces.tail(99) = y_forces.head(99).eval();
        y_forces[0] = FyH_hmi_out;
        s_forces.tail(99) = s_forces.head(99).eval();
        s_forces[0] = FxH_spring_out;

        //dash_utils::start_timer();
        x_force = dash_utils::smoothData(x_forces,0.8);
        y_force = dash_utils::smoothData(y_forces,0.8);
        s_force = dash_utils::smoothData(s_forces,0.8);
        //dash_utils::print_timer();
        hdd.FyH_hmi = y_force;
        hdd.FxH_hmi = x_force;
        hdd.FxH_spring = s_force;
        
		if(telloLocal->controller->is_human_ctrl_enabled() && !controller_unstable)
		{
            if( !(sim_conf.en_x_haptic_force) )
            {
                hdd.FxH_hmi = 0;
            }
            // hdd.FyH_hmi = 0;
            // hdd.FxH_spring = 0;
		}
		else
		{
			hdd.FxH_hmi = 0;
			hdd.FyH_hmi = 0;
			hdd.FxH_spring = 0;
		}
        // cout << "Spring: " << hdd.FxH_spring << "    , X: " << hdd.FxH_hmi << "    , Y: " << hdd.FyH_hmi << endl;

		dash_utils::pack_data_to_hmi_with_ctrls((uint8_t*)hmi_tx_buffer,hdd,sim_conf.en_force_feedback,zero_human,master_gain,zero_arms);
		int n = sendto(sockfd_tx, hmi_tx_buffer, 48,MSG_CONFIRM, 
			   (const struct sockaddr *) &servaddr_tx, sizeof(servaddr_tx));

        // set tello data here:
        pthread_mutex_lock(&tello_ctrl_mutex);
        tello->controller->set_hmi_forces(hdd);
        tello->controller->enable_human_dyn_data = sim_conf.en_human_control;
        pthread_mutex_unlock(&tello_ctrl_mutex);
        
		// END LOOP CODE FOR MUJOCO =====================================================================
		handle_end_of_periodic_task(next,period);
        
	}
    screen_recording = false;
    usbcam_recording = false;
    usbcam_hw_recording = false;
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    system("killall -2 ffmpeg");
    system("pkill -f -9 ffmpeg");
    usleep(100000);
    tcsetattr(STDIN_FILENO, TCSANOW, &originalSettings);
	// //free visualization storage
    // mjv_freeScene(&scn);
    // mjr_freeContext(&con);
    Human_dyn_data hdd;
    hdd.FxH_hmi = 0;
    hdd.FyH_hmi = 0;
    hdd.FxH_spring = 0;
    dash_utils::pack_data_to_hmi_with_ctrls((uint8_t*)hmi_tx_buffer,hdd,0,0,0,0);
    int n = sendto(sockfd_tx, hmi_tx_buffer, 48,MSG_CONFIRM, 
            (const struct sockaddr *) &servaddr_tx, sizeof(servaddr_tx));

    // // free MuJoCo model and data, deactivate
    // mj_deleteData(d);
    // mj_deleteModel(m);
    // mj_deactivate();

    exit(0);
    return NULL;
}

double xH_Commanded = 0;
int cmd_FSM = 0;
void* PS4_Controller( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    // PS4 Controller Setup code here:
    DualShock4<BLUETOOTH>* ds4;

    const Connection* connection = DualShock4Connector::GetConnectedDualShock4Device();
    if (connection->connectionType == BLUETOOTH){
        ds4 = new DualShock4<BLUETOOTH>(connection->hidDevice);
        PS4_connected = true;
    } 
    int print_cnt=0;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(1)
    {
        handle_start_of_periodic_task(next);

        // PS4 Controller repeating code here
        ds4->ReceiveInputDataPacket();
        if(print_cnt%5 == 0){
            // if(ds4->packet->circle) pause_sim = false;
            // if(ds4->packet->cross) pause_sim = true;
            // vx_desired_ps4 = (127-(double)ds4->packet->rightStick_Y)*(0.6/127.0);
            // vy_desired_ps4 = (127-(double)ds4->packet->rightStick_X)*(0.3/127.0);
            //if(ds4->packet->cross) pause_sim = true;
            xH_Commanded = (127.0-(double)ds4->packet->rightStick_Y)*(0.5/127.0);
            // cout << "xH: " << xH_Commanded << endl;
        }
        print_cnt++;

		handle_end_of_periodic_task(next,period);
	}
    return  0;
}

void* sim_step_task( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    while(!simulation_ready_to_run){usleep(100);}
    dash_utils::start_sim_timer();
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    // int mocapBodyIndex = mj_name2id(m, mjOBJ_BODY, "target");

    while(true)
    {
        handle_start_of_periodic_task(next);
        // dash_utils::start_timer();
        double elapsed = dash_utils::measure_sim_timer();
        dash_utils::start_sim_timer();
        double sim_dt = ((double)period+2.0)/1000000.0;
        // if(realtime_enabled)
        // {
        //     if(elapsed < 0.005)
        //         m->opt.timestep = sim_dt;//elapsed;
        //     else
        //         m->opt.timestep = 0.0005;
        // }
        // else{
        //     m->opt.timestep = 0.0005;
        // }
        m->opt.timestep = sim_dt;
        // pthread_mutex_lock(&sim_step_mutex);
        
        if(!pause_sim)
        {
            if(simulation_mode == 1)
            {
                if(start_target_motion)
                {
                    double delta_x;
                    
                    // Set the new position of the mocap body
                    delta_x = 0.0001;
                    if(d->mocap_pos[0] > 1.0) delta_x = 0.0002;
                    if(d->mocap_pos[0] > 3.0) delta_x = 0.0003;
                    if(d->mocap_pos[0] > 6.0) delta_x = 0.0000;
                    // if(d->mocap_pos[0] > 6.0) delta_x = 0.0000;
                    d->mocap_pos[0] = d->mocap_pos[0] + delta_x;  // Increment by 10mm (0.01m) in x-axis
                    

                    target_pos_out = Vector3d(d->mocap_pos[0],d->mocap_pos[1],d->mocap_pos[2]);
                    target_vel_out = Vector3d(delta_x*1000.0,0,0);
                }

                // dash_utils::print_timer();
                // dash_utils::start_timer();
                VectorXd tau_local;
                pthread_mutex_lock(&tau_share_mutex);
                tau_local = tau_shared;
                pthread_mutex_unlock(&tau_share_mutex);
                pthread_mutex_lock(&sim_mutex);
                // ARM TORQUES

                getArmPositionAndVelocities(m,d,tello->sim_arm_joint_pos,tello->sim_arm_joint_vel);

                // Printing arm joint positions
                // std::cout << "Arm Joint Positions:\n";
                // for (int i = 0; i < tello->sim_arm_joint_pos.size(); ++i) {
                //     std::cout << tello->sim_arm_joint_pos(i) << std::endl;
                // }

                // // Printing arm joint velocities
                // std::cout << "Arm Joint Velocities:\n";
                // for (int i = 0; i < tello->sim_arm_joint_vel.size(); ++i) {
                //     std::cout << tello->sim_arm_joint_vel(i) << std::endl;
                // }

                runArmControl();

                //END ARM TORQUES
                applyJointTorquesMujoco(tau_local);
                mj_step(m, d);
                dtime = d->time;
                sim_step_completed = true;


                const char* right_name_t = "rft"; 
                int geomIdr = mj_name2id(m, mjOBJ_GEOM, right_name_t);
                mjtNum right_foot_vel[6];
                mj_objectVelocity(m,d,mjOBJ_GEOM,geomIdr,right_foot_vel,0);
                right_toe_vel_mj << right_foot_vel[0],right_foot_vel[1],right_foot_vel[2],right_foot_vel[3],right_foot_vel[4],right_foot_vel[5];

                const char* left_name_t = "lft"; 
                int geomIdl = mj_name2id(m, mjOBJ_GEOM, left_name_t);
                mjtNum left_foot_vel[6];
                mj_objectVelocity(m,d,mjOBJ_GEOM,geomIdl,left_foot_vel,0);
                left_toe_vel_mj << left_foot_vel[0],left_foot_vel[1],left_foot_vel[2],left_foot_vel[3],left_foot_vel[4],left_foot_vel[5];

                const char* right_name_h = "rft"; 
                geomIdr = mj_name2id(m, mjOBJ_GEOM, right_name_h);
                mj_objectVelocity(m,d,mjOBJ_GEOM,geomIdr,right_foot_vel,0);
                right_heel_vel_mj << right_foot_vel[0],right_foot_vel[1],right_foot_vel[2],right_foot_vel[3],right_foot_vel[4],right_foot_vel[5];

                const char* left_name_h = "lft"; 
                geomIdl = mj_name2id(m, mjOBJ_GEOM, left_name_h);
                mj_objectVelocity(m,d,mjOBJ_GEOM,geomIdl,left_foot_vel,0);
                left_heel_vel_mj << left_foot_vel[0],left_foot_vel[1],left_foot_vel[2],left_foot_vel[3],left_foot_vel[4],left_foot_vel[5];

                // logging: 
                Traj_planner_dyn_data tpdd = tello->controller->get_traj_planner_dyn_data();
                SRB_Params srb_params = tello->controller->get_SRB_params();
                Human_params human_params = tello->controller->get_human_params();
                int FSM = tello->controller->get_FSM();
                double xR = tello->controller->get_x()(0);
                double dxR = tello->controller->get_x()(3);
                double pxR_beg_step = tpdd.st_beg_step(0);
                double xRlocal = xR - pxR_beg_step;
                double hR = srb_params.hLIP;
                double g = srb_params.g;
                double x_HWRM = tpdd.x_HWRM;
                double dx_HWRM = tpdd.dx_HWRM;
                double hH = human_params.hLIP;

                double wR = std::sqrt(g / hR);
                double wH = std::sqrt(g / hH);

                double xDCMRlocal = xRlocal + (dxR/wR);

                double xDCMHWRM = x_HWRM + (dx_HWRM/wH);
                Vector2d xDCM(xDCMHWRM/hH,xDCMRlocal/hR);

                VectorXd full_stance_swing_joint_torques;
                pthread_mutex_lock(&tau_share_mutex);
                full_stance_swing_joint_torques = tau_shared;
                pthread_mutex_unlock(&tau_share_mutex);
                x_out = tello->controller->get_x();
                u_out = tello->controller->get_GRFs();
                tau_out = tello->controller->get_joint_torques();
                full_tau_out = full_stance_swing_joint_torques;
                tau_ext_out = tello->controller->get_tau_ext();

                q_out = dash_utils::flatten(tello->controller->get_q());
                qd_out = dash_utils::flatten(tello->controller->get_qd());

                lfv_out = dash_utils::flatten(tello->controller->get_lfv_world());
                lfdv_out = dash_utils::flatten(tello->controller->get_lfdv_world());

                lfv_comm_out = dash_utils::flatten(tello->controller->get_lfv_comm_world());
                lfdv_comm_out = dash_utils::flatten(tello->controller->get_lfdv_comm_world());

                t_n_FSM_out = Eigen::Vector2d(tello->controller->get_time(),tello->controller->get_FSM());

                hdd_out = tello->controller->get_human_dyn_data();
                tpdd_out = tello->controller->get_traj_planner_dyn_data();

                impulse_out = Vector3d(push_force[0], push_force[1], push_force[2]);
                VectorXd meas_grf(4);
                meas_grf << tello->_GRFs.right_front, tello->_GRFs.right_back, tello->_GRFs.left_front, tello->_GRFs.left_back;
                meas_grf_out = meas_grf;

                xDCM_out = xDCM;

                log_data_ready = true;
                // end logging

                pthread_mutex_unlock(&sim_mutex);
                
                
            }
            else if(simulation_mode == 2)
            {
                d->time = d->time + elapsed;
                mj_kinematics(m,d);
            }
            else if(simulation_mode == 3)
            {
                pthread_mutex_lock(&sim_mutex);

                // Find the indices of the lights in the model
                int comLightIndex = mj_name2id(m, mjOBJ_LIGHT, "com_light");
                int backLightIndex = mj_name2id(m, mjOBJ_LIGHT, "back_light");

                // Set the position of com_light
                m->light_pos[3 * comLightIndex] = 30.0+tello->controller->get_x()(0);
                m->light_pos[3 * comLightIndex + 1] = 0.0;
                m->light_pos[3 * comLightIndex + 2] = 30.0;


                m->light_pos[3 * backLightIndex] = -30.0+tello->controller->get_x()(0);
                m->light_pos[3 * backLightIndex + 1] = 0.0;
                m->light_pos[3 * backLightIndex + 2] = 30.0;

                d->time = d->time + 0.001;
                tello->controller->set_time((double)(d->time));
                mj_kinematics(m,d);
                mj_forward(m,d);
                pthread_mutex_unlock(&sim_mutex);
            }
            else if(simulation_mode == 4)
            {
                pthread_mutex_lock(&sim_mutex);

                // Find the indices of the lights in the model
                int comLightIndex = mj_name2id(m, mjOBJ_LIGHT, "com_light");
                int backLightIndex = mj_name2id(m, mjOBJ_LIGHT, "back_light");

                // Set the position of com_light
                m->light_pos[3 * comLightIndex] = 30.0+tello->controller->get_x()(0);
                m->light_pos[3 * comLightIndex + 1] = 0.0;
                m->light_pos[3 * comLightIndex + 2] = 30.0;


                m->light_pos[3 * backLightIndex] = -30.0+tello->controller->get_x()(0);
                m->light_pos[3 * backLightIndex + 1] = 0.0;
                m->light_pos[3 * backLightIndex + 2] = 30.0;

                d->time = d->time + 0.001;
                tello->controller->set_time((double)(d->time));
                mj_kinematics(m,d);
                mj_forward(m,d);
                pthread_mutex_unlock(&sim_mutex);
            }
             
            pthread_mutex_lock(&sim_step_mutex);
            contactforce(m,d, controller->get_FSM());
            cd_shared = copyMjData(m,d);
            cd_shared.grf_rf = z_forces(0);
            cd_shared.grf_rb = z_forces(1);
            cd_shared.grf_lf = z_forces(2);
            cd_shared.grf_lb = z_forces(3);
            cd_shared_data_ready = true;
            pthread_mutex_unlock(&sim_step_mutex);
        }
        // pthread_mutex_unlock(&sim_step_mutex);
        // dash_utils::print_timer();
		handle_end_of_periodic_task(next,period);
	}
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    // mj_deleteData(d_shared);
    // mj_deleteModel(m_shared);
    mj_deactivate();
    usleep(2000000);
    exit(0);
    return  0;
}

std::string removeTextAfterLastSlash(const std::string& str) {
    std::size_t lastSlashPos = str.find_last_of('/');
    if (lastSlashPos != std::string::npos) {
        return str.substr(0, lastSlashPos + 1);
    }
    return str;
}

Human_dyn_data_filter hdd_pb_filter;

void* Human_Playback( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    std::string active_log = readActivePlaybackLog("/home/tello/Documents/GIT/Tello_Software/include/active_playback_log.json");
    // std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile("/home/tello/Desktop/tello_outputs/Logs/05-28-23__23-26-37/human_dyn_data.csv");
    // std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile("/home/tello/Desktop/tello_outputs/teleop/5-15_to_16/5-16-23-stepping-with-no-sim/human_dyn_data.csv");
    // std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile("/home/tello/Documents/hdd-tuning.csv");

    std::string logPath = removeTextAfterLastSlash(active_log);

    std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile(active_log);

    std::vector<Vector2d> time_vec = dash_utils::readTimeDataFromFile(logPath + "t_and_FSM.csv");

    hdd_cnt=0;

    Eigen::VectorXd xHvec(100);
    Eigen::VectorXd dxHvec(100);
    Eigen::VectorXd pxHvec(100);
    Eigen::VectorXd yHvec(100);
    Eigen::VectorXd dyHvec(100);
    Eigen::VectorXd pyHvec(100);
    Eigen::VectorXd fxH_Rvec(100);
    Eigen::VectorXd fyH_Rvec(100);
    Eigen::VectorXd fzH_Rvec(100);
    Eigen::VectorXd fxH_Lvec(100);
    Eigen::VectorXd fyH_Lvec(100);
    Eigen::VectorXd fzH_Lvec(100);
    Eigen::VectorXd fdxH_Rvec(100);
    Eigen::VectorXd fdyH_Rvec(100);
    Eigen::VectorXd fdzH_Rvec(100);
    Eigen::VectorXd fdxH_Lvec(100);
    Eigen::VectorXd fdyH_Lvec(100);
    Eigen::VectorXd fdzH_Lvec(100);
    Eigen::VectorXd FxH_hmi_vec(100);
    Eigen::VectorXd FyH_hmi_vec(100);
    Eigen::VectorXd FxH_spring_vec(100);
    double xHval;
    double dxHval;
    double pxHval;
    double yHval;
    double dyHval;
    double pyHval;
    double fxH_Rval;
    double fyH_Rval;
    double fzH_Rval;
    double fxH_Lval;
    double fyH_Lval;
    double fzH_Lval;
    double fdxH_Rval;
    double fdyH_Rval;
    double fdzH_Rval;
    double fdxH_Lval;
    double fdyH_Lval;
    double fdzH_Lval;
    double FxH_hmi_val;
    double FyH_hmi_val;
    double FxH_spring_val;
    int dyn_data_idx = 0;
   
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(1)
    {
        if(hdd_vec.size() == 0)
        {
            playback_error = true;
            break;
        }
        while(hdd_cnt < hdd_vec.size())
        {
            handle_start_of_periodic_task(next);
            if(hdd_cnt == 0 && (sim_conf.en_playback_mode))
            {
                double hR = tello->controller->get_SRB_params().hLIP;
                double hH = tello->controller->get_human_params().hLIP; 
                Human_dyn_data hdd0 = hdd_vec[0];
                double fyH_R = hdd0.fyH_R;
                double fyH_L = hdd0.fyH_L;

                double joystick_base_separation = 1.525;
                double foot_center_to_joystick = FOOT_2_JOYSTICK;
                double human_foot_width = joystick_base_separation - 2*foot_center_to_joystick - fyH_R - fyH_L;
                robot_init_foot_width = human_foot_width*(hR/hH);

            }
            if(playback_chosen && tello->controller->is_human_ctrl_enabled() && (!pause_sim) && (sim_conf.en_playback_mode) && !playback_changed)
            {
                //dash_utils::print_human_dyn_data(hdd_vec[hdd_cnt]);
                // if(PS4_connected) hdd_vec[hdd_cnt].xH = xH_Commanded;
                Human_dyn_data human_dyn_data = hdd_vec[hdd_cnt];
                
                double time = time_vec[hdd_cnt](0);
                
                // =======================================================================================================
                


                // =======================================================================================================
                if(time <= tello->controller->get_time())
                {
                    xHvec.tail(99) = xHvec.head(99).eval();
                    xHvec[0] = human_dyn_data.xH;

                    dxHvec.tail(99) = dxHvec.head(99).eval();
                    dxHvec[0] = human_dyn_data.dxH;

                    pxHvec.tail(99) = pxHvec.head(99).eval();
                    pxHvec[0] = human_dyn_data.pxH;

                    yHvec.tail(99) = yHvec.head(99).eval();
                    yHvec[0] = human_dyn_data.yH;

                    dyHvec.tail(99) = dyHvec.head(99).eval();
                    dyHvec[0] = human_dyn_data.dyH;

                    pyHvec.tail(99) = pyHvec.head(99).eval();
                    pyHvec[0] = human_dyn_data.pyH;

                    fxH_Rvec.tail(99) = fxH_Rvec.head(99).eval();
                    fxH_Rvec[0] = human_dyn_data.fxH_R;

                    fyH_Rvec.tail(99) = fyH_Rvec.head(99).eval();
                    fyH_Rvec[0] = human_dyn_data.fyH_R;

                    fzH_Rvec.tail(99) = fzH_Rvec.head(99).eval();
                    fzH_Rvec[0] = human_dyn_data.fzH_R;

                    fxH_Lvec.tail(99) = fxH_Lvec.head(99).eval();
                    fxH_Lvec[0] = human_dyn_data.fxH_L;

                    fyH_Lvec.tail(99) = fyH_Lvec.head(99).eval();
                    fyH_Lvec[0] = human_dyn_data.fyH_L;

                    fzH_Lvec.tail(99) = fzH_Lvec.head(99).eval();
                    fzH_Lvec[0] = human_dyn_data.fzH_L;

                    fdxH_Rvec.tail(99) = fdxH_Rvec.head(99).eval();
                    fdxH_Rvec[0] = human_dyn_data.fdxH_R;

                    fdyH_Rvec.tail(99) = fdyH_Rvec.head(99).eval();
                    fdyH_Rvec[0] = human_dyn_data.fdyH_R;

                    fdzH_Rvec.tail(99) = fdzH_Rvec.head(99).eval();
                    fdzH_Rvec[0] = human_dyn_data.fdzH_R;

                    fdxH_Lvec.tail(99) = fdxH_Lvec.head(99).eval();
                    fdxH_Lvec[0] = human_dyn_data.fdxH_L;

                    fdyH_Lvec.tail(99) = fdyH_Lvec.head(99).eval();
                    fdyH_Lvec[0] = human_dyn_data.fdyH_L;

                    fdzH_Lvec.tail(99) = fdzH_Lvec.head(99).eval();
                    fdzH_Lvec[0] = human_dyn_data.fdzH_L;

                    FxH_hmi_vec.tail(99) = FxH_hmi_vec.head(99).eval();
                    FxH_hmi_vec[0] = human_dyn_data.FxH_hmi;

                    FyH_hmi_vec.tail(99) = FyH_hmi_vec.head(99).eval();
                    FyH_hmi_vec[0] = human_dyn_data.FyH_hmi;

                    FxH_spring_vec.tail(99) = FxH_spring_vec.head(99).eval();
                    FxH_spring_vec[0] = human_dyn_data.FxH_spring;

                    xHval = dash_utils::smoothData(xHvec, 0.1/*alpha*/);
                    dxHval = dash_utils::smoothData(dxHvec, 0.1/*alpha*/);
                    pxHval = dash_utils::smoothData(pxHvec, 0.1/*alpha*/);
                    yHval = dash_utils::smoothData(yHvec, 0.1/*alpha*/);
                    dyHval = dash_utils::smoothData(dyHvec, 0.1/*alpha*/);
                    pyHval = dash_utils::smoothData(pyHvec, 0.1/*alpha*/);
                    fxH_Rval = dash_utils::smoothData(fxH_Rvec, 0.2/*alpha*/);
                    fyH_Rval = dash_utils::smoothData(fyH_Rvec, 0.2/*alpha*/);
                    fzH_Rval = dash_utils::smoothData(fzH_Rvec, 0.2/*alpha*/);
                    fxH_Lval = dash_utils::smoothData(fxH_Lvec, 0.2/*alpha*/);
                    fyH_Lval = dash_utils::smoothData(fyH_Lvec, 0.2/*alpha*/);
                    fzH_Lval = dash_utils::smoothData(fzH_Lvec, 0.2/*alpha*/);
                    fdxH_Rval = dash_utils::smoothData(fdxH_Rvec, 4.0/*alpha*/);
                    fdyH_Rval = dash_utils::smoothData(fdyH_Rvec, 4.0/*alpha*/);
                    fdzH_Rval = dash_utils::smoothData(fdzH_Rvec, 4.0/*alpha*/);
                    fdxH_Lval = dash_utils::smoothData(fdxH_Lvec, 4.0/*alpha*/);
                    fdyH_Lval = dash_utils::smoothData(fdyH_Lvec, 4.0/*alpha*/);
                    fdzH_Lval = dash_utils::smoothData(fdzH_Lvec, 4.0/*alpha*/);
                    FxH_hmi_val = dash_utils::smoothData(FxH_hmi_vec, 0.1/*alpha*/);
                    FyH_hmi_val = dash_utils::smoothData(FyH_hmi_vec, 0.1/*alpha*/);
                    FxH_spring_val = dash_utils::smoothData(FxH_spring_vec, 0.1/*alpha*/);

                    human_dyn_data.xH = xHval;
                    human_dyn_data.dxH = dxHval;
                    human_dyn_data.pxH = pxHval;
                    human_dyn_data.yH = yHval;
                    human_dyn_data.dyH = dyHval;
                    human_dyn_data.pyH = pyHval;
                    human_dyn_data.fxH_R = fxH_Rval;
                    human_dyn_data.fyH_R = fyH_Rval;
                    human_dyn_data.fzH_R = fzH_Rval;
                    human_dyn_data.fxH_L = fxH_Lval;
                    human_dyn_data.fyH_L = fyH_Lval;
                    human_dyn_data.fzH_L = fzH_Lval;
                    human_dyn_data.fdxH_R = fdxH_Rval;
                    human_dyn_data.fdyH_R = fdyH_Rval;
                    human_dyn_data.fdzH_R = fdzH_Rval;
                    human_dyn_data.fdxH_L = fdxH_Lval;
                    human_dyn_data.fdyH_L = fdyH_Lval;
                    human_dyn_data.fdzH_L = fdzH_Lval;
                    human_dyn_data.FxH_hmi = FxH_hmi_val;
                    human_dyn_data.FyH_hmi = FyH_hmi_val;
                    human_dyn_data.FxH_spring = FxH_spring_val;

                    if(PS4_connected && sim_conf.en_ps4_controller)
                        human_dyn_data.xH = xH_Commanded;

                    tello->controller->updateStepZHistoryL(fzH_Lval);
		            tello->controller->updateStepZHistoryR(fzH_Rval);
                    tello->controller->updateStepTimeHistory(time);

                    Traj_planner_dyn_data tpdds = tello->controller->get_traj_planner_dyn_data();
                    tpdds.step_z_history_L = tello->controller->getStepZHistoryL();
                    tpdds.step_z_history_R = tello->controller->getStepZHistoryR();
                    if(tpdds.human_FSM != 0)
                        tpdds.curr_SSP_sample_count = tpdds.curr_SSP_sample_count + 1;
                    tello->controller->set_traj_planner_step_data(tpdds);

                    // VectorXd alphas(21);
                    // alphas.setConstant(4.0);
                    // Human_dyn_data temp = dash_utils::smooth_human_dyn_data(human_dyn_data,hdd_pb_filter,alphas);
                    // human_dyn_data = temp;

                    tello->controller->set_human_dyn_data_without_forces(human_dyn_data);
                    hdd_cnt += 1;
                }
                // if(hdd_cnt == hdd_vec.size()-1) hdd_cnt--;
                //cout << "applying HDD struct # " << hdd_cnt << endl;
            }
            else if(sim_conf.en_playback_mode)
            {
                active_log = readActivePlaybackLog("/home/tello/Documents/GIT/Tello_Software/include/active_playback_log.json");
                hdd_vec = dash_utils::readHumanDynDataFromFile(active_log);
                hdd_cnt = 0;
                playback_changed = false;
                if(hdd_vec.size() == 0)
                {
                    playback_error = true;
                    break;
                }
            }
            double g = tello->controller->get_SRB_params().g;
            double hR = tello->controller->get_SRB_params().hLIP;
            double hH = tello->controller->get_human_params().hLIP;
            double wR = std::sqrt(g / hR);
            double wH = std::sqrt(g / hH);
            int periodScaled = (int)(((double)period)*(wH/wR));
            handle_end_of_periodic_task(next,50);
        }
        hdd_cnt = 0;
        tello->controller->disable_human_ctrl();
        cout << "Human Playback Complete" << endl;
        Traj_planner_dyn_data tpdd = tello->controller->get_traj_planner_dyn_data();
        tpdd.stepping_flg = false;
        tello->controller->set_traj_planner_dyn_data(tpdd);
        usleep(100000);
    }
   
    return  0;
}

void* Animate_Log( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    std::string logPath = removeTextAfterLastSlash(readActivePlaybackLog("/home/tello/Documents/GIT/Tello_Software/include/active_animation_log.json"));
    cout << "LogPath: " << logPath << endl;
    std::vector<Vector2d> time_vec = dash_utils::readTimeDataFromFile(logPath + "t_and_FSM.csv");
    std::vector<VectorXd> x_vec = dash_utils::readVectorXdfromCSV(logPath + "x.csv");
    std::vector<VectorXd> q_vec = dash_utils::readVectorXdfromCSV(logPath + "q.csv");
    std::vector<VectorXd> target_vec = dash_utils::readVectorXdfromCSV(logPath + "target_pos.csv");
    bool has_target_info = false;
    if(target_vec.size() > 0) has_target_info = true;

    usleep(1000000);
    pthread_mutex_lock(&sim_mutex);
    pthread_mutex_lock(&sim_step_mutex);
    Eigen::VectorXd& curr_x = x_vec[1]; // skip first row
    Eigen::VectorXd& curr_q = q_vec[1];
    MatrixXd q_tello(2,5);
    q_tello.row(0) = curr_q.head(5);
    q_tello.row(1) = curr_q.tail(5);
    tello->controller->set_q(q_tello);
    set_mujoco_state(curr_x);
    mj_kinematics(m,d);
    pthread_mutex_unlock(&sim_step_mutex);
    pthread_mutex_unlock(&sim_mutex);

    // d->qpos[hip_yaw_l_idx] = 0.0;
    // d->qpos[hip_roll_l_idx] = -0.04;
    // d->qpos[hip_pitch_l_idx] = -0.073+0.04+0.08;
    // d->qpos[knee_pitch_l_idx] = 0.28;
    // d->qpos[ankle_pitch_l_idx] = -0.19-0.04-0.04-0.08-0.01;
    // d->qpos[hip_yaw_r_idx] = 0.00;
    // d->qpos[hip_roll_r_idx] = -0.046;
    // d->qpos[hip_pitch_r_idx] = -0.55-0.12-0.08-0.12;
    // d->qpos[knee_pitch_r_idx] = 0.94+0.04+0.08+0.04+0.12;
    // d->qpos[ankle_pitch_r_idx] = -0.32-0.04-0.12;
    // d->qpos[torso_z_idx] = 0.028;
    // d->qpos[torso_x_idx] = 0.12;
    // d->qpos[torso_pitch_idx] = 0.04;
    // mj_kinematics(m,d);
    MatrixXd q_tello_init = q_tello;
    double hzl = q_tello_init(1,0);
    double hxl = q_tello_init(1,1);
    double hyl = q_tello_init(1,2);
    double kl  = q_tello_init(1,3);
    double al  = q_tello_init(1,4);

    double hzr = q_tello_init(0,0);
    double hxr = q_tello_init(0,1);
    double hyr = q_tello_init(0,2);
    double kr  = q_tello_init(0,3);
    double ar  = q_tello_init(0,4);
    double cam_angle = -135;
    double cam_dist = 2.0;
    double floor_height = 0.58; //COM_HEIGHT

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(1)
    {
        // if(hdd_vec.size() == 0)
        // {
        //     playback_error = true;
        //     break;
        // }
        int time_cnt = 0;
        while(time_cnt < time_vec.size() && simulation_mode == 3)
        {
            handle_start_of_periodic_task(next);
            // if(playback_chosen && tello->controller->is_human_ctrl_enabled() && (!pause_sim) && (sim_conf.en_playback_mode) && !playback_changed)
            // {
                
                double time = time_vec[time_cnt](0);
                
                if(time <= tello->controller->get_time())
                {
                    Eigen::VectorXd& curr_x = x_vec[time_cnt];
                    Eigen::VectorXd& curr_q = q_vec[time_cnt];
                    MatrixXd q_tello(2,5);
                    q_tello.row(0) = curr_q.head(5);
                    q_tello.row(1) = curr_q.tail(5);

                    // curr_x.segment<3>(3) = CoM_pos;
                    // curr_x.segment<3>(3) = CoM_rpy;
                    tello->controller->set_x(curr_x);
                    tello->controller->set_q(q_tello);

                    set_mujoco_state(curr_x);

                    cout << "CoM_vel: " << CoM_vel.transpose() << "       \r";

                    CoM_pos = curr_x.segment<3>(0);
                    d->mocap_pos[0] = CoM_pos(0);
                    d->mocap_pos[1] = CoM_pos(1);
                    d->mocap_pos[2] = CoM_pos(2);

                    Eigen::Quaterniond quat;

                    Eigen::Quaterniond quaternion(CoM_quat(0),CoM_quat(1),CoM_quat(2),CoM_quat(3));  // Quaternion (w, x, y, z)
                    // Convert quaternion to Euler angles

                    double roll = CoM_rpy(0);
                    double pitch = CoM_rpy(1);
                    double yaw = CoM_rpy(2);

                    // cout << "RPY: " << roll*180.0/M_PI << ",     " << pitch*180.0/M_PI << ",     " << yaw*180.0/M_PI << "             \r";
                    // cout.flush();

                    d->mocap_quat[0] = CoM_quat(0);
                    d->mocap_quat[1] = CoM_quat(1);
                    d->mocap_quat[2] = CoM_quat(2);
                    d->mocap_quat[3] = CoM_quat(3);

                    // if(has_target_info)
                    // {
                    //     const Eigen::VectorXd& target = target_vec[time_cnt];
                    //     d->mocap_pos[0] = target[0];
                    //     d->mocap_pos[1] = target[1];
                    //     d->mocap_pos[2] = floor_height;
                    // }
                    time_cnt++;
                    master_gain = 1.0;
                    sim_conf.en_human_control = true;
                }

            // }
            
            
            handle_end_of_periodic_task(next,50);
        }        
        d->time = 0;
        tello->controller->set_time((double)(d->time));
        pause_sim = true;
        master_gain = 0.0;
        sim_conf.en_human_control = false;
        usleep(100000);
    }
   
    return  0;
}




void nothing()
{
    
}
