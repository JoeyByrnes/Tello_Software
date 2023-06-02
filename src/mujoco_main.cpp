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
#include "IconsFontAwesome5.h"
#include "ImFileDialog.h"

pthread_mutex_t plotting_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sim_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t sim_step_mutex = PTHREAD_MUTEX_INITIALIZER;
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

extern struct termios originalSettings;

simConfig sim_conf;
int plot_width = 0;

double vx_desired_ps4 = 0;
double vy_desired_ps4 = 0;
bool PS4_connected = false;
bool enable_human_ff = false;
bool zero_human = false;
float master_gain = 0;
bool screen_recording = false;
bool usbcam_recording = false;
bool en_v2_ctrl = false;
bool en_safety_monitor = false;
bool bookmarked = false;
bool showCopyErrorPopup = false;
bool init_foot_width = false;
int hdd_cnt=0; // human_playback counter
std::string active_playback_log;
std::string active_playback_log_png;
int active_playback_log_index = 0;
std::vector<std::string> hddFiles;
bool showPlotMenu = false;

//logging
bool log_data_ready = false;
std::string log_folder;
VectorXd x_out, u_out, q_out, qd_out, tau_out, tau_ext_out, lfv_out, lfdv_out,lfv_comm_out,lfdv_comm_out, t_n_FSM_out;
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

VectorXd gnd_contacts(4);

// robot states indices
int torso_x_idx = 0;
int torso_y_idx = 1;
int torso_z_idx = 2;
int torso_roll_idx = 3;
int torso_pitch_idx = 4;
int torso_yaw_idx = 5;
int hip_yaw_r_idx = 6;
int hip_roll_r_idx = 7;
int hip_pitch_r_idx = 8;
int knee_pitch_r_idx = 9;
int ankle_pitch_r_idx = 10;
int hip_yaw_l_idx = 11;
int hip_roll_l_idx = 12;
int hip_pitch_l_idx = 13;
int knee_pitch_l_idx = 14;
int ankle_pitch_l_idx = 15;

// robot actuators indices
int hip_motor1_r_idx = 0;
int hip_motor1_l_idx = 1;
int hip_motor2_r_idx = 2;
int hip_motor2_l_idx = 3;
int hip_motor3_r_idx = 4;
int hip_motor3_l_idx = 5;
int knee_motor_r_idx = 6;
int knee_motor_l_idx = 7;
int ankle_motor_r_idx = 8;
int ankle_motor_l_idx = 9;

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

bool sim_was_restarted = false;
extern int simulation_mode;

extern int sockfd_tx;
extern char hmi_tx_buffer[100];
extern struct sockaddr_in servaddr_tx;

extern double robot_init_foot_width;

// begin SRBM-Ctrl Variables here ================================================================

SRBMController* controller;
MatrixXd lfv0(4,3), lfdv0(4,3); // global so planner can access them for now

// end SRBM-Ctrl Variables here ============================================================

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
// double t;
// double xR;
// double xdR;
// double yR;
// double ydR; 
// double zR;
// double zdR; 
// double phiR;
// double phidR;
// double thetaR;
// double thetadR;
// double psiR;
// double psidR;
// double q1l;
// double q2l;
// double q3l;
// double q4l;
// double q5l;
// double q1r;
// double q2r;
// double q3r;
// double q4r;
// double q5r;
// double qd1l;
// double qd2l;
// double qd3l;
// double qd4l;
// double qd5l;
// double qd1r;
// double qd2r;
// double qd3r;
// double qd4r;
// double qd5r;
// double t_end_stepping;
// mjtNum left_foot_toe[3];
// mjtNum left_foot_heel[3];
// mjtNum right_foot_toe[3];
// mjtNum right_foot_heel[3];
// mjtNum acceleration[3];
// mjtNum angular_velocity[3];
// VectorXd tau_LR_muxed(10);
// void mujoco_control_callback(const mjModel* m, mjData* d)
// {
//     // cout << "callback called" << endl;
//     // pthread_mutex_lock(&sim_step_mutex);
//     t = d->time;
//     controller->set_time(t);
//     // Get robot states
//     xR = d->qpos[torso_x_idx];
//     xdR = d->qvel[torso_x_idx];    
//     yR = d->qpos[torso_y_idx];
//     ydR = d->qvel[torso_y_idx];
//     zR = d->qpos[torso_z_idx];
//     zdR = d->qvel[torso_z_idx];
//     phiR = d->qpos[torso_roll_idx];
//     phidR = d->qvel[torso_roll_idx];   
//     thetaR = d->qpos[torso_pitch_idx];
//     thetadR = d->qvel[torso_pitch_idx];
//     psiR = d->qpos[torso_yaw_idx];
//     psidR = d->qvel[torso_yaw_idx];  
//     q1l = d->qpos[hip_yaw_l_idx];
//     q2l = d->qpos[hip_roll_l_idx];
//     q3l = d->qpos[hip_pitch_l_idx];             
//     q4l = d->qpos[knee_pitch_l_idx];   
//     q5l = d->qpos[ankle_pitch_l_idx];    
//     q1r = d->qpos[hip_yaw_r_idx];
//     q2r = d->qpos[hip_roll_r_idx];
//     q3r = d->qpos[hip_pitch_r_idx];             
//     q4r = d->qpos[knee_pitch_r_idx];   
//     q5r = d->qpos[ankle_pitch_r_idx];  

//     qd1l = d->qvel[hip_yaw_l_idx];
//     qd2l = d->qvel[hip_roll_l_idx];
//     qd3l = d->qvel[hip_pitch_l_idx];             
//     qd4l = d->qvel[knee_pitch_l_idx];   
//     qd5l = d->qvel[ankle_pitch_l_idx];    
//     qd1r = d->qvel[hip_yaw_r_idx];
//     qd2r = d->qvel[hip_roll_r_idx];
//     qd3r = d->qvel[hip_pitch_r_idx];             
//     qd4r = d->qvel[knee_pitch_r_idx];   
//     qd5r = d->qvel[ankle_pitch_r_idx];  

//     double t_end_stepping;

//     const char* lft = "lft";
//     const char* lfh = "lfh";
//     const char* rft = "rft";
//     const char* rfh = "rfh";
//     int geom_id = mj_name2id(m, mjOBJ_GEOM, lft);
//     memcpy(left_foot_toe, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
//     geom_id = mj_name2id(m, mjOBJ_GEOM, lfh);
//     memcpy(left_foot_heel, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
//     geom_id = mj_name2id(m, mjOBJ_GEOM, rft);
//     memcpy(right_foot_toe, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
//     geom_id = mj_name2id(m, mjOBJ_GEOM, rfh);
//     memcpy(right_foot_heel, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));

//     mujoco_lfv.row(0) = Vector3d(right_foot_toe[0],right_foot_toe[1],right_foot_toe[2]);
//     mujoco_lfv.row(1) = Vector3d(right_foot_heel[0],right_foot_heel[1],right_foot_heel[2]);
//     mujoco_lfv.row(2) = Vector3d(left_foot_toe[0],left_foot_toe[1],left_foot_toe[2]);
//     mujoco_lfv.row(3) = Vector3d(left_foot_heel[0],left_foot_heel[1],left_foot_heel[2]);   

//     VectorXd mujoco_lfv_vector = dash_utils::flatten(mujoco_lfv);
//     VectorXd lfv_comm_vector = dash_utils::flatten(controller->get_lfv_comm_world());
    
//     // contactforce(m,d, controller->get_FSM()); 
    
//     // Access the acceleration and angular velocity data from the sensors


//     int accel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "torso-linear-acceleration");
//     int gyro_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "toso-angular-velocity");

//     mju_copy3(acceleration, &d->sensordata[accel_sensor_id]);
//     mju_copy3(angular_velocity, &d->sensordata[gyro_sensor_id]);

//     // here is where controller runs async
//     applyJointTorquesMujoco(tau_LR_muxed);
//     // pthread_mutex_unlock(&sim_step_mutex);
// }

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

struct ThreadArgs {
    RoboDesignLab::DynamicRobot* tello;
    RoboDesignLab::TaskPDConfig config;
    VectorXd* torques;
};

// Define the thread function
void* taskPD2Thread(void* arg) {
    ThreadArgs* args = static_cast<ThreadArgs*>(arg);
    *(args->torques) = args->tello->taskPD2(args->config);
    return nullptr;
}

// void TELLO_locomotion_ctrl(const mjModel* m, mjData* d)
void TELLO_locomotion_ctrl(ctrlData cd)
{
    // Net wrench based PD controller with optimization-based force distribution
    // Simulation time
    // dash_utils::end_timer();
    // dash_utils::start_timer();
    // pthread_mutex_lock(&sim_step_mutex);
    // double t = d->time;
    // controller->set_time(t);
    // // Get robot states
    // double xR = d->qpos[torso_x_idx];
    // double xdR = d->qvel[torso_x_idx];    
    // double yR = d->qpos[torso_y_idx];
    // double ydR = d->qvel[torso_y_idx];
    // double zR = d->qpos[torso_z_idx];
    // double zdR = d->qvel[torso_z_idx];
    // double phiR = d->qpos[torso_roll_idx];
    // double phidR = d->qvel[torso_roll_idx];   
    // double thetaR = d->qpos[torso_pitch_idx];
    // double thetadR = d->qvel[torso_pitch_idx];
    // double psiR = d->qpos[torso_yaw_idx];
    // double psidR = d->qvel[torso_yaw_idx];  
    // double q1l = d->qpos[hip_yaw_l_idx];
    // double q2l = d->qpos[hip_roll_l_idx];
    // double q3l = d->qpos[hip_pitch_l_idx];             
    // double q4l = d->qpos[knee_pitch_l_idx];   
    // double q5l = d->qpos[ankle_pitch_l_idx];    
    // double q1r = d->qpos[hip_yaw_r_idx];
    // double q2r = d->qpos[hip_roll_r_idx];
    // double q3r = d->qpos[hip_pitch_r_idx];             
    // double q4r = d->qpos[knee_pitch_r_idx];   
    // double q5r = d->qpos[ankle_pitch_r_idx];  

    // double qd1l = d->qvel[hip_yaw_l_idx];
    // double qd2l = d->qvel[hip_roll_l_idx];
    // double qd3l = d->qvel[hip_pitch_l_idx];             
    // double qd4l = d->qvel[knee_pitch_l_idx];   
    // double qd5l = d->qvel[ankle_pitch_l_idx];    
    // double qd1r = d->qvel[hip_yaw_r_idx];
    // double qd2r = d->qvel[hip_roll_r_idx];
    // double qd3r = d->qvel[hip_pitch_r_idx];             
    // double qd4r = d->qvel[knee_pitch_r_idx];   
    // double qd5r = d->qvel[ankle_pitch_r_idx];  

    double t_end_stepping;
    controller->set_time(cd.t);

    // mjtNum left_foot_toe[3];
    // mjtNum left_foot_heel[3];
    // mjtNum right_foot_toe[3];
    // mjtNum right_foot_heel[3];
    // const char* lft = "lft";
    // const char* lfh = "lfh";
    // const char* rft = "rft";
    // const char* rfh = "rfh";
    // int geom_id = mj_name2id(m, mjOBJ_GEOM, lft);
    // memcpy(left_foot_toe, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
    // geom_id = mj_name2id(m, mjOBJ_GEOM, lfh);
    // memcpy(left_foot_heel, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
    // geom_id = mj_name2id(m, mjOBJ_GEOM, rft);
    // memcpy(right_foot_toe, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));
    // geom_id = mj_name2id(m, mjOBJ_GEOM, rfh);
    // memcpy(right_foot_heel, &d->geom_xpos[3*geom_id], 3*sizeof(mjtNum));

    // mujoco_lfv.row(0) = Vector3d(right_foot_toe[0],right_foot_toe[1],right_foot_toe[2]);
    // mujoco_lfv.row(1) = Vector3d(right_foot_heel[0],right_foot_heel[1],right_foot_heel[2]);
    // mujoco_lfv.row(2) = Vector3d(left_foot_toe[0],left_foot_toe[1],left_foot_toe[2]);
    // mujoco_lfv.row(3) = Vector3d(left_foot_heel[0],left_foot_heel[1],left_foot_heel[2]);   

    // VectorXd mujoco_lfv_vector = dash_utils::flatten(cd.mujoco_lfv);
    VectorXd lfv_comm_vector = dash_utils::flatten(controller->get_lfv_comm_world());
    //dash_utils::setOutputFolder("/home/joey/Desktop/tello_outputs/");
    //dash_utils::writeVectorToCsv(mujoco_lfv_vector,"mujoco_lfv.csv");
    //dash_utils::writeVectorToCsv(lfv_comm_vector,"lfv_comm.csv");
    
    // contactforce(m,d, controller->get_FSM()); 
    
    // Access the acceleration and angular velocity data from the sensors
    // mjtNum acceleration[3];
    // mjtNum angular_velocity[3];

    // int accel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "torso-linear-acceleration");
    // int gyro_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "toso-angular-velocity");

    // mju_copy3(acceleration, &d->sensordata[accel_sensor_id]);
    // mju_copy3(angular_velocity, &d->sensordata[gyro_sensor_id]);

    // Vector3d acc_no_g = subtractG(Vector3d(psiR,thetaR,phiR),Vector3d(acceleration[0],acceleration[1],acceleration[2]));
    Vector3d imu_acc = Vector3d(cd.acceleration[0],cd.acceleration[1],cd.acceleration[2]);
    Vector3d imu_gyro = Vector3d(cd.angular_velocity[0],cd.angular_velocity[1],cd.angular_velocity[2]);

    // Print the acceleration and angular velocity data
    // printf("Acceleration: (%f, %f, %f)\n", acceleration[0], acceleration[1], acceleration[2]);
    // printf("Angular Velocity: (%f, %f, %f)\n", angular_velocity[0], angular_velocity[1], angular_velocity[2]);

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

    VectorXd task_velocities = tello->joint_vel_to_task_vel(qdVec);

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
        // set_mujoco_state(tello->controller->get_x());
    }
    else
    {
        
        // VectorXd tau = controller->update(estimated_pc, estimated_dpc, EA_curr, imu_gyro,q ,qd ,time);
        // MatrixXd lfv_comm = controller->get_lfv_comm_world();
        // MatrixXd lfdv_comm = controller->get_lfdv_comm_world();
        
        // t_end_stepping = controller->get_SRB_params().t_end_stepping;  

        // filter debugging:
        // VectorXd pos_out(11),vel_out(11), EA_out(6), pc_out(9);
        //pos_out << pc_curr_plot, 0, filter_state.getPosition(), CoM_z, (pc_curr_plot-filter_state.getPosition());
        //dash_utils::writeVectorToCsv(pos_out,"pos_real_vs_filter.csv");

        //vel_out << dpc_curr, 0, filter_state.getVelocity(), 0, (dpc_curr-filter_state.getVelocity());
        //dash_utils::writeVectorToCsv(vel_out,"vel_real_vs_filter.csv");
        // BEGIN TASK PD CODE ======================================+++++++++++++++++

        Vector3d target_front_left = controller->get_lfv_comm_hip().row(2);
        Vector3d target_back_left = controller->get_lfv_comm_hip().row(3);
        Vector3d target_front_right = controller->get_lfv_comm_hip().row(0);
        Vector3d target_back_right = controller->get_lfv_comm_hip().row(1);

        Vector3d target_front_left_vel = controller->get_lfdv_comm_hip().row(2);
        Vector3d target_back_left_vel = controller->get_lfdv_comm_hip().row(3);
        Vector3d target_front_right_vel = controller->get_lfdv_comm_hip().row(0);
        Vector3d target_back_right_vel = controller->get_lfdv_comm_hip().row(1);

        VectorXd vel_desired(12);
        vel_desired  << target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel;

        VectorXd pos_desired(12);
        pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

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

            swing_pd_config.ignore_joint_velocity = true;
            swing_pd_config.task_ff_force = VectorXd::Zero(12);
            swing_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
            swing_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
            swing_pd_config.setTaskKp(0,10000,0);
            swing_pd_config.setTaskKd(0,80,0);
            swing_pd_config.setJointKp(kp_vec_joint_swing);
            swing_pd_config.setJointKd(kd_vec_joint_swing);
            swing_pd_config.motor_kp = VectorXd::Zero(10);
            swing_pd_config.motor_kd = VectorXd::Zero(10);
            
            swing_leg_torques = tello->taskPD2(swing_pd_config);
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
        posture_pd_config.setTaskKp(10000,10000,0);
        posture_pd_config.setTaskKd(80,80,0);
        posture_pd_config.setJointKp(kp_vec_joint_posture);
        posture_pd_config.setJointKd(kd_vec_joint_posture);

        VectorXd posture_ctrl_torques = tello->taskPD2(posture_pd_config);
        
        // END TASK PD CODE ======================================+++++++++++++++++
        VectorXd tau_LR(10);
        tau_LR << tau.tail(5), tau.head(5);
        tau_LR = tau_LR + posture_ctrl_torques;

        VectorXd torques_left  = tello->swing_stance_mux(tau_LR.head(5), swing_leg_torques.head(5),
                                                            0.00,controller->get_isSwingToStanceRight(), 
                                                            cd.t-controller->get_transitionStartRight(), 
                                                            0);
        VectorXd torques_right = tello->swing_stance_mux(tau_LR.tail(5), swing_leg_torques.tail(5),
                                                            0.00,controller->get_isSwingToStanceLeft(),
                                                            cd.t-controller->get_transitionStartLeft(), 
                                                            1);
        VectorXd tau_LR_muxed(10);
        tau_LR_muxed << torques_left,torques_right;
        tau_ready = false;
        pthread_mutex_lock(&tau_share_mutex);
        tau_shared = tau_LR_muxed;
        pthread_mutex_unlock(&tau_share_mutex);
        tau_ready = true;
        
        // applyJointTorquesMujoco(tau_LR_muxed);

        // begin update filter states: ------------------------------------------------------
        // RoboDesignLab::IMU_data imu_data;
        // imu_data.timestamp = t;
        // imu_data.acc = imu_acc;
        // imu_data.gyro = imu_gyro;
        // pthread_mutex_lock(&EKF_mutex);
        // filter_state = tello->get_filter_state();
        // pthread_mutex_unlock(&EKF_mutex);
        // tello->set_imu_data_for_ekf(imu_data);
        // tello->set_gnd_contact_data_for_ekf(gnd_contacts);
        // tello->set_lfv_hip_data_for_ekf(tello->controller->get_lfv_hip());
        // tello->set_q_data_for_ekf(q);
        // filter_data_ready = true;

        
        // Matrix3d Rwb_filter = filter_state.getRotation();
        // Vector3d origin_in_filter_frame = Rwb_filter*(-pc_curr);
        // if(camera_cnt%33 == 0)
        // {
        //   // tello->update_filter_landmark_data(1,origin_in_filter_frame);
        // }
        // camera_cnt++;
        // end update filter states: ------------------------------------------------------

        // cout << "Position Error: " << (pc_curr-filter_state.getPosition()).transpose() << endl;
        // cout << "contacts: " << gnd_contacts.transpose() << endl;
        // cout << "imu_acc: " << imu_acc.transpose() << endl;

    }
    // if(d->time > t_end_stepping+4 && stepping_in_progress)
    // {
    //     // stepping has finished, switch to balancing
    //     stepping_in_progress = false;
    //     pause_sim = true;
    // }
    
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

    if (std::filesystem::is_directory("/home/tello")) {
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
            dash_utils::parse_json_to_srb_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",srb_params);
            dash_utils::parse_json_to_pd_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",swing_conf,posture_conf);
            copyFile("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",log_folder);
        }
        if(simulation_mode == 2)
        {
            dash_utils::parse_json_to_srb_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config_SRBsim.json",srb_params);
            dash_utils::parse_json_to_pd_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config_SRBsim.json",swing_conf,posture_conf);
            copyFile("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config_SRBsim.json",log_folder);
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
    //cin.get();
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
        sim_time = srb_params.vx_des_t(srb_params.vx_des_t.size()-1);
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

// static float xH[5001], yH[5001], t[5001], dxH[5001], dyH[5001], pxH[5001], pyH[5001], u1z[5001],u2z[5001],u3z[5001],u4z[5001],FSM_[5001];

void DrawPlot()
{
    // glfwGetWindowSize(window, &windowWidth, &windowHeight);
    // ImPlot::StyleColorsLight();
    // ImGui::SetNextWindowSize(ImVec2((double)windowWidth/4.0,windowHeight-95));
    // ImGui::SetNextWindowPos(ImVec2(0,95));
    // plot_width = windowWidth/4.0+5;
    // ImGui::Begin("ImGraph",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);
    
    // static float xs1[5001], ys1[5001];
    // for (int i = 0; i <5000; i++) {
    //     if(!pause_sim){
    //         FSM_[i] = FSM_[i+1];
    //         // xH[i] = xH[i+1];
    //         // yH[i] = yH[i+1];
    //         // dxH[i] = dxH[i+1];
    //         // dyH[i] = dyH[i+1];
    //         // pxH[i] = pxH[i+1];
    //         // pyH[i] = pyH[i+1];
            
    //         // u1z[i] = u1z[i+1];
    //         // u2z[i] = u2z[i+1];
    //         // u3z[i] = u3z[i+1];
    //         // u4z[i] = u4z[i+1];
    //     }
    //     t[i] = i;
    // }
    // if(!pause_sim){
    //     xH[5000] = tello->controller->get_human_dyn_data().xH;
    //     yH[5000] = tello->controller->get_human_dyn_data().yH;
    //     dxH[5000] = tello->controller->get_human_dyn_data().dxH;
    //     dyH[5000] = tello->controller->get_human_dyn_data().dyH;
    //     pxH[5000] = tello->controller->get_human_dyn_data().pxH;
    //     pyH[5000] = tello->controller->get_human_dyn_data().pyH;
    //     u1z[5000] = tello->controller->get_GRFs()(2);
    //     u2z[5000] = tello->controller->get_GRFs()(5);
    //     u3z[5000] = tello->controller->get_GRFs()(8);
    //     u4z[5000] = tello->controller->get_GRFs()(11);
    // }
    //     t[5000] = 5000;
    // if (ImPlot::BeginPlot("Tello Real-Time Data",ImVec2((double)windowWidth/4.0-15,windowHeight-115))) {
    //     ImPlot::SetupAxes("x","y",ImPlotAxisFlags_NoDecorations,ImPlotAxisFlags_NoDecorations | ImPlotAxisFlags_AutoFit);

    //     // ImPlot::SetNextLineStyle(ImVec4(0.5, 0, 0, 1), 3.0f);
    //     // ImPlot::PlotLine("xH", t, xH, 5001,ImPlotLineFlags_None);
    //     // ImPlot::SetNextLineStyle(ImVec4(0, 0, 0.5, 1), 3.0f);
    //     // ImPlot::PlotLine("yH", t, yH, 5001,ImPlotLineFlags_None);

    //     // ImPlot::SetNextLineStyle(ImVec4(0.5, 0.5, 0, 1), 3.0f);
    //     // ImPlot::PlotLine("dxH", t, dxH, 5001,ImPlotLineFlags_None);
    //     // ImPlot::SetNextLineStyle(ImVec4(0, 0.5, 0.5, 1), 3.0f);
    //     // ImPlot::PlotLine("dyH", t, dyH, 5001,ImPlotLineFlags_None);

    //     // ImPlot::SetNextLineStyle(ImVec4(0.5, 0, 0.5, 1), 3.0f);
    //     // ImPlot::PlotLine("pxH", t, pxH, 5001,ImPlotLineFlags_None);
    //     // ImPlot::SetNextLineStyle(ImVec4(0.3, 0.3, 0.3, 1), 3.0f);
    //     // ImPlot::PlotLine("pyH", t, pyH, 5001,ImPlotLineFlags_None);

    //     ImPlot::SetNextLineStyle(ImVec4(0.5, 0, 0, 1), 3.0f);
    //     ImPlot::PlotLine("u1z", t, u1z, 5001,ImPlotLineFlags_None);
    //     ImPlot::SetNextLineStyle(ImVec4(0.3, 0.3, 0.3, 1), 3.0f);
    //     ImPlot::PlotLine("u2z", t, u2z, 5001,ImPlotLineFlags_None);
    //     ImPlot::SetNextLineStyle(ImVec4(0, 0, 0.5, 1), 3.0f);
    //     ImPlot::PlotLine("u3z", t, u3z, 5001,ImPlotLineFlags_None);
    //     ImPlot::SetNextLineStyle(ImVec4(0.3, 0, 0.3, 1), 3.0f);
    //     ImPlot::PlotLine("u4z", t, u4z, 5001,ImPlotLineFlags_None);

    //     ImPlot::EndPlot();
    // }
    // ImGui::End();
}
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
    while(!sim_ready_for_control) usleep(100);
    while(1)
    {
        handle_start_of_periodic_task(next);

        pthread_mutex_lock(&sim_step_mutex);
        ctrlData cd_local;
        cd_local = cd_shared;
        pthread_mutex_unlock(&sim_step_mutex);

        if(simulation_mode == 1)
        {
            if(!pause_sim){
                //mjtNum simstart = d->time;
                //while (d->time - simstart < 1.0 / 60.0){
                    //mj_step(m, d);
                    dash_utils::start_timer();
                    TELLO_locomotion_ctrl(cd_local);
                    dash_utils::print_timer();
                //}  
            } 
        }
        if(simulation_mode == 2)
        {
            if(!pause_sim){
                //mjtNum simstart = d->time;
                //while (d->time - simstart < 1.0 / 60.0){
                    // d->time = d->time + elapsed;
                                       
                    dash_utils::start_timer();
                    TELLO_locomotion_ctrl(cd_local);
                    dash_utils::print_timer();
                    set_mujoco_state(tello->controller->get_x());
                    // mj_kinematics(m,d);
                //}
            } 
            // cout << "CoM XYZ:" << tello->controller->get_x().head(3).transpose() << endl;
            //cout << "q right:" << tello->controller->get_q().row(0) << endl;
        }
        
        handle_end_of_periodic_task(next, period);
    }

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

    log_folder = createLogFolder("/home/joey/Desktop/tello_outputs/Logs/");
    dash_utils::setOutputFolder(log_folder);

    sim_conf = readSimConfigFromFile("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/sim_config.json");


    // INITIALIZE SRBM CONTROLLER ========================================================

    initializeSRBMCtrl();

    dash_utils::writeSRBParamsToTxt(tello->controller->get_SRB_params(),"srb_params.csv");
    dash_utils::writeHumanParamsToTxt(tello->controller->get_human_params(),"human_params.csv");

    // BEGIN SETUP CODE FOR MUJOCO ======================================================================
    
	// activate software
    // mj_activate("./lib/Mujoco/mjkey.txt");
    // mj_activate("tello_files/mjkey.txt");
    if (std::filesystem::is_directory("/home/tello")) {
        //std::cout << "The directory /home/tello exists!" << std::endl;
        mj_activate("tello_files/mjkey.txt");
        if(simulation_mode == 1)
        {
            m = mj_loadXML("./tello_files/tello-massive-color.xml", NULL, error, 1000);
            // m_shared = mj_loadXML("./tello_files/tello-massive-color.xml", NULL, error, 1000);
        }
        else
        {
            m = mj_loadXML("./tello_files/tello-massive-blue-grey.xml", NULL, error, 1000);
            // m_shared = mj_loadXML("./tello_files/tello-massive-blue-grey.xml", NULL, error, 1000);
        }
    }
    else {
        //std::cout << "The directory /home/tello does not exist!" << std::endl;
        mj_activate("./lib/Mujoco/mjkey.txt");
        if(simulation_mode == 1)
        {
            m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
            // m_shared = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
        }
        else
        {
            m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-blue-grey.xml", NULL, error, 1000);
            // m_shared = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-blue-grey.xml", NULL, error, 1000);
        }
        
    }
    // m = mj_loadXML("tello_files/tello-massive-color.xml", NULL, error, 1000);
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

    cam.elevation = -25;
    cam.distance = 2.0;
    cam.azimuth = -15;
    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = -0.2;

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
    if (std::filesystem::is_directory("/home/tello")) {
        //std::cout << "The directory /home/tello exists!" << std::endl;
        font = io.Fonts->AddFontFromFileTTF("./tello_files/fonts/roboto/Roboto-Light.ttf", baseFontSize);
    }
    else {
        //std::cout << "The directory /home/tello does not exist!" << std::endl;
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
    std::string plotfolderPath = "/home/joey/Desktop/tello_outputs/Favorite_Logs/Plots";
    std::vector<std::string> pngFiles;
    std::vector<GLuint> image_textures;
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
            hddFiles.push_back("/home/joey/Desktop/tello_outputs/Favorite_Logs/"+name.substr(0, name.length() - 4)+"/human_dyn_data.csv");
        }
    }

    // Print the filenames
    for (const auto& filename : hddFiles) {
        std::cout << filename << std::endl;
    }


    // dash_utils::start_sim_timer();

    
    
	while(!glfwWindowShouldClose(window))
    {
        handle_start_of_periodic_task(next);
		// BEGIN LOOP CODE FOR MUJOCO ===================================================================
        // double elapsed = dash_utils::measure_sim_timer();
        // if(realtime_enabled)
        // {
        //     m->opt.timestep = elapsed;
        // }
        // else{
        //      m->opt.timestep = 0.002;
        // }
        // dash_utils::start_sim_timer();
        // dash_utils::print_timer();
        // dash_utils::start_timer();
        // Get body ID
        // int body_id = mj_name2id(m, mjOBJ_BODY, "torso");

        // Apply force-torque to body
        // mjtNum force[3] = {push_force_x, push_force_y, -push_force_z}; // x, y, z components of the force
        // mjtNum torque[3] = {0.0, 0.0, 0.0}; // x, y, z components of the torque
        // mjtNum point[3] = {0.0, 0.0, 0.0}; // x, y, z components of the torque
        // pthread_mutex_lock(&sim_mutex);
        // mj_applyFT(m, d, force, torque, point, body_id, d->qfrc_applied);
        // pthread_mutex_unlock(&sim_mutex);

        // if(push_force_x > 0){
        //     printf("Pushed in X\n");
        //     push_force_x = 0;
        // }
        // if(push_force_y > 0){
        //     printf("Pushed in Y\n");
        //     push_force_y = 0;
        // }
        // if(push_force_z > 0){
        //     printf("Pushed in Z\n");
        //     push_force_z = 0;
        // }

        pthread_mutex_lock(&sim_step_mutex);
        ctrlData cd_local;
        cd_local = cd_shared;
        pthread_mutex_unlock(&sim_step_mutex);

        if(simulation_mode == 1)
        {
            if(!pause_sim){
                //mjtNum simstart = d->time;
                //while (d->time - simstart < 1.0 / 60.0){
                    //mj_step(m, d);
                    
                    dash_utils::print_timer();
                    dash_utils::start_timer();
                    TELLO_locomotion_ctrl(cd_local);
                    
                //}  
            } 
        }
        if(simulation_mode == 2)
        {
            if(!pause_sim){
                //mjtNum simstart = d->time;
                //while (d->time - simstart < 1.0 / 60.0){
                    // d->time = d->time + elapsed;
                    dash_utils::print_timer();
                    dash_utils::start_timer();
                    TELLO_locomotion_ctrl(cd_local);
                    
                    
                    // dash_utils::start_timer();

                    // dash_utils::print_timer();
                    set_mujoco_state(tello->controller->get_x());
                    // mj_kinematics(m,d);
                //}
            } 
            // cout << "CoM XYZ:" << tello->controller->get_x().head(3).transpose() << endl;
            //cout << "q right:" << tello->controller->get_q().row(0) << endl;
        }
        //tau_shared is ready to send back to sim thread here:


        // logging: 
        if(!pause_sim)
        {
            x_out = tello->controller->get_x();
            u_out = tello->controller->get_GRFs();
            tau_out = tello->controller->get_joint_torques();
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
            // cout << "xH_ref: " << tpdd_out.x_HWRM << "   dxH_ref: " << tpdd_out.dx_HWRM << endl;
            log_data_ready = true;
        }
        // end logging

        std::string text = "\tTello Mujoco Simulation    |    Test: " + recording_file_name + "    |    Time: " + std::to_string(d->time)+ "\t";
        glfwSetWindowTitle(window, text.c_str());

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        cam.lookat[0] = d->qpos[torso_x_idx];
        cam.lookat[1] = d->qpos[torso_y_idx];
        // set the background color to white
        
        // update scene and render
        pthread_mutex_lock(&sim_mutex);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        pthread_mutex_unlock(&sim_mutex);
        mjr_render(viewport, &scn, &con);
        
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
        ImGui::PushFont(font);
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
            ImGui::Checkbox(" " ICON_FA_USER_ALT_SLASH "  Boot to Autonomous Mode (RR) ", &(sim_conf.en_autonomous_mode_on_boot));
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
            ImGui::Checkbox(" " ICON_FA_CHART_LINE "  Display Realtime Plot   ", &(sim_conf.en_realtime_plot));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            if (ImGui::MenuItem(" " ICON_FA_SAVE "  Save Configuration"))
            { 
                writeSimConfigToFile(sim_conf, "/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/sim_config.json");
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();
        char notes[1001]; // String variable to store the entered notes
        if(bookmarked)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.84f, 0.0f, 1.0f)); // Set text color to gold
        }
        if (ImGui::BeginMenu(" " ICON_FA_STAR " "))
        {
            if(bookmarked) ImGui::PopStyleColor();
            ImGui::Separator();
            if(!pause_sim || screen_recording || usbcam_recording)
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
                    if(pause_sim && !screen_recording && !usbcam_recording)
                    {
                        std::string command = "cp -R " + log_folder + " /home/joey/Desktop/tello_outputs/Favorite_Logs/";
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


        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.3f, 0.0f, 1.0f)); // set button color
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.4f, 0.0f, 1.0f)); // set hover color
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.6f, 0.2f, 0.0f, 1.0f)); // set active color
        if (ImGui::Button(" " ICON_FA_UNDO " ")) {
            pause_sim = true;
            controller_unstable = false;
            last_Xf = 0;
            last_Yf = 0;
            last_springf = 0;
            tello->controller->disable_human_ctrl();
            hdd_cnt = 0;
            mj_resetData(m, d);
            d = mj_makeData(m);
            //tello->resetController();
            tello->controller->reset();
            initializeSRBMCtrl();
            set_mujoco_state(tello->controller->get_x());
            initializeLegs();
            pthread_mutex_lock(&sim_mutex);
            mj_forward(m, d);
            pthread_mutex_unlock(&sim_mutex);
        }
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor(4);
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
                tello->controller->disable_human_ctrl();
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
                // setenv("DISPLAY", ":1", 1);
                // const char* command = "taskset -c 0 ffmpeg -f x11grab -video_size 3840x2160 -framerate 30 -i :1 -c:v h264_nvenc -qp 0 output.mp4";
                // screen_record_pipe = popen(command, "w");
                // if (!screen_record_pipe) {
                //      std::cerr << "Error starting ffmpeg process!" << std::endl;
                // }
                

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
                system("killall -2 ffmpeg");
                
                recording_in_progress = false;
                usb_recording_in_progress = false;
                
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
        ImGui::Separator();
        if(!(sim_conf.en_autonomous_mode_on_boot))
        {
            std::string human_label;
            if(sim_conf.en_playback_mode) human_label = " Human-Playback   ";
            else human_label = " Human-Control   ";
            ImGui::Checkbox(human_label.c_str(), &tello->controller->enable_human_dyn_data);
            // ImGui::Separator();
            // ImGui::Checkbox(" Real-Time   ", &realtime_enabled);
            if(!(sim_conf.en_playback_mode))
            {
                ImGui::Separator();
                ImGui::Checkbox(" Force-Fdbk   ", &enable_human_ff);      // Edit bools storing our window open/close state
                ImGui::Separator();//init_foot_width
                ImGui::Checkbox(" Safety-Monitor   ", &sim_conf.en_safety_monitor);      // Edit bools storing our window open/close state
                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_WEIGHT " Tare CoM  ")) {
                    zero_human = true;
                }
                else{
                    zero_human = false;
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
                dash_init::init_foot_width(robot_init_foot_width,tello->controller->get_human_params(),
                                            tello->controller->get_SRB_params(),new_lfv0,q0);
                tello->controller->set_lfv0(new_lfv0);
                tello->controller->set_q0(q0);
                Human_params hp = tello->controller->get_human_params();
                hp.human_nom_ft_width = robot_init_foot_width;
                // double joystick_base_separation = 1.465;
                // double foot_center_to_joystick = 0.0635;
                // hp.fyH_home = (joystick_base_separation/2.0) - hp.human_nom_ft_width - foot_center_to_joystick; // joystick y width
                // cout << "FyH Home: " << hp.fyH_home << endl;
                tello->controller->set_human_params(hp);
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
        ImGui::PopFont();
        ImGui::PopStyleColor(4);

        // if(sim_conf.en_realtime_plot)
        //     DrawPlot();
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
            for(int i=0;i<pngFiles.size();i++)
            {
                ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
                ImGui::Separator();
                ImGui::PopStyleColor();
                if (ImGui::ImageButton((void*)(intptr_t)(image_textures[i]), ImVec2(plot_width, plot_height)))
                {
                    active_playback_log_png = pngFiles[i];
                    active_playback_log_index = i;
                    writeActivePlaybackLog(hddFiles[i],"/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/active_playback_log.json");
                }
            }
            ImGui::End();
        }
        

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // swap OpenGL buffers (blocking call due to v-sync)
        
        glfwSwapBuffers(window);
        
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();


        //handle UDP transmit here:
		Human_dyn_data hdd = tello->controller->get_human_dyn_data();
        if(sim_conf.en_safety_monitor)
        {
            if( (fabs(hdd.FxH_hmi - last_Xf) > 100) || (fabs(hdd.FyH_hmi - last_Yf) > 100) || (fabs(hdd.FxH_spring - last_springf) > 100)){
                controller_unstable = true;
            }
        }
        // if( (fabs(hdd.FyH_hmi - last_Yf) > 100) ){
        //     controller_unstable = true;
        // }
        last_Xf = hdd.FxH_hmi;
        last_Yf = hdd.FyH_hmi;
        last_springf = hdd.FxH_spring;

        // hdd.FyH_hmi = 20*sin(sin_cnt);
        // sin_cnt+= 0.016;
        // if(sin_cnt >= 2*M_PI) sin_cnt = 0;
        // sin_val = hdd.FyH_hmi;
        x_forces.tail(99) = x_forces.head(99).eval();
        x_forces[0] = hdd.FxH_hmi;
        y_forces.tail(99) = y_forces.head(99).eval();
        y_forces[0] = hdd.FyH_hmi;
        s_forces.tail(99) = s_forces.head(99).eval();
        s_forces[0] = hdd.FxH_spring;

        //dash_utils::start_timer();
        x_force = dash_utils::smoothData(x_forces,0.8);
        y_force = dash_utils::smoothData(y_forces,0.8);
        s_force = dash_utils::smoothData(s_forces,0.8);
        //dash_utils::print_timer();
        hdd.FyH_hmi = y_force;
        hdd.FxH_hmi = x_force;
        hdd.FxH_spring = s_force;
		if(tello->controller->is_human_ctrl_enabled() && !controller_unstable)
		{
			hdd.FxH_hmi = 0;
            // hdd.FxH_spring = 0;
		}
		else
		{
			hdd.FxH_hmi = 0;
			hdd.FyH_hmi = 0;
			hdd.FxH_spring = 0;
		}
		// cout << "Fx: " << hdd.FxH_hmi << "  Fy: " << hdd.FyH_hmi << endl;
        // dash_utils::pack_data_to_hmi_with_ctrls((uint8_t*)hmi_tx_buffer,hdd,enable_human_ff,zero_human,master_gain);
		// int n = sendto(sockfd_tx, hmi_tx_buffer, 20,MSG_CONFIRM, 
		// 	   (const struct sockaddr *) &servaddr_tx, sizeof(servaddr_tx));

		dash_utils::pack_data_to_hmi((uint8_t*)hmi_tx_buffer,hdd);
		int n = sendto(sockfd_tx, hmi_tx_buffer, 12,MSG_CONFIRM, 
			   (const struct sockaddr *) &servaddr_tx, sizeof(servaddr_tx));
        
		// END LOOP CODE FOR MUJOCO =====================================================================
		handle_end_of_periodic_task(next,period);
        
	}
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    system("killall -2 ffmpeg");
    tcsetattr(STDIN_FILENO, TCSANOW, &originalSettings);
	// //free visualization storage
    // mjv_freeScene(&scn);
    // mjr_freeContext(&con);

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
            xH_Commanded = (127-(double)ds4->packet->rightStick_Y)*(0.1/127.0);
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

    while(true)
    {
        handle_start_of_periodic_task(next);
        // dash_utils::start_timer();
        double elapsed = dash_utils::measure_sim_timer();
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
                // dash_utils::print_timer();
                // dash_utils::start_timer();
                VectorXd tau_local;
                pthread_mutex_lock(&tau_share_mutex);
                tau_local = tau_shared;
                pthread_mutex_unlock(&tau_share_mutex);
                applyJointTorquesMujoco(tau_local);
                pthread_mutex_lock(&sim_mutex);
                mj_step(m, d);
                pthread_mutex_unlock(&sim_mutex);
                
                
            }
            else
            {
                d->time = d->time + elapsed;
                mj_kinematics(m,d);
            }
            pthread_mutex_lock(&sim_step_mutex);
            cd_shared = copyMjData(m,d);
            pthread_mutex_unlock(&sim_step_mutex);
        }
        // pthread_mutex_unlock(&sim_step_mutex);
        // dash_utils::print_timer();
		handle_end_of_periodic_task(next,period);
	}
    //free visualization storage
    // mjv_freeScene(&scn);
    // mjr_freeContext(&con);

    // // free MuJoCo model and data, deactivate
    // mj_deleteData(d);
    // mj_deleteModel(m);
    // // mj_deleteData(d_shared);
    // // mj_deleteModel(m_shared);
    // mj_deactivate();

    exit(0);
    return  0;
}

void* Human_Playback( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    std::string active_log = readActivePlaybackLog("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/active_playback_log.json");
    // std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile("/home/joey/Desktop/tello_outputs/Logs/05-28-23__23-26-37/human_dyn_data.csv");
    // std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile("/home/joey/Desktop/tello_outputs/teleop/5-15_to_16/5-16-23-stepping-with-no-sim/human_dyn_data.csv");
    // std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile("/home/joey/Documents/hdd-tuning.csv");

    std::vector<Human_dyn_data> hdd_vec = dash_utils::readHumanDynDataFromFile(active_log);

    hdd_cnt=0;

    VectorXd  xHvec(100);
    VectorXd dxHvec(100);
    VectorXd pxHvec(100);
    VectorXd  yHvec(100);
    VectorXd dyHvec(100);
    VectorXd pyHvec(100);
    double xHval, dxHval, pxHval, yHval, dyHval, pyHval;
    int dyn_data_idx = 0;
   
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(1)
    {
        while(hdd_cnt < hdd_vec.size())
        {
            handle_start_of_periodic_task(next);
            if(tello->controller->is_human_ctrl_enabled() && (!pause_sim) && (sim_conf.en_playback_mode) )
            {
                //dash_utils::print_human_dyn_data(hdd_vec[hdd_cnt]);
                if(PS4_connected) hdd_vec[hdd_cnt].xH = xH_Commanded;
                Human_dyn_data human_dyn_data = hdd_vec[hdd_cnt];
                hdd_cnt += 1;
                //smooth data here
                // xHvec.tail(99) = xHvec.head(99).eval();
                // xHvec[0] = human_dyn_data.xH;
                // dxHvec.tail(99) = dxHvec.head(99).eval();
                // dxHvec[0] = human_dyn_data.dxH;
                // pxHvec.tail(99) = pxHvec.head(99).eval();
                // pxHvec[0] = human_dyn_data.pxH;

                // yHvec.tail(99) = yHvec.head(99).eval();
                // yHvec[0] = human_dyn_data.yH;
                // dyHvec.tail(99) = dyHvec.head(99).eval();
                // dyHvec[0] = human_dyn_data.dyH;
                // pyHvec.tail(99) = pyHvec.head(99).eval();
                // pyHvec[0] = human_dyn_data.pyH;
                

                // //dash_utils::start_timer();
                // xHval = dash_utils::smoothData(xHvec,1);
                // dxHval = dash_utils::smoothData(dxHvec,1);
                // pxHval = dash_utils::smoothData(pxHvec,1);
                // yHval = dash_utils::smoothData(yHvec,1);
                // dyHval = dash_utils::smoothData(dyHvec,1);
                // pyHval = dash_utils::smoothData(pyHvec,1);
                // //dash_utils::print_timer();
                // human_dyn_data.xH  =  xHval;
                // human_dyn_data.dxH = dxHval;
                // human_dyn_data.pxH = pxHval;
                // human_dyn_data.yH  =  yHval;
                // human_dyn_data.dyH = dyHval;
                // human_dyn_data.pyH = pyHval;

                // human_dyn_data.xH = 0;


                tello->controller->set_human_dyn_data(human_dyn_data);
                // if(hdd_cnt == hdd_vec.size()-1) hdd_cnt--;
                //cout << "applying HDD struct # " << hdd_cnt << endl;
            }
            else if(sim_conf.en_playback_mode)
            {
                active_log = readActivePlaybackLog("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/active_playback_log.json");
                hdd_vec = dash_utils::readHumanDynDataFromFile(active_log);
                hdd_cnt = 0;
            }
            double g = tello->controller->get_SRB_params().g;
            double hR = tello->controller->get_SRB_params().hLIP;
            double hH = tello->controller->get_human_params().hLIP;
            double wR = std::sqrt(g / hR);
            double wH = std::sqrt(g / hH);
            int periodScaled = (int)(((double)period)*(wH/wR));
            handle_end_of_periodic_task(next,periodScaled);
        }
        hdd_cnt = 0;
        tello->controller->disable_human_ctrl();
        cout << "Human Playback Complete" << endl;
        Traj_planner_dyn_data tpdd = tello->controller->get_traj_planner_dyn_data();
        tpdd.stepping_flg = false;
        tello->controller->set_traj_planner_dyn_data(tpdd);
        usleep(10000);
    }
   
    return  0;
}
double fY = 0;
void* logging( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    usleep(1000000);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(true)
    {
        handle_start_of_periodic_task(next);
        while(!log_data_ready) usleep(100);
        log_data_ready = false;
        // logging:
        dash_utils::writeVectorToCsv(x_out,"x.csv");
        dash_utils::writeVectorToCsv(u_out,"u.csv");
        dash_utils::writeVectorToCsv(tau_out,"tau.csv");
        dash_utils::writeVectorToCsv(tau_ext_out,"tau_ext.csv");

        dash_utils::writeVectorToCsv(q_out,"q.csv");
        dash_utils::writeVectorToCsv(qd_out,"qd.csv");

        dash_utils::writeVectorToCsv(lfv_out,"lfv.csv");
        dash_utils::writeVectorToCsv(lfdv_out,"lfdv.csv");

        dash_utils::writeVectorToCsv(lfv_comm_out,"lfv_comm.csv");
        dash_utils::writeVectorToCsv(lfdv_comm_out,"lfdv_comm.csv");

        dash_utils::writeVectorToCsv(t_n_FSM_out,"t_and_FSM.csv");

        dash_utils::writeHumanDynDataToCsv(hdd_out,"human_dyn_data.csv");
        dash_utils::writeTrajPlannerDataToCsv(tpdd_out,"traj_planner_dyn_data.csv");
        // end logging

		handle_end_of_periodic_task(next,period);
	}
    cout << "Human Playback Complete" << endl;
    return  0;
}
int recording_cnt = 0;
void* screenRecord( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    std::string cnt_str;

    usleep(1000000);
    std::string text = executeCommand("xwininfo -root -tree | grep \"Tello Mujoco\"");
    size_t paren_position = text.find(')');
    text = text.substr(paren_position+1);
    cout << "TEXT: " << text << endl;
    std::string win_pos;
    std::string win_size;

    std::regex sizeRegex(R"(\b(\d+x\d+)\b)");
    std::regex posRegex(R"( \+(\d+)\+(\d+))");

    std::smatch sizeMatch, posMatch;

    if (std::regex_search(text, sizeMatch, sizeRegex)) {
        win_size = sizeMatch[1].str();
        std::cout << "win_size: " << win_size << std::endl;
    }

    if (std::regex_search(text, posMatch, posRegex)) {
        win_pos = posMatch[1].str() + "," + posMatch[2].str();
        std::cout << "win_pos: " << win_pos << std::endl;
    }

    if(sim_conf.en_auto_record)
        screen_recording = true;

    // screen_rec_pid = fork();
    while(1)
    {

        while(!screen_recording || recording_in_progress || !(sim_conf.en_screen_recording)) usleep (1000);
        // screen_rec_pid = fork();
        if (!recording_in_progress) {

            std::string text = executeCommand("xwininfo -root -tree | grep \"Tello Mujoco\"");
            size_t paren_position = text.find(')');
            text = text.substr(paren_position+1);
            cout << "TEXT: " << text << endl;
            std::string win_pos;
            std::string win_size;

            std::regex sizeRegex(R"(\b(\d+x\d+)\b)");
            std::regex posRegex(R"( \+(\d+)\+(\d+))");

            std::smatch sizeMatch, posMatch;

            if (std::regex_search(text, sizeMatch, sizeRegex)) {
                win_size = sizeMatch[1].str();
                std::cout << "win_size: " << win_size << std::endl;
            }

            if (std::regex_search(text, posMatch, posRegex)) {
                win_pos = posMatch[1].str() + "," + posMatch[2].str();
                std::cout << "win_pos: " << win_pos << std::endl;
            }

            // Parsing str1
            std::istringstream iss1(win_size);
            std::string token1;
            int size_x, size_y;

            std::getline(iss1, token1, 'x');
            size_x = std::stoi(token1);

            std::getline(iss1, token1);
            size_y = std::stoi(token1)+72;

            // Parsing str2
            std::istringstream iss2(win_pos);
            std::string token2;
            int pos_x, pos_y;

            std::getline(iss2, token2, ',');
            pos_x = std::stoi(token2);

            std::getline(iss2, token2);
            pos_y = std::stoi(token2)-72;

            std::ostringstream oss1;
            oss1 << size_x << 'x' << size_y;
            std::string window_size = oss1.str();

            std::ostringstream oss2;
            oss2 << pos_x << ',' << pos_y;
            std::string window_pos = oss2.str();

            cout << "pos x: " << pos_x << "   pos y: " << pos_y << "   size x: " << size_x << "   size y: " << size_y << endl;
            recording_in_progress = true;
            // Child process - execute FFmpeg command
            cnt_str = to_string(recording_cnt);
            //execl("/usr/bin/ffmpeg", "ffmpeg", "-f", "x11grab", "-video_size", "3840x2160", /*"-loglevel", "quiet",*/ "-framerate", "30", "-i", ":1+eDP-1-1", "-c:v", "h264_nvenc", "-qp", "0", "ScreenCapture.mp4", NULL);
            //cout << "recording to: " << log_folder+"ScreenCapture_"+cnt_str+".mp4" << endl;
            system(("taskset -c 1 ffmpeg -f x11grab -video_size "+window_size+" -loglevel quiet -framerate 30 -i $DISPLAY+"+window_pos+" -c:v h264_nvenc -qp 0 " + log_folder+"ScreenCapture_"+cnt_str+".mp4").c_str());
            recording_cnt++;
        }
        usleep(10000);
    }

    return  0;
}
int usb_recording_cnt = 0;
void* usbCamRecord( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    std::string cnt_str;
    // screen_rec_pid = fork();
    usleep(1000000);
    if(sim_conf.en_auto_record)
        usbcam_recording = true;
    while(1)
    {

        while(!usbcam_recording || usb_recording_in_progress || !(sim_conf.en_HMI_recording)) usleep (1000);
        // screen_rec_pid = fork();
        if (!usb_recording_in_progress) {
            usb_recording_in_progress = true;
            // Child process - execute FFmpeg command
            cnt_str = to_string(usb_recording_cnt);
            system(("taskset -c 0 ffmpeg -f v4l2 -loglevel quiet -framerate 30 -video_size 800x600 -input_format mjpeg -i /dev/video4 -c:v copy " + log_folder+"usb_camera_"+cnt_str+".mp4").c_str());
            usb_recording_cnt++;
        }
        usleep(10000);
    }

    return  0;
}

void* plotting( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::vector<double> x, y1,y2,y3,y4,y5,y6,y7,y8,y9,y10,y11,y12;
    while(tello->controller->get_time() < 0.01){ usleep(1000); }

    int plotting_history = 50;
    double last_plot_time = 0;

    // Initialize the plot
    plt::backend("TkAgg");
    plt::figure_size(1200, 800);
    plt::rcparams({{"font.size", "20"}});
    plt::rcparams({{"lines.linewidth", "2"}});
    plt::subplots_adjust({{"wspace", 0.5}, {"hspace", 0.5}});
    plt::ion();
    plt::plot(x, y1);
    // Enable legend.
    plt::legend();
    plt::show();

    x.push_back(0);
    y1.push_back(0);
    y2.push_back(0);
    y3.push_back(0);
    y4.push_back(0);
    y5.push_back(0);
    y6.push_back(0);
    y7.push_back(0);
    y8.push_back(0);
    y9.push_back(0);
    y10.push_back(0);
    y11.push_back(0);
    y12.push_back(0);

    while(1)
    {
        handle_start_of_periodic_task(next);
    	// PLOTTING CODE HERE
        // Update the plot with new data
        
        if(tello->controller->get_time() > x[x.size()-1])
        {

            // TELEOP DATA ====================================================================================

            x.push_back(tello->controller->get_time());
            Human_dyn_data hdd = tello->controller->get_human_dyn_data();
            y1.push_back(hdd.FxH_hmi);
            y2.push_back(hdd.FyH_hmi);
            y3.push_back(hdd.FxH_spring);

            y4.push_back(x_force);
            y5.push_back(fY);
            y6.push_back(s_force);

            if(tello->controller->get_time() - last_plot_time > 0.1){
                last_plot_time = tello->controller->get_time();
                plt::rcparams({{"legend.loc","lower left"}});
                plt::clf();
                plt::title("HMI Force");
                //plt::subplot(3, 1, 1);
                //plt::named_plot("X", x, y1, "r-");
                //plt::named_plot("Y", x, y4, "b-");
                //plt::subplot(3, 1, 2);
                plt::named_plot("Y", x, y2, "r-");
                plt::named_plot("Log Y", x, y5, "b-");
                //plt::subplot(3, 1, 3);
                //plt::named_plot("S", x, y3, "r-");
                //plt::named_plot("Y", x, y6, "b-");
                plt::legend();
                plt::pause(0.001);
                
            }
            

            // POSITION DATA ==================================================================================
            // x.push_back(tello->controller->get_time());
            // // y1.push_back(tello->controller->get_pc()(0));
            // // y2.push_back(tello->controller->get_pc()(1));
            // // y3.push_back(tello->controller->get_pc()(2));

            // y4.push_back(tello->get_filter_state().getPosition()(0));
            // y5.push_back(tello->get_filter_state().getPosition()(1));
            // y6.push_back(tello->get_filter_state().getPosition()(2));

            // // y7.push_back(z_plotting(0));
            // // y8.push_back(z_plotting(1));
            // // y9.push_back(z_plotting(2));
            // // y10.push_back(z_plotting(3));
            // if(tello->controller->get_time() - last_plot_time > 0.3){
            //     last_plot_time = tello->controller->get_time();
            //     plt::clf();
            //     plt::subplot(3, 1, 1);
            //     plt::title("CoM Tracking");
            //     // plt::named_plot("CoM X", x, y1, "r-");
            //     plt::named_plot("EKF X", x, y4, "b-");
            //     plt::legend();
            //     plt::subplot(3, 1, 2);
            //     // plt::named_plot("CoM Y", x, y2, "r-");
            //     plt::named_plot("EKF Y", x, y5, "b-");
            //     plt::legend();
            //     plt::subplot(3, 1, 3);
            //     // plt::named_plot("CoM Z", x, y3, "r-");
            //     plt::named_plot("Kin Z", x, y6, "b-");
            //     plt::legend();
            //     plt::pause(0.001);
            // }
            // FOOT POSITION DATA =============================================================================
            // x.push_back(tello->controller->get_time());
            // Vector3d hip_right_pos_world = controller->get_right_leg_last().col(0);
            // Vector3d hip_left_pos_world = controller->get_left_leg_last().col(0);
            // Vector3d hip_orientation_world(tello->controller->get_EA());
            // VectorXd task_pos = tello->joint_pos_to_task_pos(tello->getJointPositions());
            // Vector3d task_right_front(task_pos(6),task_pos(7),task_pos(8));
            // Vector3d rf_wrld = dash_utils::hipToWorld(task_right_front,hip_right_pos_world,hip_orientation_world);

            // y1.push_back(rf_wrld(0));
            // y2.push_back(rf_wrld(1));
            // y3.push_back(rf_wrld(2));

            // Vector3d rf_hip = dash_utils::worldToHip(mujoco_lfv.row(0),hip_right_pos_world,hip_orientation_world);

            // y4.push_back(mujoco_lfv.row(0)(0)+0.002);
            // y5.push_back(mujoco_lfv.row(0)(1)+0.002);
            // y6.push_back(mujoco_lfv.row(0)(2)+0.002);

            // plt::clf();
            // plt::named_plot("task x", x, y1, "r-");
            // plt::named_plot("task y", x, y2, "b-");
            // plt::named_plot("task z", x, y3, "g-");

            // plt::named_plot("MJ wrld X", x, y4, "r--");
            // plt::named_plot("MJ wrld Y", x, y5, "b--");
            // plt::named_plot("MJ wrld Z", x, y6, "g--");
            // plt::title("Foot pos error");
            // plt::legend();
            // plt::pause(0.001);

            // VELOCITY and Position DATA ==================================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(dpc_curr(0));
            // y2.push_back(dpc_curr(1));
            // y3.push_back(dpc_curr(2));

            // y4.push_back(dx_smoothed);
            // y5.push_back(dy_smoothed);
            // y6.push_back(dz_smoothed);

            // // y7.push_back(pc_curr(0));
            // // y8.push_back(pc_curr(1));
            // // y9.push_back(pc_curr(2));

            // // y10.push_back(tello->get_filter_state().getPosition()(0));
            // // y11.push_back(tello->get_filter_state().getPosition()(1));
            // // y12.push_back(CoM_z_last);

            // if(tello->controller->get_time() - last_plot_time > 0.1){
            //     last_plot_time = tello->controller->get_time();
            //     plt::rcparams({{"legend.loc","lower left"}});
            //     plt::clf();
            //     // plt::subplot(3, 2, 1);
            //     // plt::title("CoM X Position True vs EKF");
            //     // plt::named_plot("True X", x, y7, "r-");
            //     // plt::named_plot("EKF X", x, y10, "b-");
            //     // plt::legend();
            //     // plt::subplot(3, 2, 3);
            //     // plt::title("CoM Y Position True vs EKF");
            //     // plt::named_plot("True Y", x, y8, "r-");
            //     // plt::named_plot("EKF Y", x, y11, "b-");
            //     // plt::legend();
            //     // plt::subplot(3, 2, 5);
            //     // plt::title("CoM Z Position True vs Kinematics");
            //     // plt::named_plot("True Z", x, y9, "r-");
            //     // plt::named_plot("Kin Z", x, y12, "b-");
            //     // plt::legend();
            //     plt::subplot(3, 1, 1);
            //     plt::title("CoM X Velocity True vs Estimated");
            //     plt::named_plot("True dX", x, y1, "r-");
            //     plt::named_plot("Est. dX", x, y4, "b-");
            //     plt::legend();
            //     plt::subplot(3, 1, 2);
            //     plt::title("CoM X Velocity True vs Estimated");
            //     plt::named_plot("True dY", x, y2, "r-");
            //     plt::named_plot("Est.  dY", x, y5, "b-");
            //     plt::legend();
            //     plt::subplot(3, 1, 3);
            //     plt::title("CoM X Velocity True vs Estimated");
            //     plt::named_plot("True dZ", x, y3, "r-");
            //     plt::named_plot("Est. dZ", x, y6, "b-");
            //     plt::legend();
            //     plt::pause(0.001);
            // }
            

            // ACCELEROMETER DATA: ==============================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->_acc(0));
            // y2.push_back(tello->_acc(1));
            // y3.push_back(tello->_acc(2));

            // // y4.push_back(tello->_acc(0));
            // // y5.push_back(tello->_acc(1));
            // // y6.push_back(tello->_acc(2));

            // plt::clf();
            // plt::named_plot("acc 0", x, y1, "r-");
            // plt::named_plot("acc 1", x, y2, "b-");
            // plt::named_plot("acc 2", x, y3, "g-");
            // plt::title("IMU Data");
            // plt::legend();
            // plt::pause(0.001);

            // GYRO DATA: =========================================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->_gyro(0));
            // y2.push_back(tello->_gyro(1));
            // y3.push_back(tello->_gyro(2));

            // y4.push_back(tello->controller->get_dEA()(0));
            // y5.push_back(tello->controller->get_dEA()(1));
            // y6.push_back(tello->controller->get_dEA()(2));

            // plt::clf();
            // plt::named_plot("gyro 0", x, y1, "r-");
            // plt::named_plot("gyro 1", x, y2, "b-");
            // plt::named_plot("gyro 2", x, y3, "g-");
            
            // plt::named_plot("dEA 0", x, y4, "r--");
            // plt::named_plot("dEA 1", x, y5, "b--");
            // plt::named_plot("dEA 2", x, y6, "g--");

            // plt::title("IMU Data");
            // plt::legend();
            // plt::pause(0.001);

            // ROTATION DATA: =======================================================================================
            // x.push_back(tello->controller->get_time());
            // // Vector3d euler_angles = filter_state.getRotation().eulerAngles(0, 1, 2); // XYZ order
            // // double roll = euler_angles(0);
            // // double pitch = euler_angles(1);
            // // double yaw = euler_angles(2);

            // Eigen::Quaterniond quat(filter_state.getRotation());
            // double roll, pitch, yaw;
            // Eigen::Matrix3d rotation = quat.toRotationMatrix();
            // if (std::abs(rotation(2, 0)) != 1) {
            //     pitch = -asin(rotation(2, 0));
            //     roll = atan2(rotation(2, 1) / cos(pitch), rotation(2, 2) / cos(pitch));
            //     yaw = atan2(rotation(1, 0) / cos(pitch), rotation(0, 0) / cos(pitch));
            // } else {
            //     pitch = rotation(2, 0) > 0 ? M_PI / 2 : -M_PI / 2;
            //     roll = 0;
            //     yaw = atan2(-rotation(1, 2), rotation(1, 1));
            // }

            // Eigen::Matrix3d rotation_matrix;
            // Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
            // Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            // Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

            // Eigen::Quaterniond quat2 = roll_angle * pitch_angle * yaw_angle;
            // rotation_matrix = quat2.toRotationMatrix();
            
            // y1.push_back(tello->controller->get_EA()(0));
            // y2.push_back(tello->controller->get_EA()(1));
            // y3.push_back(tello->controller->get_EA()(2));

            // y4.push_back(roll);
            // y5.push_back(pitch);
            // y6.push_back(yaw);

            // plt::clf();
            // plt::title("Rotation Data");
            // plt::named_plot("Roll", x, y1, "r-");
            // plt::named_plot("Pitch", x, y2, "b-");
            // plt::named_plot("Yaw", x, y3, "g-");
            // plt::named_plot("EKF_Roll", x, y4, "r--");
            // plt::named_plot("EKF Pitch", x, y5, "b--");
            // plt::named_plot("EKF Yaw", x, y6, "g--");
            // plt::legend();
            // plt::pause(0.001);

            // FOOT ROTATION: =======================================================================================
            // x.push_back(tello->controller->get_time());
            // // Vector3d euler_angles = filter_state.getRotation().eulerAngles(0, 1, 2); // XYZ order
            // // double roll = euler_angles(0);
            // // double pitch = euler_angles(1);
            // // double yaw = euler_angles(2);

            // Eigen::Quaterniond quat(tello->plot_mat);
            // double roll, pitch, yaw;
            // Eigen::Matrix3d rotation = quat.toRotationMatrix();
            // if (std::abs(rotation(2, 0)) != 1) {
            //     pitch = -asin(rotation(2, 0));
            //     roll = atan2(rotation(2, 1) / cos(pitch), rotation(2, 2) / cos(pitch));
            //     yaw = atan2(rotation(1, 0) / cos(pitch), rotation(0, 0) / cos(pitch));
            // } else {
            //     pitch = rotation(2, 0) > 0 ? M_PI / 2 : -M_PI / 2;
            //     roll = 0;
            //     yaw = atan2(-rotation(1, 2), rotation(1, 1));
            // }

            // Eigen::Matrix3d rotation_matrix;
            // Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
            // Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
            // Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());

            // Eigen::Quaterniond quat2 = roll_angle * pitch_angle * yaw_angle;
            // rotation_matrix = quat2.toRotationMatrix();
            
            // y1.push_back(tello->controller->get_EA()(0));
            // y2.push_back(tello->controller->get_EA()(1));
            // y3.push_back(tello->controller->get_EA()(2));

            // y4.push_back(roll);
            // y5.push_back(pitch);
            // y6.push_back(yaw);

            // plt::clf();
            // plt::title("Rotation Data");
            // plt::named_plot("Roll", x, y1, "r-");
            // plt::named_plot("Pitch", x, y2, "b-");
            // plt::named_plot("Yaw", x, y3, "g-");
            // plt::named_plot("foot Roll", x, y4, "r--");
            // plt::named_plot("foot Pitch", x, y5, "b--");
            // plt::named_plot("foot Yaw", x, y6, "g--");
            // plt::legend();
            // plt::pause(0.001);

            // FOOT POS DATA: =========================================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->plot_data(0));
            // y2.push_back(tello->plot_data(1));
            // y3.push_back(tello->plot_data(2));

            // y4.push_back(tello->plot_data(3));
            // y5.push_back(tello->plot_data(4));
            // y6.push_back(tello->plot_data(5));

            // y7.push_back(tello->plot_data(6));
            // y8.push_back(tello->plot_data(7));
            // y9.push_back(tello->plot_data(8));

            // y10.push_back(tello->plot_data(9));
            // y11.push_back(tello->plot_data(10));
            // y12.push_back(tello->plot_data(11));

            // plt::clf();
            // plt::subplot(3, 2, 1);
            // plt::title("Foot Positions");
            // plt::named_plot("RF X", x, y1, "r-");
            // plt::named_plot("RB X", x, y4, "b-");
            // plt::named_plot("LF X", x, y7, "g-");
            // plt::named_plot("LB X", x, y10, "y-");
            // plt::legend();
            // plt::subplot(3, 1, 2);
            // plt::named_plot("RF Y", x, y2, "r-");
            // plt::named_plot("RB Y", x, y5, "b-");
            // plt::named_plot("LF Y", x, y8, "g-");
            // plt::named_plot("LB Y", x, y11, "y-");
            // plt::subplot(3, 1, 3);
            // plt::named_plot("RF Z", x, y3, "r-");
            // plt::named_plot("RB Z", x, y6, "b-");
            // plt::named_plot("LF Z", x, y9, "g-");
            // plt::named_plot("LB Z", x, y12, "y-");
            // plt::legend();
            // plt::pause(0.001);

            // Uncomment the following lines for scope-like plotting:
            
            // if (x.size() > plotting_history) x.erase(x.begin());
            // if (y1.size() > plotting_history) y1.erase(y1.begin());
            // if (y2.size() > plotting_history) y2.erase(y2.begin());
            // if (y3.size() > plotting_history) y3.erase(y3.begin());
            // if (y4.size() > plotting_history) y4.erase(y4.begin());
            // if (y5.size() > plotting_history) y5.erase(y5.begin());
            // if (y6.size() > plotting_history) y6.erase(y6.begin());
            // if (y7.size() > plotting_history) y7.erase(y7.begin());
            // if (y8.size() > plotting_history) y8.erase(y8.begin());
            // if (y9.size() > plotting_history) y9.erase(y9.begin());
            // if (y10.size() > plotting_history) y10.erase(y10.begin());
            // if (y11.size() > plotting_history) y11.erase(y11.begin());
            // if (y12.size() > plotting_history) y12.erase(y12.begin());
        }
        usleep(100);
		//handle_end_of_periodic_task(next,period);
	}
    return  0;
}

void nothing(){}