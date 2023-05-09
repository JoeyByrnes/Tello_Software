#define WITHOUT_NUMPY
#include "mujoco_main.h"
#include "mujoco_utilities.h"
#include "state_estimator.h"

pthread_mutex_t plotting_mutex = PTHREAD_MUTEX_INITIALIZER;
extern pthread_mutex_t EKF_mutex;

namespace plt = matplotlibcpp;
GLFWwindow* window;

double vx_desired_ps4 = 0;
double vy_desired_ps4 = 0;
bool PS4_connected = false;

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

double push_force_x = 0;
double push_force_y = 0;
double push_force_z = 0;

bool sim_was_restarted = false;
extern int simulation_mode;

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

void TELLO_locomotion_ctrl(const mjModel* m, mjData* d)
{
    // Net wrench based PD controller with optimization-based force distribution
    // Simulation time
    // dash_utils::end_timer();
    // dash_utils::start_timer();
    double t = d->time;
    controller->set_time(t);
    // Get robot states
    double xR = d->qpos[torso_x_idx];
    double xdR = d->qvel[torso_x_idx];    
    double yR = d->qpos[torso_y_idx];
    double ydR = d->qvel[torso_y_idx];
    double zR = d->qpos[torso_z_idx];
    double zdR = d->qvel[torso_z_idx];
    double phiR = d->qpos[torso_roll_idx];
    double phidR = d->qvel[torso_roll_idx];   
    double thetaR = d->qpos[torso_pitch_idx];
    double thetadR = d->qvel[torso_pitch_idx];
    double psiR = d->qpos[torso_yaw_idx];
    double psidR = d->qvel[torso_yaw_idx];  
    double q1l = d->qpos[hip_yaw_l_idx];
    double q2l = d->qpos[hip_roll_l_idx];
    double q3l = d->qpos[hip_pitch_l_idx];             
    double q4l = d->qpos[knee_pitch_l_idx];   
    double q5l = d->qpos[ankle_pitch_l_idx];    
    double q1r = d->qpos[hip_yaw_r_idx];
    double q2r = d->qpos[hip_roll_r_idx];
    double q3r = d->qpos[hip_pitch_r_idx];             
    double q4r = d->qpos[knee_pitch_r_idx];   
    double q5r = d->qpos[ankle_pitch_r_idx];  

    double qd1l = d->qvel[hip_yaw_l_idx];
    double qd2l = d->qvel[hip_roll_l_idx];
    double qd3l = d->qvel[hip_pitch_l_idx];             
    double qd4l = d->qvel[knee_pitch_l_idx];   
    double qd5l = d->qvel[ankle_pitch_l_idx];    
    double qd1r = d->qvel[hip_yaw_r_idx];
    double qd2r = d->qvel[hip_roll_r_idx];
    double qd3r = d->qvel[hip_pitch_r_idx];             
    double qd4r = d->qvel[knee_pitch_r_idx];   
    double qd5r = d->qvel[ankle_pitch_r_idx];  

    double t_end_stepping;

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

    mujoco_lfv.row(0) = Vector3d(right_foot_toe[0],right_foot_toe[1],right_foot_toe[2]);
    mujoco_lfv.row(1) = Vector3d(right_foot_heel[0],right_foot_heel[1],right_foot_heel[2]);
    mujoco_lfv.row(2) = Vector3d(left_foot_toe[0],left_foot_toe[1],left_foot_toe[2]);
    mujoco_lfv.row(3) = Vector3d(left_foot_heel[0],left_foot_heel[1],left_foot_heel[2]);   

    VectorXd mujoco_lfv_vector = dash_utils::flatten(mujoco_lfv);
    VectorXd lfv_comm_vector = dash_utils::flatten(controller->get_lfv_comm_world());
    //dash_utils::setOutputFolder("/home/joey/Desktop/tello_outputs/");
    //dash_utils::writeVectorToCsv(mujoco_lfv_vector,"mujoco_lfv.csv");
    //dash_utils::writeVectorToCsv(lfv_comm_vector,"lfv_comm.csv");
    
    contactforce(m,d, controller->get_FSM()); 
    
    // Access the acceleration and angular velocity data from the sensors
    mjtNum acceleration[3];
    mjtNum angular_velocity[3];

    int accel_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "torso-linear-acceleration");
    int gyro_sensor_id = mj_name2id(m, mjOBJ_SENSOR, "toso-angular-velocity");

    mju_copy3(acceleration, &d->sensordata[accel_sensor_id]);
    mju_copy3(angular_velocity, &d->sensordata[gyro_sensor_id]);

    // Vector3d acc_no_g = subtractG(Vector3d(psiR,thetaR,phiR),Vector3d(acceleration[0],acceleration[1],acceleration[2]));
    Vector3d imu_acc = Vector3d(acceleration[0],acceleration[1],acceleration[2]);
    Vector3d imu_gyro = Vector3d(angular_velocity[0],angular_velocity[1],angular_velocity[2]);

    // Print the acceleration and angular velocity data
    // printf("Acceleration: (%f, %f, %f)\n", acceleration[0], acceleration[1], acceleration[2]);
    // printf("Angular Velocity: (%f, %f, %f)\n", angular_velocity[0], angular_velocity[1], angular_velocity[2]);

	// Torso state vectors
    VectorXd SRB_q(6);
    VectorXd SRB_qd(6);
    SRB_q << xR, yR, zR, phiR, thetaR, psiR;
    SRB_qd << xdR, ydR, zdR, phidR, thetadR, psidR;
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
    qLeg_l << q1l, q2l, q3l, q4l, q5l;
    VectorXd qLeg_r(5);
    qLeg_r << q1r, q2r, q3r, q4r, q5r;

    VectorXd qdLeg_l(5);
    qdLeg_l << qd1l, qd2l, qd3l, qd4l, qd5l;
    VectorXd qdLeg_r(5);
    qdLeg_r << qd1r, qd2r, qd3r, qd4r, qd5r;

    // Compute rotation matrix from world to body frame
    Matrix3d Rwb;
    Rwb = AngleAxisd(phiR, Vector3d::UnitX())
        * AngleAxisd(thetaR, Vector3d::UnitY())
        * AngleAxisd(psiR, Vector3d::UnitZ());
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
	EA_curr = Vector3d(phiR,thetaR,psiR);
    dEA_curr = Vector3d(phidR,thetadR,psidR);
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
        controller->set_lfdv_hip(lfdv_hip);

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

    dx_smoothed = smoothVelocity(x_vels,3);
    dy_smoothed = smoothVelocity(y_vels,3);
    dz_smoothed = smoothVelocity(z_vels,3);

    // Vector3d estimated_dpc(dx_smoothed,dy_smoothed,dz_smoothed);
    
    VectorXd tau = controller->update(pc_curr, dpc_curr, EA_curr, dEA_curr,q ,qd ,t);

    t_end_stepping = controller->get_SRB_params().t_end_stepping;
    if(tello->controller->get_sim_mode() == 2)
    {
        // set_mujoco_state(tello->controller->get_x());
    }
    else
    {
        // VectorXd tau = controller->update(estimated_pc, estimated_dpc, EA_curr, imu_gyro,q ,qd ,time);
        MatrixXd lfv_comm = controller->get_lfv_comm_world();
        MatrixXd lfdv_comm = controller->get_lfdv_comm_world();
        
        // t_end_stepping = controller->get_SRB_params().t_end_stepping;  

        // filter debugging:
        VectorXd pos_out(11),vel_out(11), EA_out(6), pc_out(9);
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

        // Set up configuration struct for Task Space Controller
        
        swing_pd_config.task_ff_force = VectorXd::Zero(12);
        swing_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
        swing_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
        swing_pd_config.setTaskKp(0,0,0);
        swing_pd_config.setTaskKd(0,0,0);
        swing_pd_config.setJointKp(kp_vec_joint_swing);
        swing_pd_config.setJointKd(kd_vec_joint_swing);
        swing_pd_config.motor_kp = VectorXd::Zero(10);
        swing_pd_config.motor_kd = VectorXd::Zero(10);
        
        VectorXd swing_leg_torques = tello->taskPD2(swing_pd_config);

        // Re-using
        posture_pd_config = swing_pd_config;
        posture_pd_config.setTaskKp(0,0,0);
        posture_pd_config.setTaskKd(0,0,0);
        posture_pd_config.setJointKp(kp_vec_joint_posture);
        posture_pd_config.setJointKd(kd_vec_joint_posture);

        VectorXd posture_ctrl_torques = tello->taskPD2(posture_pd_config);

        // END TASK PD CODE ======================================+++++++++++++++++
        VectorXd tau_LR(10);
        tau_LR << tau.tail(5), tau.head(5);
        tau_LR = tau_LR + posture_ctrl_torques;

        VectorXd torques_left  = tello->swing_stance_mux(tau_LR.head(5), swing_leg_torques.head(5),
                                                            0.00,controller->get_isSwingToStanceRight(), 
                                                            d->time-controller->get_transitionStartRight(), 
                                                            0);
        VectorXd torques_right = tello->swing_stance_mux(tau_LR.tail(5), swing_leg_torques.tail(5),
                                                            0.00,controller->get_isSwingToStanceLeft(),
                                                            d->time-controller->get_transitionStartLeft(), 
                                                            1);
        VectorXd tau_LR_muxed(10);
        tau_LR_muxed << torques_left,torques_right;


        applyJointTorquesMujoco(tau_LR_muxed);

        // begin update filter states: ------------------------------------------------------
        RoboDesignLab::IMU_data imu_data;
        imu_data.timestamp = t;
        imu_data.acc = imu_acc;
        imu_data.gyro = imu_gyro;
        pthread_mutex_lock(&EKF_mutex);
        filter_state = tello->get_filter_state();
        pthread_mutex_unlock(&EKF_mutex);
        tello->set_imu_data_for_ekf(imu_data);
        tello->set_gnd_contact_data_for_ekf(gnd_contacts);
        tello->set_lfv_hip_data_for_ekf(tello->controller->get_lfv_hip());
        tello->set_q_data_for_ekf(q);
        filter_data_ready = true;

        
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
    if(d->time > t_end_stepping+4 && stepping_in_progress)
    {
        // stepping has finished, switch to balancing
        stepping_in_progress = false;
        pause_sim = true;
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
    
    if(simulation_mode == 1)
        dash_utils::parse_json_to_srb_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",srb_params);

    if(simulation_mode == 2)
        dash_utils::parse_json_to_srb_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config_SRBsim.json",srb_params);

    dash_utils::parse_json_to_pd_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",swing_conf,posture_conf);
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
    
    //if(simulation_mode == 1)
    //{
        printf("Choose a Test:\n\n");
        printf("x: lean\n");
        printf("y: side2side\n");
        printf("z: squat\n");
        printf("r: roll\n");
        printf("p: pitch\n");
        printf("w: yaw\n");
        printf("b: balance\n");
        printf("s: stepping/walking\n");
        //cin.get();
        // char DoF;
        // cin.get(DoF);
        // if(DoF != 's')
        // {
        //     dash_planner::SRB_6DoF_Test(recording_file_name,sim_time,srb_params,lfv0,DoF,1);
        // }
        // else{
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
        // }

    //}
    //else if(simulation_mode == 2) srb_params.planner_type = 2;

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

    // INITIALIZE SRBM CONTROLLER ========================================================

    initializeSRBMCtrl();

    // BEGIN SETUP CODE FOR MUJOCO ======================================================================
    
	// activate software
    mj_activate("./lib/Mujoco/mjkey.txt");
    // mj_activate("tello_files/mjkey.txt");

    if(simulation_mode == 1)
    {
        m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
    }
    else
    {
        m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-blue-grey.xml", NULL, error, 1000);
    }
    // m = mj_loadXML("tello_files/tello-massive-color.xml", NULL, error, 1000);
	if (!m)
    {
        printf('r',"%s\n", error);
        exit(1);
    }
	// make data
    d = mj_makeData(m);

    initializeLegs();

    // install control callback
    mjcb_control = TELLO_locomotion_ctrl;


	// init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

		// create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1920, 1080, "Tello Mujoco Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    cam.elevation = -18;
    cam.distance = 1.8;
    cam.azimuth = 135;
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

    // Plotting:

	// END SETUP CODE FOR MUJOCO ========================================================================
    mj_step(m, d);

    // initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    ImGui::StyleColorsDark();
    ImFont* font = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/roboto/Roboto-Light.ttf", 50);
    
	while(!glfwWindowShouldClose(window))
    {
        handle_start_of_periodic_task(next);
		// BEGIN LOOP CODE FOR MUJOCO ===================================================================

        // Get body ID
        int body_id = mj_name2id(m, mjOBJ_BODY, "torso");

        // Apply force-torque to body
        mjtNum force[3] = {push_force_x, push_force_y, -push_force_z}; // x, y, z components of the force
        mjtNum torque[3] = {0.0, 0.0, 0.0}; // x, y, z components of the torque
        mjtNum point[3] = {0.0, 0.0, 0.0}; // x, y, z components of the torque
        mj_applyFT(m, d, force, torque, point, body_id, d->qfrc_applied);

        if(push_force_x > 0){
            printf("Pushed in X\n");
            push_force_x = 0;
        }
        if(push_force_y > 0){
            printf("Pushed in Y\n");
            push_force_y = 0;
        }
        if(push_force_z > 0){
            printf("Pushed in Z\n");
            push_force_z = 0;
        }
        if(simulation_mode == 1)
        {
            if(!pause_sim){
                mjtNum simstart = d->time;
                while (d->time - simstart < 1.0 / 60.0){
                    mj_step(m, d);
                }  
            } 
        }
        if(simulation_mode == 2)
        {
            if(!pause_sim){
                mjtNum simstart = d->time;
                while (d->time - simstart < 1.0 / 60.0){
                    d->time = d->time + 0.002;
                    TELLO_locomotion_ctrl(m,d);
                    set_mujoco_state(tello->controller->get_x());
                    mj_kinematics(m,d);
                }
            } 
            // cout << "CoM XYZ:" << tello->controller->get_x().head(3).transpose() << endl;
            //cout << "q right:" << tello->controller->get_q().row(0) << endl;
        }
        std::string text = "\tTello Mujoco Simulation    |    Test: " + recording_file_name + "    |    Time: " + std::to_string(d->time)+ "\t";
        glfwSetWindowTitle(window, text.c_str());

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);


        cam.lookat[0] = d->qpos[torso_x_idx];
        cam.lookat[1] = d->qpos[torso_y_idx];
        // set the background color to white

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // render ImGui GUI
        ImVec4 dark_navy = hex2ImVec4(0x082032);
        ImVec4 med_navy = hex2ImVec4(0x2C394B);
        ImVec4 light_navy = hex2ImVec4(0x334756);
        ImVec4 lighter_navy = hex2ImVec4(0x54748c);
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::SetNextWindowSizeConstraints(ImVec2(-1, 50), ImVec2(-1, FLT_MAX));
        // add toolbar with button
        ImGui::PushStyleColor(ImGuiCol_MenuBarBg, dark_navy);
        ImGui::PushStyleColor(ImGuiCol_Button, med_navy);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, light_navy);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, lighter_navy);
        ImGui::PushFont(font);
        ImGui::BeginMainMenuBar();
        if (ImGui::Button("  Restart  ")) {
            mj_resetData(m, d);
            //tello->resetController();
            tello->controller->reset();
            initializeSRBMCtrl();
            set_mujoco_state(tello->controller->get_x());
            initializeLegs();
            mj_step(m, d);
        }
        if(pause_sim)
        {
            if (ImGui::Button("    Run    ")) {
                pause_sim = false;
            }
        }
        else
        {
            if (ImGui::Button("  Pause  ")) {
                pause_sim = true;
            }
        }
        
        ImGui::EndMainMenuBar();
        ImGui::PopFont();
        ImGui::PopStyleColor();
        ImGui::PopStyleColor();
        ImGui::PopStyleColor();
        ImGui::PopStyleColor();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());


        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

		// END LOOP CODE FOR MUJOCO =====================================================================
		handle_end_of_periodic_task(next,period);
        dash_utils::start_timer();
	}

	//free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    exit(0);
    return NULL;
}

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
            if(ds4->packet->circle) pause_sim = false;
            if(ds4->packet->cross) pause_sim = true;
            vx_desired_ps4 = (127-(double)ds4->packet->rightStick_Y)*(0.6/127.0);
            vy_desired_ps4 = (127-(double)ds4->packet->rightStick_X)*(0.3/127.0);
        }
        print_cnt++;

		handle_end_of_periodic_task(next,period);
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
            x.push_back(tello->controller->get_time());
            y1.push_back(dpc_curr(0));
            y2.push_back(dpc_curr(1));
            y3.push_back(dpc_curr(2));

            y4.push_back(dx_smoothed);
            y5.push_back(dy_smoothed);
            y6.push_back(dz_smoothed);

            // y7.push_back(pc_curr(0));
            // y8.push_back(pc_curr(1));
            // y9.push_back(pc_curr(2));

            // y10.push_back(tello->get_filter_state().getPosition()(0));
            // y11.push_back(tello->get_filter_state().getPosition()(1));
            // y12.push_back(CoM_z_last);

            if(tello->controller->get_time() - last_plot_time > 0.1){
                last_plot_time = tello->controller->get_time();
                plt::rcparams({{"legend.loc","lower left"}});
                plt::clf();
                // plt::subplot(3, 2, 1);
                // plt::title("CoM X Position True vs EKF");
                // plt::named_plot("True X", x, y7, "r-");
                // plt::named_plot("EKF X", x, y10, "b-");
                // plt::legend();
                // plt::subplot(3, 2, 3);
                // plt::title("CoM Y Position True vs EKF");
                // plt::named_plot("True Y", x, y8, "r-");
                // plt::named_plot("EKF Y", x, y11, "b-");
                // plt::legend();
                // plt::subplot(3, 2, 5);
                // plt::title("CoM Z Position True vs Kinematics");
                // plt::named_plot("True Z", x, y9, "r-");
                // plt::named_plot("Kin Z", x, y12, "b-");
                // plt::legend();
                plt::subplot(3, 1, 1);
                plt::title("CoM X Velocity True vs Estimated");
                plt::named_plot("True dX", x, y1, "r-");
                plt::named_plot("Est. dX", x, y4, "b-");
                plt::legend();
                plt::subplot(3, 1, 2);
                plt::title("CoM X Velocity True vs Estimated");
                plt::named_plot("True dY", x, y2, "r-");
                plt::named_plot("Est.  dY", x, y5, "b-");
                plt::legend();
                plt::subplot(3, 1, 3);
                plt::title("CoM X Velocity True vs Estimated");
                plt::named_plot("True dZ", x, y3, "r-");
                plt::named_plot("Est. dZ", x, y6, "b-");
                plt::legend();
                plt::pause(0.001);
            }
            

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