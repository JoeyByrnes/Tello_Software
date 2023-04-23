#define WITHOUT_NUMPY
#include "mujoco_main.h"
#include "mujoco_utilities.h"

namespace plt = matplotlibcpp;
GLFWwindow* window;

extern RoboDesignLab::DynamicRobot* tello;
inekf::RobotState filter_state;
extern MatrixXd lfv_dsp_start;
Matrix3d rotation_mat;

char error[1000];

double sim_time;
bool pause_sim = true;
double stepping_in_progress = true;
double CoM_z_last = 0;

MatrixXd right_leg_last(3,5);
MatrixXd left_leg_last(3,5);
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

// begin SRBM-Ctrl Variables here ================================================================

SRBMController* controller;
MatrixXd lfv0(4,3), lfdv0(4,3); // global so planner can access them for now

// end SRBM-Ctrl Variables here ============================================================

void TELLO_locomotion_ctrl(const mjModel* m, mjData* d)
{
    // Net wrench based PD controller with optimization-based force distribution

    // Simulation time
    double time = d->time;
    controller->set_time(time);
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

    MatrixXd mujoco_lfv(4,3);
    mujoco_lfv.row(0) = Vector3d(right_foot_toe[0],right_foot_toe[1],right_foot_toe[2]);
    mujoco_lfv.row(1) = Vector3d(right_foot_heel[0],right_foot_heel[1],right_foot_heel[2]);
    mujoco_lfv.row(2) = Vector3d(left_foot_toe[0],left_foot_toe[1],left_foot_toe[2]);
    mujoco_lfv.row(3) = Vector3d(left_foot_heel[0],left_foot_heel[1],left_foot_heel[2]);   

    VectorXd mujoco_lfv_vector = dash_utils::flatten(mujoco_lfv);
    VectorXd lfv_comm_vector = dash_utils::flatten(controller->get_lfv_comm_world());
    dash_utils::setOutputFolder("/home/joey/Desktop/tello_outputs/");
    dash_utils::writeVectorToCsv(mujoco_lfv_vector,"mujoco_lfv.csv");
    dash_utils::writeVectorToCsv(lfv_comm_vector,"lfv_comm.csv");

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

    // CoM vector
    VectorXd pc_curr = SRB_q.head(3);
    VectorXd dpc_curr = SRB_qd.head(3);
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

    VectorXd imu_acc_world = Rwb*imu_acc;
    imu_acc_world +=Vector3d(0,0,9.81);

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
	VectorXd EA_curr = Vector3d(phiR,thetaR,psiR);
    VectorXd dEA_curr = Vector3d(phidR,thetadR,psidR);
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
    controller->set_lfdv_hip(lfdv_hip);

	// call SRBM-Ctrl here ======================================================================================
    double CoM_z = controller->get_CoM_z(controller->get_lfv_hip(),gnd_contacts,EA_curr); 
    if(CoM_z == -1) CoM_z = CoM_z_last;
    CoM_z_last = CoM_z;

    // Populate Controller inputs with estimated data instead of mujoco given data:
    Vector3d pc_curr_plot = pc_curr;

    Vector3d estimated_pc(filter_state.getPosition()(0),filter_state.getPosition()(1),CoM_z);
    Vector3d estimated_dpc(filter_state.getVelocity()(0),dpc_curr(1),filter_state.getVelocity()(1));

    VectorXd tau = controller->update(pc_curr, dpc_curr, EA_curr, dEA_curr,q ,qd ,time);
    MatrixXd lfv_comm = controller->get_lfv_comm_world();
    MatrixXd lfdv_comm = controller->get_lfdv_comm_world();
    
    double t_end_stepping = controller->get_SRB_params().t_end_stepping;  

    // filter debugging:
    VectorXd pos_out(11),vel_out(11), EA_out(6), pc_out(9);
    pos_out << pc_curr_plot, 0, filter_state.getPosition(), CoM_z, (pc_curr_plot-filter_state.getPosition());
    dash_utils::setOutputFolder("/home/joey/Desktop/tello_outputs/");
    dash_utils::writeVectorToCsv(pos_out,"pos_real_vs_filter.csv");

    vel_out << dpc_curr, 0, filter_state.getVelocity(), 0, (dpc_curr-filter_state.getVelocity());
    dash_utils::setOutputFolder("/home/joey/Desktop/tello_outputs/");
    dash_utils::writeVectorToCsv(vel_out,"vel_real_vs_filter.csv");
    

    // BEGIN TASK PD CODE ======================================+++++++++++++++++

    Joint_PD_config swing_conf, posture_conf;
    dash_utils::parse_json_to_pd_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",swing_conf,posture_conf);
    
    VectorXd kp_vec_joint(10);
	VectorXd kd_vec_joint(10);

    kp_vec_joint << swing_conf.hip_yaw_Kp, swing_conf.hip_roll_Kp,swing_conf.hip_pitch_Kp, swing_conf.knee_Kp, swing_conf.ankle_Kp,
                    swing_conf.hip_yaw_Kp, swing_conf.hip_roll_Kp,swing_conf.hip_pitch_Kp, swing_conf.knee_Kp, swing_conf.ankle_Kp;

    kd_vec_joint << swing_conf.hip_yaw_Kd, swing_conf.hip_roll_Kd,swing_conf.hip_pitch_Kd, swing_conf.knee_Kd, swing_conf.ankle_Kd,
                    swing_conf.hip_yaw_Kd, swing_conf.hip_roll_Kd,swing_conf.hip_pitch_Kd, swing_conf.knee_Kd, swing_conf.ankle_Kd;

	MatrixXd kp_mat_joint = kp_vec_joint.asDiagonal();
	MatrixXd kd_mat_joint = kd_vec_joint.asDiagonal();

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

	int task_kp = 0;
	int task_kd = 0;
	VectorXd kp_vec_task = VectorXd::Ones(12)*task_kp;
	VectorXd kd_vec_task = VectorXd::Ones(12)*task_kd;

    kp_vec_task(0) = 0;
    kp_vec_task(1) = 0;
    kp_vec_task(2) = 0;

    kp_vec_task(3) = kp_vec_task(0);
    kp_vec_task(4) = kp_vec_task(1);
    kp_vec_task(5) = kp_vec_task(2);

    kp_vec_task(6) = kp_vec_task(0);
    kp_vec_task(7) = kp_vec_task(1);
    kp_vec_task(8) = kp_vec_task(2);

    kp_vec_task(9) = kp_vec_task(0);
    kp_vec_task(10) = kp_vec_task(1);
    kp_vec_task(11) = kp_vec_task(2);

    kd_vec_task(0) = 0;
    kd_vec_task(1) = 0;
    kd_vec_task(2) = 0;

    kd_vec_task(3) = kd_vec_task(0);
    kd_vec_task(4) = kd_vec_task(1);
    kd_vec_task(5) = kd_vec_task(2);

    kd_vec_task(6) = kd_vec_task(0);
    kd_vec_task(7) = kd_vec_task(1);
    kd_vec_task(8) = kd_vec_task(2);

    kd_vec_task(9) = kd_vec_task(0);
    kd_vec_task(10) = kd_vec_task(1);
    kd_vec_task(11) = kd_vec_task(2);

	MatrixXd kp_mat_task = kp_vec_task.asDiagonal();
	MatrixXd kd_mat_task = kd_vec_task.asDiagonal();

	// Set up configuration struct for Task Space Controller
    RoboDesignLab::TaskPDConfig task_pd_config;
	task_pd_config.task_ff_force = VectorXd::Zero(12);
	task_pd_config.task_pos_desired = pos_desired;
	task_pd_config.task_vel_desired = vel_desired;
	task_pd_config.task_kp = kp_mat_task;
	task_pd_config.task_kd = kd_mat_task;
	task_pd_config.joint_kp = kp_mat_joint;
	task_pd_config.joint_kd = kd_mat_joint;
	task_pd_config.motor_kp = VectorXd::Zero(10);
	task_pd_config.motor_kd = VectorXd::Zero(10);
	
	VectorXd swing_leg_torques = tello->taskPD2(task_pd_config);

    task_kp = 0;
	task_kd = 0;
	kp_vec_task = VectorXd::Ones(12)*task_kp;
	kd_vec_task = VectorXd::Ones(12)*task_kd;
	kp_mat_task = kp_vec_task.asDiagonal();
	kd_mat_task = kd_vec_task.asDiagonal();

    kp_vec_joint << posture_conf.hip_yaw_Kp, posture_conf.hip_roll_Kp,posture_conf.hip_pitch_Kp, posture_conf.knee_Kp, posture_conf.ankle_Kp,
                    posture_conf.hip_yaw_Kp, posture_conf.hip_roll_Kp,posture_conf.hip_pitch_Kp, posture_conf.knee_Kp, posture_conf.ankle_Kp;

    kd_vec_joint << posture_conf.hip_yaw_Kd, posture_conf.hip_roll_Kd,posture_conf.hip_pitch_Kd, posture_conf.knee_Kd, posture_conf.ankle_Kd,
                    posture_conf.hip_yaw_Kd, posture_conf.hip_roll_Kd,posture_conf.hip_pitch_Kd, posture_conf.knee_Kd, posture_conf.ankle_Kd;

    kp_mat_joint = kp_vec_joint.asDiagonal();
	kd_mat_joint = kd_vec_joint.asDiagonal();
    task_pd_config.joint_kp = kp_mat_joint;
	task_pd_config.joint_kd = kd_mat_joint;

    VectorXd posture_ctrl_torques = tello->taskPD2(task_pd_config);

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
    imu_data.timestamp = time;
    imu_data.acc = imu_acc;
    imu_data.gyro = imu_gyro;

    Matrix3d R_foot_right = controller->get_foot_orientation_wrt_body(q.row(0));
    Matrix3d R_foot_left = controller->get_foot_orientation_wrt_body(q.row(1));

    tello->update_filter_IMU_data(imu_data);
    tello->update_filter_contact_data(gnd_contacts);
    tello->update_filter_kinematic_data(controller->get_lfv_hip(),R_foot_right,R_foot_left);
    
    // filter_state = tello->get_filter_state();
    // Vector3d euler_angles = filter_state.getRotation().eulerAngles(0, 1, 2); // ZYX order
    // double roll = euler_angles(0);
    // double pitch = euler_angles(1);
    // Matrix3d new_rotation_matrix;
    // new_rotation_matrix = AngleAxisd(roll, Vector3d::UnitX()) * AngleAxisd(pitch, Vector3d::UnitY()) * AngleAxisd(0, Vector3d::UnitZ());
    // filter_state.setRotation(new_rotation_matrix);
    // tello->set_filter_state(filter_state);


    // end update filter states: ------------------------------------------------------
    filter_state = tello->get_filter_state();

    // cout << "Position Error: " << (pc_curr-filter_state.getPosition()).transpose() << endl;
    // cout << "contacts: " << gnd_contacts.transpose() << endl;
    // cout << "imu_acc: " << imu_acc.transpose() << endl;


    if(d->time > t_end_stepping+4 && stepping_in_progress)
    {
        // stepping has finished, switch to balancing
        stepping_in_progress = false;
        pause_sim = true;
    }
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

    controller = tello->controller;
    SRB_Params srb_params = controller->get_SRB_params();
    Traj_planner_dyn_data traj_planner_dyn_data = controller->get_traj_planner_dyn_data();
    Human_params human_params = controller->get_human_params();
    VectorXd x0 = controller->get_x0();
    MatrixXd q0 = controller->get_q0();
    lfv0 = controller->get_lfv0();
    lfdv0 = controller->get_lfdv0();
    lfv_dsp_start = lfv0;

    dash_utils::parse_json_to_srb_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",srb_params);

    std::string recording_file_name;
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
    char DoF;
    cin.get(DoF);
    if(DoF != 's')
    {
        dash_planner::SRB_6DoF_Test(recording_file_name,sim_time,srb_params,lfv0,DoF,1);
    }
    else{
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

	// Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

    controller->set_SRB_params(srb_params);
    controller->set_traj_planner_dyn_data(traj_planner_dyn_data);    

    // BEGIN SETUP CODE FOR MUJOCO ======================================================================
    
	// activate software
    mj_activate("./lib/Mujoco/mjkey.txt");

	m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-massive-color.xml", NULL, error, 1000);
	if (!m)
    {
        printf('r',"%s\n", error);
        exit(1);
    }
	// make data
    d = mj_makeData(m);

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
    cam.elevation = -20;
    cam.distance = 1.8;
    cam.azimuth = 155;
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

    m->opt.timestep = 0.001;

    // Plotting:

	// END SETUP CODE FOR MUJOCO ========================================================================
    mj_step(m, d);
    
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

        if(!pause_sim){
            mjtNum simstart = d->time;
            while (d->time - simstart < 1.0 / 60.0){
                mj_step(m, d);
            }  
        } 
        std::string text = "\tTello Mujoco Simulation    |    Test: " + recording_file_name + "    |    Time: " + std::to_string(d->time)+ "\t";
        glfwSetWindowTitle(window, text.c_str());

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);


        cam.lookat[0] = d->qpos[torso_x_idx];
        cam.lookat[1] = 0;
        // set the background color to white

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

		// END LOOP CODE FOR MUJOCO =====================================================================
		handle_end_of_periodic_task(next,period);
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

void* plotting( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    std::vector<double> x, y1,y2,y3,y4,y5,y6;
    while(tello->controller->get_time() < 0.01){ usleep(1000); }

    // Initialize the plot
    plt::backend("TkAgg");
    plt::figure_size(1200, 800);
    plt::rcparams({{"font.size", "20"}});
    plt::rcparams({{"lines.linewidth", "2"}});
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

    while(!glfwWindowShouldClose(window))
    {
        handle_start_of_periodic_task(next);
    	// PLOTTING CODE HERE
        // Update the plot with new data

        if(tello->controller->get_time() > x[x.size()-1])
        {

            // POSITION DATA ==================================================================================
            x.push_back(tello->controller->get_time());
            y1.push_back(tello->controller->get_pc()(0));
            y2.push_back(tello->controller->get_pc()(1));
            y3.push_back(tello->controller->get_pc()(2));

            y4.push_back(tello->get_filter_state().getPosition()(0));
            y5.push_back(tello->get_filter_state().getPosition()(1));
            y6.push_back(tello->get_filter_state().getPosition()(2));

            plt::clf();
            plt::subplot(3, 1, 1);
            plt::title("CoM Tracking");
            plt::named_plot("CoM X", x, y1, "r-");
            plt::named_plot("EKF X", x, y4, "b");
            plt::legend();
            plt::subplot(3, 1, 2);
            plt::named_plot("CoM Y", x, y2, "r-");
            plt::named_plot("EKF Y", x, y5, "b");
            plt::legend();
            plt::subplot(3, 1, 3);
            plt::named_plot("CoM Z", x, y3, "r-");
            plt::named_plot("EKF Z", x, y6, "b");
            plt::legend();
            plt::pause(0.001);

             // VELOCITY DATA ==================================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->controller->get_dpc()(0));
            // y2.push_back(tello->controller->get_dpc()(1));
            // y3.push_back(tello->controller->get_dpc()(2));

            // y4.push_back(tello->get_filter_state().getVelocity()(0));
            // y5.push_back(tello->get_filter_state().getVelocity()(1));
            // y6.push_back(tello->get_filter_state().getVelocity()(2));

            // plt::clf();
            // plt::subplot(3, 1, 1);
            // plt::title("CoM Tracking");
            // plt::named_plot("CoM X", x, y1, "r-");
            // plt::named_plot("EKF X", x, y4, "b");
            // plt::legend();
            // plt::subplot(3, 1, 2);
            // plt::named_plot("CoM Y", x, y2, "r-");
            // plt::named_plot("EKF Y", x, y5, "b");
            // plt::legend();
            // plt::subplot(3, 1, 3);
            // plt::named_plot("CoM Z", x, y3, "r-");
            // plt::named_plot("EKF Z", x, y6, "b");
            // plt::legend();
            // plt::pause(0.001);

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
            // Vector3d euler_angles = filter_state.getRotation().eulerAngles(0, 1, 2); // XYZ order
            // double roll = euler_angles(0);
            // double pitch = euler_angles(1);
            // double yaw = euler_angles(2);
            
            // y1.push_back(tello->controller->get_EA()(0)*180.0/M_PI);
            // y2.push_back(tello->controller->get_EA()(1)*180.0/M_PI);
            // y3.push_back(tello->controller->get_EA()(2)*180.0/M_PI);

            // y4.push_back(roll*180.0/M_PI);
            // y5.push_back(pitch*180.0/M_PI);
            // y6.push_back(yaw*180.0/M_PI);

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

            // FOOT POS DATA: =========================================================================================
            // x.push_back(tello->controller->get_time());
            // y1.push_back(tello->plot_data(9));
            // y2.push_back(tello->plot_data(10));
            // y3.push_back(tello->plot_data(11));

            // plt::clf();
            // plt::named_plot("Xcom", x, y1, "r-");
            // plt::named_plot("Ycom", x, y2, "b-");
            // plt::named_plot("Zcom", x, y3, "g-");

            // plt::title("IMU Data");
            // plt::legend();
            // plt::pause(0.001);


        }
        

		handle_end_of_periodic_task(next,period);
	}
    return  0;
}