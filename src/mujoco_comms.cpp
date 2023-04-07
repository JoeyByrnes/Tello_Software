#include "mujoco_comms.h"

extern RoboDesignLab::DynamicRobot* tello;

char error[1000];

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

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

double push_force_x = 0;
double push_force_y = 0;
double push_force_z = 0;

// end SRBM-Ctrl Variables here ================================================================
SRB_Params srb_params;
Human_params human_params;
Human_dyn_data human_dyn_data;
Traj_planner_dyn_data traj_planner_dyn_data;
MatrixXd lfv0(4,3), lfdv0(4,3), q0(2,5), qd0(2,5);
VectorXd u0;
VectorXd x0(21);
// initialize loop variables (time, FSM, etc.)
double t = 0; // time variable
int FSM = 0;
int FSM_prev = 0; // previous FSM state

VectorXd x = x0; // SRB states
MatrixXd q = q0; // initial leg joint angles
MatrixXd qd = qd0; // initial leg joint velocities

MatrixXd lfv = lfv0; // initial end-effector positions (foot positions)
MatrixXd lfdv = lfdv0; // initial end-effector velocities (foot velocities)
VectorXd u = u0; // initial GRFs

VectorXd tau_ext(6); // external disturbance wrench

// initialize internal calculations
VectorXd tau(10);
MatrixXd SRB_state_ref(6, 2);
VectorXd tau_SRB_des(6);
VectorXd net_external_wrench(6);
VectorXd SRB_wrench_ref(6);
// Initialize 3x5x4 cube Jv Matrices
MatrixXd Jv_mat[4];

// begin SRBM-Ctrl Variables here ============================================================


void initial_legs_configuration(mjData* d)
{
    // set initial leg configuration
    d->qpos[hip_yaw_l_idx] = 0;
    d->qpos[hip_roll_l_idx] = 0;
    d->qpos[hip_pitch_l_idx] = -M_PI / 4;
    d->qpos[knee_pitch_l_idx] = M_PI / 2;
    d->qpos[ankle_pitch_l_idx] = -M_PI / 4;
    d->qpos[hip_yaw_r_idx] = 0;
    d->qpos[hip_roll_r_idx] = 0;
    d->qpos[hip_pitch_r_idx] = -M_PI / 4;
    d->qpos[knee_pitch_r_idx] = M_PI / 2;
    d->qpos[ankle_pitch_r_idx] = -M_PI / 4;

}


// Callbacks

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        mj_resetData(m, d);
        initial_legs_configuration(d);
        mj_forward(m, d);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_X)
    {
        push_force_x = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_Y)
    {
        push_force_y = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_Z)
    {
        push_force_z = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_C)
    {
        mju_zero(d->qfrc_applied, m->nv);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

VectorXd calc_wb(Vector3d dEA, VectorXd EA) {
  // Extract Euler angles from input vector
  double phi = EA(0);
  double theta = EA(1);
  double psi = EA(2);

  // Calculate rotation matrix from Euler angles
  Matrix3d Rmat;
  Rmat = AngleAxisd(phi, Vector3d::UnitX())
        * AngleAxisd(theta, Vector3d::UnitY())
        * AngleAxisd(psi, Vector3d::UnitZ());

  // Calculate inverse of T_EA
  double cos_theta = cos(theta);
  Matrix3d T_EA;
//   inv_T_EA << cos(psi)/cos_theta, -sin(psi), cos(psi)*tan(theta)/cos_theta,
//               sin(psi)/cos(theta), cos(psi), sin(psi)*tan(theta)/cos(theta),
//               0, 0, 1;
T_EA << cos(psi)/cos(theta), sin(psi)/cos(theta), 0,
      -sin(psi), cos(psi), 0,
      cos(psi)*tan(theta), sin(psi)*tan(theta), 1;

  // Calculate w from dEA using the inverse of T_EA
  Vector3d w = T_EA.inverse() * dEA;

  // Calculate wb from w and R
  VectorXd wb(3);
  wb = Rmat.transpose() * w;

  return wb;
}

Eigen::VectorXd calc_pd(VectorXd position, VectorXd velocity, VectorXd desiredPosition, VectorXd desiredVelocity, MatrixXd Kp, MatrixXd Kd) 
{
  // Compute position error
  Eigen::VectorXd positionError = desiredPosition - position;
  // Compute velocity error
  Eigen::VectorXd velocityError = desiredVelocity - velocity;
  // Compute control output
  return Kp*positionError + Kd*velocityError;
}

void TELLO_locomotion_ctrl(const mjModel* m, mjData* d)
{
    // Net wrench based PD controller with optimization-based force distribution

    // Simulation time
    double time = d->time;

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

	// Fill SRBM-Ctrl variables with Mujoco Variables here:
	q.row(0) = qLeg_r;
	q.row(1) = qLeg_l;
    qd.row(0) = qdLeg_r;
	qd.row(1) = qdLeg_l;
	Eigen::Map<Eigen::VectorXd> R_curr(Rwb.data(), Rwb.size());
	VectorXd EA_curr = Vector3d(phiR,thetaR,psiR);
    VectorXd dEA_curr = Vector3d(phidR,thetadR,psidR);
    VectorXd wb_curr = calc_wb(dEA_curr,EA_curr);

	x.segment(0, 3) = pc_curr;
	x.segment(3, 3) = dpc_curr;
	x.segment(6, 9) = R_curr;
	x.segment(15, 3) = wb_curr;
	x.segment(18, 3) = EA_curr;

    t = (double)d->time;

	// call SRBM-Ctrl here ======================================================================================

    // SRB FK
    MatrixXd torso_vertices(3,8);
    MatrixXd right_leg(5,1);
    MatrixXd left_leg(5,1);
    dash_kin::SRB_FK(torso_vertices, right_leg, left_leg, lfv, srb_params, x, q);
    

	// SRB kinematics
	dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);
    
	// SRB trajectory planner
	MatrixXd lfv_comm(4,3);
	MatrixXd lfdv_comm(4,3);
	dash_planner::SRB_Traj_Planner(srb_params, human_dyn_data, traj_planner_dyn_data,
								   human_params, FSM, FSM_prev, t, x, lfv, lfdv, u, 
								   tau_ext, SRB_state_ref, SRB_wrench_ref, lfv_comm, lfdv_comm);

    // Transform lfv an lfdv to hip frames here:
    double W = srb_params.W;
    double CoM2H_z_dist = srb_params.CoM2H_z_dist;
    MatrixXd lfv_comm_4x4(4,4);
	MatrixXd lfdv_comm_4x4(4,4);
    MatrixXd lfv_comm_body_4x4(4,4);
	MatrixXd lfdv_comm_body_4x4(4,4);
    MatrixXd lfv_comm_hip_4x4(4,4);
	MatrixXd lfdv_comm_hip_4x4(4,4);

    MatrixXd lfv_comm_body(4,3);
	MatrixXd lfdv_comm_body(4,3);
    MatrixXd lfv_comm_hip(4,3);
	MatrixXd lfdv_comm_hip(4,3);

    lfv_comm_4x4.setConstant(1);
    lfv_comm_4x4.block(0,0,4,3) = lfv_comm;
    lfdv_comm_4x4.setConstant(1);
    lfdv_comm_4x4.block(0,0,4,3) = lfdv_comm;

    // // Transform from mujoco world to body:
    Matrix3d R = Rwb;
    VectorXd pcom = pc_curr;

    Eigen::Matrix<double,4,4> HTMwd2com;
    //HTMwd2com << R, pcom, 0, 0, 0, 1;
    HTMwd2com.block(0,0,3,3) = R;
    HTMwd2com.block(0,3,3,1) = pcom;
    HTMwd2com.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();
    
    // right hip to world
    Eigen::Matrix4d HTMcom2hr;
    // HTMcom2hr.setIdentity();
    // HTMcom2hr(1,3) = -W/2.0;
    // HTMcom2hr(2,3) = -CoM2H_z_dist;
    HTMcom2hr.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    HTMcom2hr.block(0,3,3,1) = Eigen::Vector3d(0, -W/2.0, -CoM2H_z_dist);
    HTMcom2hr.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();

    Eigen::Matrix4d HTMwd2hr = HTMwd2com * HTMcom2hr;
    
    // left hip to world
    Eigen::Matrix4d HTMcom2hl;
    // HTMcom2hl.setIdentity();
    // HTMcom2hl(1,3) = W/2.0;
    // HTMcom2hl(2,3) = -CoM2H_z_dist;
    HTMcom2hl.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    HTMcom2hl.block(0,3,3,1) = Eigen::Vector3d(0, W/2.0, -CoM2H_z_dist);
    HTMcom2hl.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();

    Eigen::Matrix4d HTMwd2hl = HTMwd2com * HTMcom2hl;

    lfv_comm_hip.row(0) = (HTMwd2hr.inverse() * lfv_comm_4x4.row(0).transpose()).head(3);
    lfv_comm_hip.row(1) = (HTMwd2hr.inverse() * lfv_comm_4x4.row(1).transpose()).head(3);

    lfv_comm_hip.row(2) = (HTMwd2hl.inverse() * lfv_comm_4x4.row(2).transpose()).head(3);
    lfv_comm_hip.row(3) = (HTMwd2hl.inverse() * lfv_comm_4x4.row(3).transpose()).head(3);

    Vector3d right_front = lfv_comm_hip.row(0);
    Vector3d right_back = lfv_comm_hip.row(1);
    Vector3d left_front = lfv_comm_hip.row(2);
    Vector3d left_back = lfv_comm_hip.row(3);

    // BEGIN TASK PD CODE ======================================+++++++++++++++++

    int joint_kp = 1000;
	int joint_kd = 0;
	VectorXd kp_vec_joint = VectorXd::Ones(10)*(joint_kp);
	VectorXd kd_vec_joint = VectorXd::Ones(10)*joint_kd;

	MatrixXd kp_mat_joint = kp_vec_joint.asDiagonal();
	MatrixXd kd_mat_joint = kd_vec_joint.asDiagonal();

	int motor_kp = 0;
	int motor_kd = 1000;
	VectorXd kp_vec_motor = VectorXd::Ones(10)*motor_kp;
	VectorXd kd_vec_motor = VectorXd::Ones(10)*motor_kd;

	VectorXd vel_desired = VectorXd::Zero(12);

	
	Vector3d target_front_left = left_front;
	Vector3d target_back_left = left_back;
	Vector3d target_front_right = right_front;
	Vector3d target_back_right = right_back;

	VectorXd pos_desired(12);
	pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

	int task_kp = 0;
	int task_kd = 0;
	VectorXd kp_vec_task = VectorXd::Ones(12)*task_kp;
	VectorXd kd_vec_task = VectorXd::Ones(12)*task_kd;
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
	task_pd_config.motor_kp = kp_vec_motor;
	task_pd_config.motor_kd = kd_vec_motor;

    VectorXd qVec(10), qdVec(10);
    qVec << q.row(1).transpose(), q.row(0).transpose();
    qdVec << qd.row(1).transpose(), qd.row(0).transpose();

    tello->sim_joint_pos << qVec;
    tello->sim_joint_vel << qdVec;
	
	tello->taskPD(task_pd_config);

    // END TASK PD CODE ======================================+++++++++++++++++
    
    // cout << "lfv_comm: " << endl;
    // cout << lfv_comm << endl;

    // cout << "lfv_comm_hip: " << endl;
    // cout << lfv_comm_hip << endl;

    // cout << "==================================" << endl;
    // cout.flush();

	// SRB controller
	dash_ctrl::SRB_Balance_Controller(u, tau, srb_params, FSM, x, lfv, qd, Jv_mat, u, SRB_wrench_ref);

    // for(int i = 0; i< 10; i++){
    //     printf("%f, ",tau(i));
    // }
    // printf("\n");
    // printf("%f,\t ",(float)d->time);
    // for(int i = 0; i< 12; i++){
    //     printf("%f, ",u(i));
    // }
    // printf("\n");

	// end SRBM-Ctrl call here ==================================================================================
    
	VectorXd tau_leg_r = tau.segment<5>(0); 
	VectorXd tau_leg_l = tau.segment<5>(5);
	// Set leg joint torques
    
    d->ctrl[hip_motor1_l_idx]  = tau_leg_l(0);
    d->ctrl[hip_motor2_l_idx]  = tau_leg_l(1);
    d->ctrl[hip_motor3_l_idx]  = tau_leg_l(2);
    d->ctrl[knee_motor_l_idx]  = tau_leg_l(3);
    d->ctrl[ankle_motor_l_idx] = tau_leg_l(4);
    d->ctrl[hip_motor1_r_idx]  = tau_leg_r(0);
    d->ctrl[hip_motor2_r_idx]  = tau_leg_r(1);
    d->ctrl[hip_motor3_r_idx]  = tau_leg_r(2);
    d->ctrl[knee_motor_r_idx]  = tau_leg_r(3);
    d->ctrl[ankle_motor_r_idx] = tau_leg_r(4);

    d->ctrl[hip_motor1_l_idx]  += tello->sim_joint_torques(0);
    d->ctrl[hip_motor2_l_idx]  += tello->sim_joint_torques(1);
    d->ctrl[hip_motor3_l_idx]  += tello->sim_joint_torques(2);
    d->ctrl[knee_motor_l_idx]  += tello->sim_joint_torques(3);
    d->ctrl[ankle_motor_l_idx] += tello->sim_joint_torques(4);
    d->ctrl[hip_motor1_r_idx]  += tello->sim_joint_torques(5);
    d->ctrl[hip_motor2_r_idx]  += tello->sim_joint_torques(6);
    d->ctrl[hip_motor3_r_idx]  += tello->sim_joint_torques(7);
    d->ctrl[knee_motor_r_idx]  += tello->sim_joint_torques(8);
    d->ctrl[ankle_motor_r_idx] += tello->sim_joint_torques(9);

    FSM_prev = FSM;

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

	// BEGIN SETUP CODE FOR MUJOCO ======================================================================

	// Load Tello parameters
    dash_init::SRB_params_tello(srb_params);

	// Human initialization
    dash_init::Human_Init(human_params, human_dyn_data);

    // Robot Initialization
    dash_init::SRB_Init(x0, q0, qd0, lfv0, lfdv0, u0, srb_params, human_params);
	x = x0;
	q = q0;
	qd = qd0;
	lfv = lfv0;
	lfdv = lfdv0;
	u = u0;

    std::string recording_file_name;
    double sim_time;
    // printf("Choose the DoF to test:\n\n");
    // printf("x: lean\n");
    // printf("y: side2side\n");
    // printf("z: squat\n");
    // printf("r: roll\n");
    // printf("p: pitch\n");
    // printf("w: yaw\n");
    // printf("b: balance\n");
    // //cin.get();
    // char DoF;
    // cin.get(DoF);
    // dash_planner::SRB_6DoF_Test(recording_file_name,sim_time,srb_params,lfv0,DoF,1);
    printf("Walking Selected\n\n");
    // Option 2: Walking using LIP angular momentum regulation about contact point
    // user input (walking speed and step frequency)
    double des_walking_speed = 0.0;
    double des_walking_step_period = 0.2;
    // end user input
    recording_file_name = "walking";
    srb_params.planner_type = 1; 
    srb_params.T = des_walking_step_period;
    VectorXd t_traj, v_traj;
    double t_beg_stepping_time, t_end_stepping_time;
    dash_planner::SRB_LIP_vel_traj(des_walking_speed,t_traj,v_traj,t_beg_stepping_time,t_end_stepping_time);
    srb_params.vx_des_t = t_traj;
    srb_params.vx_des_vx = v_traj;
    srb_params.t_beg_stepping = t_beg_stepping_time;
    srb_params.t_end_stepping = t_end_stepping_time;
    sim_time = srb_params.vx_des_t(srb_params.vx_des_t.size()-1);

	// Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

	for (int i = 0; i < 4; i++) {
		Jv_mat[i] = MatrixXd::Zero(3, 5);
	}

	// activate software
    mj_activate("./lib/Mujoco/mjkey.txt");

	m = mj_loadXML("../../../lib/Mujoco/model/tello/tello.xml", NULL, error, 1000);
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
    GLFWwindow* window = glfwCreateWindow(1080, 1080, "Tello Mujoco Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    cam.elevation = -15;
    cam.distance = 1.5;
    cam.azimuth = 135;
    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = -0.3;
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

	// END SETUP CODE FOR MUJOCO ========================================================================
    // mj_step(m, d);
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

		mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0){
           mj_step(m, d);
        }   
        

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

		// END LOOP CODE FOR MUJOCO =====================================================================
		handle_end_of_periodic_task(next,period*1000);
	}

	//free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();


    return NULL;
}