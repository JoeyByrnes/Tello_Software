#include "mujoco_comms.h"

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

	// Torso state vectors
    VectorXd SRB_q(6);
    VectorXd SRB_qd(6);
    SRB_q << xR, yR, zR, phiR, thetaR, psiR;
    SRB_qd << xdR, ydR, zdR, phidR, thetadR, psidR;

    // CoM vector
    VectorXd pcom = SRB_q.head(3);
    MatrixXd pcom_row(1, 3);
    pcom_row = pcom.transpose();
    MatrixXd pcom_mat(4, 3);
    pcom_mat = pcom_row.replicate(4, 1);     

    // Leg joints and parameters vectors
    VectorXd qLeg_l(5);
    qLeg_l << q1l, q2l, q3l, q4l, q5l;
    VectorXd qLeg_r(5);
    qLeg_r << q1r, q2r, q3r, q4r, q5r;
    VectorXd pLeg(4);
    //pLeg << L1, L2, L3, L4;     

    // Compute rotation matrix from world to body frame
    MatrixXd Rx = Eigen::AngleAxisd(phiR, Vector3d::UnitX()).toRotationMatrix(); 
    MatrixXd Ry = Eigen::AngleAxisd(thetaR, Vector3d::UnitX()).toRotationMatrix(); 
    MatrixXd Rz = Eigen::AngleAxisd(psiR, Vector3d::UnitX()).toRotationMatrix(); 
    MatrixXd Rwb = Rx * Ry * Rz;   

	// Fill SRBM-Ctrl variables with Mujoco Variables here:
	q.row(0) = qLeg_r;
	q.row(1) = qLeg_l;
	Eigen::Map<Eigen::VectorXd> r_curr(Rwb.data(), Rwb.size());
	VectorXd EA_curr = dash_utils::calc_EA(Rwb);

	x.segment(0, 3) = pcom;
	//x.segment(3, 3) = Vector3d::Zero();// need dpc_curr
	x.segment(6, 9) = r_curr;
	//x.segment(15, 3) = Vector3d::Zero(); // need wb_curr
	x.segment(18, 3) = EA_curr;

	// call SRBM-Ctrl here ======================================================================================
	
	// SRB kinematics
	dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);

	// SRB trajectory planner
	MatrixXd lfv_comm;
	MatrixXd lfdv_comm;
	dash_planner::SRB_Traj_Planner(srb_params, human_dyn_data, traj_planner_dyn_data,
								   human_params, FSM, FSM_prev, t, x, lfv, lfdv, u, 
								   tau_ext, SRB_state_ref, SRB_wrench_ref, lfv_comm, lfdv_comm);

	// SRB controller
	dash_ctrl::SRB_Balance_Controller(u, tau, srb_params, FSM, x, lfv, qd, Jv_mat, u, SRB_wrench_ref);

	// end SRBM-Ctrl call here ==================================================================================

	VectorXd tau_leg_r = tau.segment<5>(0); 
	VectorXd tau_leg_l = tau.segment<5>(5);
	// Set leg joint torques
    d->ctrl[hip_motor1_l_idx] = tau_leg_l(0);
    d->ctrl[hip_motor2_l_idx] = tau_leg_l(1);
    d->ctrl[hip_motor3_l_idx] = tau_leg_l(2);
    d->ctrl[knee_motor_l_idx] = tau_leg_l(3);
    d->ctrl[ankle_motor_l_idx] = tau_leg_l(4);
    d->ctrl[hip_motor1_r_idx] = tau_leg_r(0);
    d->ctrl[hip_motor2_r_idx] = tau_leg_r(1);
    d->ctrl[hip_motor3_r_idx] = tau_leg_r(2);
    d->ctrl[knee_motor_r_idx] = tau_leg_r(3);
    d->ctrl[ankle_motor_r_idx] = tau_leg_r(4);  
	
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

	// Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

	for (int i = 0; i < 4; i++) {
		Jv_mat[i] = MatrixXd::Zero(3, 5);
	}

	// activate software
    mj_activate("./lib/Mujoco/mjkey.txt");

	m = mj_loadXML("../../../lib/Mujoco/model/tello/TELLO_Simple.xml", NULL, error, 1000);
	if (!m)
    {
        printf('r',"%s\n", error);
        exit(1);
    }
	// make data
    d = mj_makeData(m);

	// set initial legs configuration
    initial_legs_configuration(d);

    // install control callback
    mjcb_control = TELLO_locomotion_ctrl;

	// init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

		// create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "TELLO like robot", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
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

	while(!glfwWindowShouldClose(window))
    {
        handle_start_of_periodic_task(next);
		// BEGIN LOOP CODE FOR MUJOCO ===================================================================

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
		handle_end_of_periodic_task(next,period);
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