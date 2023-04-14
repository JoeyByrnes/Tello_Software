#include "mujoco_comms.h"

extern RoboDesignLab::DynamicRobot* tello;

char error[1000];

bool pause_sim = true;
bool isSwingToStanceLeft = true;
bool isSwingToStanceRight = true;
int transitionTimeStepLeft = 0;
int transitionTimeStepRight = 0;
double stepping_in_progress = true;

double transitionStartLeft = -10;
double transitionStartRight = -10;

MatrixXd right_leg_last(3,5);
MatrixXd left_leg_last(3,5);

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

// Define a function to parse the JSON file and write to an srb_params struct
void parse_json_to_srb_params(const std::string& json_file_path, SRB_Params& params) {
  // Open the JSON file
  std::ifstream json_file(json_file_path);
  if (!json_file.is_open()) {
    std::cerr << "Error: could not open JSON file\n";
    return;
  }

  // Parse the JSON file
  nlohmann::json json_data;
  try {
    json_file >> json_data;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }

  // Extract the values and write to the srb_params struct
  try {
    params.Kp_xR = json_data["srb_params"]["Kp_xR"].get<double>();
    params.Kd_xR = json_data["srb_params"]["Kd_xR"].get<double>();
    params.Kp_yR = json_data["srb_params"]["Kp_yR"].get<double>();
    params.Kd_yR = json_data["srb_params"]["Kd_yR"].get<double>();
    params.Kp_zR = json_data["srb_params"]["Kp_zR"].get<double>();
    params.Kd_zR = json_data["srb_params"]["Kd_zR"].get<double>();
    params.Kp_phiR = json_data["srb_params"]["Kp_phiR"].get<double>();
    params.Kd_phiR = json_data["srb_params"]["Kd_phiR"].get<double>();
    params.Kp_thetaR = json_data["srb_params"]["Kp_thetaR"].get<double>();
    params.Kd_thetaR = json_data["srb_params"]["Kd_thetaR"].get<double>();
    params.Kp_psiR = json_data["srb_params"]["Kp_psiR"].get<double>();
    params.Kd_psiR = json_data["srb_params"]["Kd_psiR"].get<double>();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }
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
    if (act == GLFW_PRESS && key == GLFW_KEY_ENTER)
    {
        pause_sim = !pause_sim;
    }
}

void window_close_callback(GLFWwindow* window)
{
    glfwSetWindowShouldClose(window, GLFW_TRUE);
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

// Function to convert foot position in world frame to hip frame
Vector3d worldToHip(Vector3d foot_pos_world, Vector3d hip_pos_world, Vector3d hip_orient_world)
{
    // Calculate transformation matrix from world frame to torso frame
    double tx = hip_pos_world(0);
    double ty = hip_pos_world(1);
    double tz = hip_pos_world(2);
    double rx = hip_orient_world(0);
    double ry = hip_orient_world(1);
    double rz = hip_orient_world(2);

    Matrix4d T_world_to_hip;
    T_world_to_hip << cos(ry)*cos(rz), -cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz), sin(rx)*sin(rz) + cos(rx)*sin(ry)*cos(rz), tx,
                        cos(ry)*sin(rz), cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz), -sin(rx)*cos(rz) + cos(rx)*sin(ry)*sin(rz), ty,
                        -sin(ry), sin(rx)*cos(ry), cos(rx)*cos(ry), tz,
                        0, 0, 0, 1;

    // Convert foot position to hip frame
    Vector4d foot_pos_world_homogeneous;
    foot_pos_world_homogeneous << foot_pos_world, 1.0;
    Vector4d foot_pos_hip_homogeneous = T_world_to_hip.inverse() * foot_pos_world_homogeneous;
    Vector3d foot_pos_hip = foot_pos_hip_homogeneous.head(3);

    return foot_pos_hip;
}

Vector3d hipToWorld(Vector3d vector_hip, Vector3d hip_pos_world, Vector3d hip_orient_world)
{
    // Calculate transformation matrix from world frame to hip frame
    double tx = hip_pos_world(0);
    double ty = hip_pos_world(1);
    double tz = hip_pos_world(2);
    double rx = hip_orient_world(0);
    double ry = hip_orient_world(1);
    double rz = hip_orient_world(2);

    Matrix4d T_world_to_hip;
    T_world_to_hip << cos(ry)*cos(rz), -cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz), sin(rx)*sin(rz) + cos(rx)*sin(ry)*cos(rz), tx,
                     cos(ry)*sin(rz), cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz), -sin(rx)*cos(rz) + cos(rx)*sin(ry)*sin(rz), ty,
                     -sin(ry), sin(rx)*cos(ry), cos(rx)*cos(ry), tz,
                     0, 0, 0, 1;

    // Calculate transformation matrix from hip frame to world frame
    Matrix4d T_hip_to_world = T_world_to_hip.inverse();

    // Convert vector from hip frame to world frame
    Vector4d vector_hip_homogeneous;
    vector_hip_homogeneous << vector_hip, 1.0;
    Vector4d vector_world_homogeneous = T_hip_to_world * vector_hip_homogeneous;
    Vector3d vector_world = vector_world_homogeneous.head(3);

    return vector_world;
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

void contactforce(const mjModel* m, mjData* d)
{
    mjMARKSTACK
    mjtNum* f0_contact_pos = mj_stackAlloc(d, 3);//contact position in unkown frame. check
    mjtNum* f0_contact_frame = mj_stackAlloc(d, 9);
    mjtNum* f0_contact_force = mj_stackAlloc(d, 6);//contact force in contact frame
    for (int i = 0; i < d->ncon; i++)
    {
        mjContact* cur_contact = &((d->contact)[i]);
        std::string geom1 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom1);
        std::string geom2 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2);
        //std::cout << "contact point # " << i + 1 << std::endl;
        std::cout << mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2) << "  :"; // normal
        if (geom1.compare("floor") == 0 || geom2.compare("floor") == 0)
        {
            //get robot's geom id
            int bodyid;
            if(geom1.compare("floor") == 0){ //return 0 if same string
                bodyid = cur_contact->geom1;
            }else{
                bodyid = cur_contact->geom2;
            }
            //std::cout << "Collision Point = " << mj_id2name(m, mjOBJ_GEOM, bodyid) << '\n';
            // calculate position/force/normal of contact points.
            mju_copy(f0_contact_pos, cur_contact->pos, 3);
            mju_copy(f0_contact_frame, cur_contact->frame, 9); //contact frame in world frame
            mj_contactForce(m, d, i, f0_contact_force); // torque in contact frame, stored in transposed form
            //Print contact frame
            // std::cout << "Contact Frame = " << '\n';
            // mju_printMat(f0_contact_frame, 3, 3);
            //Print contact force
            mjtNum fw_contact_force[9];
            // std::cout << "Contact Force in Contact Frame = " << '\n';
            mju_transpose(fw_contact_force, f0_contact_force, 3, 3); //
            // mju_printMat(fw_contact_force, 3, 3);
            //std::cout << "Contact Force in World Frame = " << '\n';
            mjtNum f_world[9];
            mju_mulMatMat(f_world, f0_contact_frame, fw_contact_force, 3, 3, 3);
            // mju_printMat(f_world, 3, 3);
            std::cout << f_world[0] << ", " << f_world[3] << ", " << f_world[6] << endl;

        } // if one geom is object
    } // for i = 1:ncon
    mjFREESTACK
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

    mjtNum left_foot_toe[3];
    mjtNum left_foot_heel[3];
    mjtNum right_foot_toe[3];
    mjtNum right_foot_heel[3];
    const char* lft = "left_foot_toe";
    const char* lfh = "left_foot_heel";
    const char* rft = "right_foot_toe";
    const char* rfh = "right_foot_heel";
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

    // cout << "---------------------------------------------------------------------------------------" << endl;
    // contactforce(m,d); 

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

    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d torso_orientation_world(phiR, thetaR, psiR);

    Vector3d right_front_vel1 = hipToWorld(lfdv_hip.row(0), hip_right_pos_world, torso_orientation_world);
    Vector3d right_back_vel1  = hipToWorld(lfdv_hip.row(1), hip_right_pos_world, torso_orientation_world);
    Vector3d left_front_vel1  = hipToWorld(lfdv_hip.row(2), hip_left_pos_world, torso_orientation_world);
    Vector3d left_back_vel1   = hipToWorld(lfdv_hip.row(3), hip_left_pos_world, torso_orientation_world);

    lfdv.row(0) = right_front_vel1;
    lfdv.row(1) = right_back_vel1;
    lfdv.row(2) = left_front_vel1;
    lfdv.row(3) = left_back_vel1;

	// call SRBM-Ctrl here ======================================================================================

    // SRB FK
    MatrixXd torso_vertices(3,8);
    MatrixXd right_leg(3,5);
    MatrixXd left_leg(3,5);
    dash_kin::SRB_FK(torso_vertices, right_leg, left_leg, lfv, srb_params, x, q);    
    right_leg_last = right_leg;
    left_leg_last = left_leg;

    // cout << "lfv:" << endl;
    // cout << lfv << endl;
    // cout << "mujoco lfv:" << endl;
    // cout << mujoco_lfv << endl;
    // cout << "=========================" << endl;

	// SRB kinematics
	dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);
    
	// SRB trajectory planner
	MatrixXd lfv_comm(4,3);
	MatrixXd lfdv_comm(4,3);
	dash_planner::SRB_Traj_Planner(srb_params, human_dyn_data, traj_planner_dyn_data,
								   human_params, FSM, FSM_prev, t, x, lfv, lfdv, u, 
								   tau_ext, SRB_state_ref, SRB_wrench_ref, lfv_comm, lfdv_comm);

    // cout << "SRB_State_Ref:" << endl;
    // cout << SRB_state_ref.col(0).transpose() << endl;
    // cout << "State:" << endl;
    // cout << x.segment<3>(0).transpose() << x.segment<3>(18).transpose() << endl;
    // cout << "===============================" << endl;

    VectorXd state(6);
    state << x.segment<3>(0), x.segment<3>(18);

    VectorXd state_vel(6);
    state_vel << x.segment<3>(3), dEA_curr;

    dash_utils::setOutputFolder("/home/joey/Desktop/tello_outputs/latest/");
    dash_utils::writeVectorToCsv(SRB_state_ref.col(0),"SRB_state_Ref.csv");
    dash_utils::writeVectorToCsv(state,"state.csv");

    dash_utils::writeVectorToCsv(SRB_state_ref.col(1),"SRB_state_Ref_vel.csv");
    dash_utils::writeVectorToCsv(state_vel,"state_vel.csv");

    // Transform lfv an lfdv to hip frames here:

    Vector3d right_front_world = lfv_comm.row(0).transpose();
    Vector3d right_back_world  = lfv_comm.row(1).transpose();
    Vector3d left_front_world  = lfv_comm.row(2).transpose();
    Vector3d left_back_world   = lfv_comm.row(3).transpose();
    Vector3d right_front_torso = worldToHip(right_front_world, hip_right_pos_world, torso_orientation_world);
    Vector3d right_back_torso  = worldToHip(right_back_world, hip_right_pos_world, torso_orientation_world);
    Vector3d left_front_torso  = worldToHip(left_front_world, hip_left_pos_world, torso_orientation_world);
    Vector3d left_back_torso   = worldToHip(left_back_world, hip_left_pos_world, torso_orientation_world);

    Vector3d right_front_world_vel = lfdv_comm.row(0).transpose();
    Vector3d right_back_world_vel  = lfdv_comm.row(1).transpose();
    Vector3d left_front_world_vel  = lfdv_comm.row(2).transpose();
    Vector3d left_back_world_vel   = lfdv_comm.row(3).transpose();
    Vector3d right_front_torso_vel = worldToHip(right_front_world_vel, hip_right_pos_world, torso_orientation_world);
    Vector3d right_back_torso_vel  = worldToHip(right_back_world_vel, hip_right_pos_world, torso_orientation_world);
    Vector3d left_front_torso_vel  = worldToHip(left_front_world_vel, hip_left_pos_world, torso_orientation_world);
    Vector3d left_back_torso_vel   = worldToHip(left_back_world_vel, hip_left_pos_world, torso_orientation_world);
    
    Vector3d right_front = right_front_torso;
    Vector3d right_back = right_back_torso;
    Vector3d left_front = left_front_torso;
    Vector3d left_back = left_back_torso;

    Vector3d right_front_vel = right_front_torso_vel;
    Vector3d right_back_vel = right_back_torso_vel;
    Vector3d left_front_vel = left_front_torso_vel;
    Vector3d left_back_vel = left_back_torso_vel;

    // BEGIN TASK PD CODE ======================================+++++++++++++++++

    double joint_kp = 1000;
	double joint_kd = 20;
	VectorXd kp_vec_joint = VectorXd::Ones(10)*(joint_kp);
	VectorXd kd_vec_joint = VectorXd::Ones(10)*joint_kd;

    kp_vec_joint(4) = 500;
    kp_vec_joint(9) = 500;
    kd_vec_joint(4) = 5;
    kd_vec_joint(9) = 5;

	MatrixXd kp_mat_joint = kp_vec_joint.asDiagonal();
	MatrixXd kd_mat_joint = kd_vec_joint.asDiagonal();

	Vector3d target_front_left = left_front;
	Vector3d target_back_left = left_back;
	Vector3d target_front_right = right_front;
	Vector3d target_back_right = right_back;

    Vector3d target_front_left_vel = left_front_vel;
	Vector3d target_back_left_vel = left_back_vel;
	Vector3d target_front_right_vel = right_front_vel;
	Vector3d target_back_right_vel = right_back_vel;

    VectorXd vel_desired(12);
    vel_desired  << target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel;

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
	task_pd_config.motor_kp = VectorXd::Zero(10);
	task_pd_config.motor_kd = VectorXd::Zero(10);
	
	VectorXd swing_leg_torques = tello->taskPD2(task_pd_config);

    joint_kp = 0;
	joint_kd = 0;
	kp_vec_joint = VectorXd::Ones(10)*(joint_kp);
	kd_vec_joint = VectorXd::Ones(10)*joint_kd;
    kp_vec_joint(0) = 300;
    kd_vec_joint(0) = 200;
    kp_vec_joint(5) = 300;
    kd_vec_joint(5) = 200;

    kp_vec_joint(1) = 500;
    kd_vec_joint(1) = 0;

    kp_vec_joint(6) = 500;
    kd_vec_joint(6) = 0;

    kp_mat_joint = kp_vec_joint.asDiagonal();
	kd_mat_joint = kd_vec_joint.asDiagonal();
    task_pd_config.joint_kp = kp_mat_joint;
	task_pd_config.joint_kd = kd_mat_joint;

	// kp_vec_task = VectorXd::Ones(12)*task_kp;
	// kd_vec_task = VectorXd::Ones(12)*task_kd;
    // kd_vec_task(2) = 50;
    // kd_vec_task(5) = 50;
    // kd_vec_task(8) = 50;
    // kd_vec_task(11) = 50;
	// kp_mat_task = kp_vec_task.asDiagonal();
	// kd_mat_task = kd_vec_task.asDiagonal();

    // task_pd_config.task_kp = kp_mat_task;
	// task_pd_config.task_kd = kd_mat_task;

    VectorXd posture_ctrl_torques = tello->taskPD2(task_pd_config);

    // END TASK PD CODE ======================================+++++++++++++++++

	// SRB controller
	dash_ctrl::SRB_Balance_Controller(u, tau, srb_params, FSM, x, lfv, qd, Jv_mat, u, SRB_wrench_ref);

    // cout << endl << "QP right_foot_toe   :  " << u.segment<3>(0).transpose() << endl;
    // cout <<         "QP right_foot_heel  :  " << u.segment<3>(3).transpose() << endl;
    // cout <<         "QP left_foot_toe    :  " << u.segment<3>(6).transpose() << endl;
    // cout <<         "QP left_foot_heel   :  " << u.segment<3>(9).transpose() << endl;
    // cout <<         "---------------------------------------------------------------------------------------" << endl;
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

    VectorXd srbm_ctrl_torques(10);
    srbm_ctrl_torques << tau_leg_l, tau_leg_r;

    if(FSM == 1 && FSM_prev == 0)
    {
        // From double to single support left
        isSwingToStanceLeft = false;
        transitionStartLeft = d->time;
    }
    if(FSM == -1 && FSM_prev == 0)
    {
        // From double to single support right
        isSwingToStanceRight = false;
        transitionStartRight = d->time;
    }
    if(FSM == 0 && FSM_prev == -1)
    {
        // From single support right to double
        isSwingToStanceRight = true;
        transitionStartRight = d->time;
    }
    if(FSM == 0 && FSM_prev == 1)
    {
        // From single support left to double
        isSwingToStanceLeft = true;
        transitionStartLeft = d->time;
    }


    VectorXd torques_left  = tello->switchControllerJoint(tau_leg_l, swing_leg_torques.head(5),
                                                          0.01,isSwingToStanceRight, d->time-transitionStartRight, 0);
    VectorXd torques_right = tello->switchControllerJoint(tau_leg_r, swing_leg_torques.tail(5),
                                                          0.01,isSwingToStanceLeft,d->time-transitionStartLeft, 1);

    if(d->time > srb_params.t_end_stepping+0.01 && stepping_in_progress)
    {
        // stepping has finished, switch to balancing
        stepping_in_progress = false;
        pause_sim = true;
    }

    d->ctrl[hip_motor1_l_idx]  = torques_left(0);
    d->ctrl[hip_motor2_l_idx]  = torques_left(1);
    d->ctrl[hip_motor3_l_idx]  = torques_left(2);
    d->ctrl[knee_motor_l_idx]  = torques_left(3);
    d->ctrl[ankle_motor_l_idx] = torques_left(4);
    d->ctrl[hip_motor1_r_idx]  = torques_right(0);
    d->ctrl[hip_motor2_r_idx]  = torques_right(1);
    d->ctrl[hip_motor3_r_idx]  = torques_right(2);
    d->ctrl[knee_motor_r_idx]  = torques_right(3);
    d->ctrl[ankle_motor_r_idx] = torques_right(4);

    if(FSM == -1 || FSM ==0 || srb_params.planner_type == 0) 
    {
        d->ctrl[hip_motor1_r_idx]  += posture_ctrl_torques(5);
        d->ctrl[hip_motor2_r_idx]  += posture_ctrl_torques(6);
        d->ctrl[hip_motor3_r_idx]  += posture_ctrl_torques(7);
        d->ctrl[knee_motor_r_idx]  += posture_ctrl_torques(8);
        d->ctrl[ankle_motor_r_idx] += posture_ctrl_torques(9);
    }
    if(FSM == 1 || FSM == 0 || srb_params.planner_type == 0)
    {
        d->ctrl[hip_motor1_l_idx]  += posture_ctrl_torques(0);
        d->ctrl[hip_motor2_l_idx]  += posture_ctrl_torques(1);
        d->ctrl[hip_motor3_l_idx]  += posture_ctrl_torques(2);
        d->ctrl[knee_motor_l_idx]  += posture_ctrl_torques(3);
        d->ctrl[ankle_motor_l_idx] += posture_ctrl_torques(4);
    } 

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
        double des_walking_speed = 0.1;
        double des_walking_step_period = 0.2;
        // end user input
        recording_file_name = "Walking";
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
    }

    parse_json_to_srb_params("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/srb_pd_config.json",srb_params);

    srb_params.m = 22;
    srb_params.zcl = 0.03; // swing-leg max height in m


	// Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

	for (int i = 0; i < 4; i++) {
		Jv_mat[i] = MatrixXd::Zero(3, 5);
	}

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
    GLFWwindow* window = glfwCreateWindow(1920, 1080, "Tello Mujoco Sim", NULL, NULL);
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