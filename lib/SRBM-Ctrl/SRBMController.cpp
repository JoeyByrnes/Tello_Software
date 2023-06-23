#include "SRBMController.h"
extern Vector4d z_plotting;
extern bool first_time_planner;
extern bool first_time_running_qp;

SRBMController::SRBMController()
{
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
    lfv_dsp_start = lfv0;

    for (int i = 0; i < 4; i++) {
		Jv_mat[i] = MatrixXd::Zero(3, 5);
	}

    // Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

    last_print_time = std::chrono::high_resolution_clock::now();
}

SRBMController::SRBMController(const SRBMController& other) {
        enable_human_dyn_data = other.enable_human_dyn_data;
        srb_params = other.srb_params;
        human_params = other.human_params;
        human_dyn_data = other.human_dyn_data;
        traj_planner_dyn_data = other.traj_planner_dyn_data;
        t = other.t;
        FSM = other.FSM;
        FSM_prev = other.FSM_prev;
        net_external_wrench = other.net_external_wrench;
        x_next = other.x_next;
        _EA = other._EA;
        _dEA = other._dEA;
        _pc = other._pc;
        _dpc = other._dpc;
        _ddpc = other._ddpc;
        x = other.x;
        q = other.q;
        qd = other.qd;
        lfv = other.lfv;
        lfv_dsp_start = other.lfv_dsp_start;
        lfdv = other.lfdv;
        lfv_comm = other.lfv_comm;
        lfdv_comm = other.lfdv_comm;
        lfddv_comm = other.lfddv_comm;
        u = other.u;
        lfv0 = other.lfv0;
        lfdv0 = other.lfdv0;
        q0 = other.q0;
        qd0 = other.qd0;
        u0 = other.u0;
        x0 = other.x0;
        tau_ext = other.tau_ext;
        tau = other.tau;
        SRB_state_ref = other.SRB_state_ref;
        SRB_wrench_ref = other.SRB_wrench_ref;
        
        // Copy Jv_mat array
        for (int i = 0; i < 4; i++) {
            Jv_mat[i] = other.Jv_mat[i];
        }
        
        right_leg_last = other.right_leg_last;
        left_leg_last = other.left_leg_last;
        isSwingToStanceLeft = other.isSwingToStanceLeft;
        isSwingToStanceRight = other.isSwingToStanceRight;
        transitionStartLeft = other.transitionStartLeft;
        transitionStartRight = other.transitionStartRight;
        start_time = other.start_time;
        end_time = other.end_time;
        last_print_time = other.last_print_time;
        simulation_mode = other.simulation_mode;
        using_human_playback = other.using_human_playback;
    }

// all inputs must be in the world frame
VectorXd SRBMController::update(Vector3d body_position, Vector3d body_linear_velocity,
                                 Vector3d body_orientation, Vector3d body_angular_velocity,
                                 MatrixXd joint_pos, MatrixXd joint_vel, double t) 
{
    
        _EA = body_orientation;
        _dEA = body_angular_velocity;
        _pc = body_position;
        _dpc = body_linear_velocity;
        
        double phiR   = body_orientation(0);
        double thetaR = body_orientation(1);
        double psiR   = body_orientation(2);

        double phidR   = body_angular_velocity(0);
        double thetadR = body_angular_velocity(1);
        double psidR   = body_angular_velocity(2); 

        // Compute rotation matrix from world to body frame
        Matrix3d Rwb;
        Rwb = AngleAxisd(phiR  , Vector3d::UnitX())
            * AngleAxisd(thetaR, Vector3d::UnitY())
            * AngleAxisd(psiR  , Vector3d::UnitZ());

        Eigen::Map<Eigen::VectorXd> R_curr(Rwb.data(), Rwb.size());
        VectorXd wb_curr = dash_utils::calc_wb(body_angular_velocity,body_orientation);

        x.segment(0, 3) = body_position;
        x.segment(3, 3) = body_linear_velocity;
        x.segment(6, 9) = R_curr;
        x.segment(15, 3) = wb_curr;
        x.segment(18, 3) = body_orientation;
 
    return update(x, joint_pos, joint_vel, t);
}

VectorXd SRBMController::update(VectorXd srb_state, MatrixXd joint_pos, MatrixXd joint_vel, double t) 
{

    // SRB FK
    MatrixXd torso_vertices(3,8);
    MatrixXd right_leg(3,5);
    MatrixXd left_leg(3,5);
    q = joint_pos;
    qd = joint_vel;
    x = srb_state;
    
    dash_kin::SRB_FK(torso_vertices, right_leg, left_leg, lfv, srb_params, x, q);
    
    right_leg_last = right_leg;
    left_leg_last = left_leg;
    
    // SRB kinematics
	dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);

	// SRB trajectory planner
	dash_planner::SRB_Traj_Planner(srb_params, human_dyn_data, traj_planner_dyn_data,
								   human_params, FSM, FSM_prev, t, x, lfv, lfdv, u, 
								   tau_ext, SRB_state_ref, SRB_wrench_ref, lfv_comm, lfdv_comm, lfddv_comm);
    // SRB controller
    
	dash_ctrl::SRB_Balance_Controller(u, tau, srb_params, FSM, x, lfv, qd, Jv_mat, u, SRB_wrench_ref);
    
    // cout << "Tau: " << tau.transpose() << endl;
    // cout << "GRFs: " << u.transpose() << endl;
    
    if(FSM == 1 && FSM_prev == 0)
    {
        // From double to single support left
        isSwingToStanceLeft = false;
        transitionStartLeft = t;
    }
    if(FSM == -1 && FSM_prev == 0)
    {
        // From double to single support right
        isSwingToStanceRight = false;
        transitionStartRight = t;
    }
    if(FSM == 0 && FSM_prev == -1)
    {
        // From single support right to double
        isSwingToStanceRight = true;
        transitionStartRight = t;
    }
    if(FSM == 0 && FSM_prev == 1)
    {
        // From single support left to double
        isSwingToStanceLeft = true;
        transitionStartLeft = t;
    }
    
    if(simulation_mode == 2)
    {
        dash_dyn::SRB_Dyn(x_next, net_external_wrench, srb_params, x, lfv, u, tau_ext);
        x = x_next;
        lfv = lfv_comm;
        lfdv = lfdv_comm;
        dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);
    }
    // cout << lfv_comm  << endl;
    // cout << "===============================" << endl;
    // cout << endl;
    // cout << "FSM: " << FSM << endl;

    FSM_prev = FSM;
    // Return joint torques
    return tau;
}

VectorXd SRBMController::update_euler_integration(VectorXd srb_state, MatrixXd q, MatrixXd qd, double t) 
{

    // SRB FK
    MatrixXd torso_vertices(3,8);
    MatrixXd right_leg(3,5);
    MatrixXd left_leg(3,5);
    this->q = q;
    
    dash_kin::SRB_FK(torso_vertices, right_leg, left_leg, lfv, srb_params, x, q);
    
    right_leg_last = right_leg;
    left_leg_last = left_leg;
    
    // SRB kinematics
	dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);
    
	// SRB trajectory planner
	dash_planner::SRB_Traj_Planner(srb_params, human_dyn_data, traj_planner_dyn_data,
								   human_params, FSM, FSM_prev, t, x, lfv, lfdv, u, 
								   tau_ext, SRB_state_ref, SRB_wrench_ref, lfv_comm, lfdv_comm, lfddv_comm);
    // SRB controller
    
	dash_ctrl::SRB_Balance_Controller(u, tau, srb_params, FSM, x, lfv, qd, Jv_mat, u, SRB_wrench_ref);

    // SRB dynamics
    dash_dyn::SRB_Dyn(x_next, net_external_wrench, srb_params, x, lfv, u, tau_ext);
    
    // if(FSM == 1 && FSM_prev == 0)
    // {
    //     // From double to single support left
    //     isSwingToStanceLeft = false;
    //     transitionStartLeft = t;
    // }
    // if(FSM == -1 && FSM_prev == 0)
    // {
    //     // From double to single support right
    //     isSwingToStanceRight = false;
    //     transitionStartRight = t;
    // }
    // if(FSM == 0 && FSM_prev == -1)
    // {
    //     // From single support right to double
    //     isSwingToStanceRight = true;
    //     transitionStartRight = t;
    // }
    // if(FSM == 0 && FSM_prev == 1)
    // {
    //     // From single support left to double
    //     isSwingToStanceLeft = true;
    //     transitionStartLeft = t;
    // }

    FSM_prev = FSM;
    x = x_next;
    // Return joint torques
    return tau;
}

MatrixXd SRBMController::get_lfv_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));
    MatrixXd lfv_hip(4,3);

    lfv_hip.row(0) = dash_utils::worldToHip(lfv.row(0).transpose(), hip_right_pos_world, hip_orientation_world);
    lfv_hip.row(1) = dash_utils::worldToHip(lfv.row(1).transpose(), hip_right_pos_world, hip_orientation_world);
    lfv_hip.row(2) = dash_utils::worldToHip(lfv.row(2).transpose(), hip_left_pos_world, hip_orientation_world);
    lfv_hip.row(3) = dash_utils::worldToHip(lfv.row(3).transpose(), hip_left_pos_world, hip_orientation_world);

    return lfv_hip;
}

MatrixXd SRBMController::get_ankles_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));
    MatrixXd ankles_hip(2,3);

    ankles_hip.row(0) = dash_utils::worldToHip(right_leg_last.col(2), hip_right_pos_world, hip_orientation_world);
    ankles_hip.row(1) = dash_utils::worldToHip(left_leg_last.col(2), hip_left_pos_world, hip_orientation_world);
    
    return ankles_hip;
}

MatrixXd SRBMController::get_lfdv_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));
    MatrixXd lfdv_hip(4,3);

    Vector3d pc_curr = x.segment<3>(0);
    Vector3d dpc_curr = x.segment<3>(3);
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d wb_curr = x.segment<3>(15);

    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)
    // construct leg parameter vector
    VectorXd p_leg(4);
    p_leg << thigh_length, calf_length, foot_length, heel_length; 

    // MatrixXd r_mat = lfv.transpose() - p_hips;
    MatrixXd r_mat = lfv.transpose() - pc_curr.replicate(1, 4);
    // calculate joint velocities
    Vector3d ww_curr = R_curr*wb_curr; // angular velocity in world frame
    VectorXd qd_b(6);
    qd_b << dpc_curr, ww_curr; // floating base velocities
    MatrixXd lfd_mat = lfdv.transpose(); // matrix form of end-effector velocities
    VectorXd r_mat_right_t = r_mat.col(0); // right front end-effector position relative to hip
    VectorXd lfd_mat_right_t = lfd_mat.col(0); // right front end-effector velocity in world frame
    VectorXd lfdv_hip_r_t = dash_utils::world_to_robot_task_vel(qd_b, R_curr, r_mat_right_t, lfd_mat_right_t); // right ee vel in hip frame

    VectorXd r_mat_right_h = r_mat.col(1); // right back end-effector position relative to hip
    VectorXd lfd_mat_right_h = lfd_mat.col(1); // right back end-effector velocity in world frame
    VectorXd lfdv_hip_r_h = dash_utils::world_to_robot_task_vel(qd_b, R_curr, r_mat_right_h, lfd_mat_right_h); // right ee vel in hip frame
    
    VectorXd r_mat_left_t = r_mat.col(2); // left front end-effector position relative to hip
    VectorXd lfd_mat_left_t = lfd_mat.col(2); // left front end-effector velocity in world frame
    VectorXd lfdv_hip_l_t = dash_utils::world_to_robot_task_vel(qd_b, R_curr, r_mat_left_t, lfd_mat_left_t); // left ee vel in hip frame

    VectorXd r_mat_left_h = r_mat.col(3); // left front end-effector position relative to hip
    VectorXd lfd_mat_left_h = lfd_mat.col(3); // left front end-effector velocity in world frame
    VectorXd lfdv_hip_l_h = dash_utils::world_to_robot_task_vel(qd_b, R_curr, r_mat_left_h, lfd_mat_left_h); // left ee vel in hip frame

    lfdv_hip.row(0) = lfdv_hip_r_t.transpose().head(3);
    lfdv_hip.row(1) = lfdv_hip_r_h.transpose().head(3);
    lfdv_hip.row(2) = lfdv_hip_l_t.transpose().head(3);
    lfdv_hip.row(3) = lfdv_hip_l_h.transpose().head(3);

    return lfdv_hip;
}

MatrixXd SRBMController::get_lfv_comm_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));
    MatrixXd lfv_comm_hip(4,3);

    lfv_comm_hip.row(0) = dash_utils::worldToHip(lfv_comm.row(0).transpose(), hip_right_pos_world, hip_orientation_world);
    lfv_comm_hip.row(1) = dash_utils::worldToHip(lfv_comm.row(1).transpose(), hip_right_pos_world, hip_orientation_world);
    lfv_comm_hip.row(2) = dash_utils::worldToHip(lfv_comm.row(2).transpose(), hip_left_pos_world, hip_orientation_world);
    lfv_comm_hip.row(3) = dash_utils::worldToHip(lfv_comm.row(3).transpose(), hip_left_pos_world, hip_orientation_world);

    return lfv_comm_hip;
}

MatrixXd SRBMController::get_lfdv_comm_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));
    MatrixXd lfdv_comm_hip(4,3);

    Vector3d pc_curr = x.segment<3>(0);
    Vector3d dpc_curr = x.segment<3>(3);
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d wb_curr = x.segment<3>(15);

    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)
    // construct leg parameter vector
    VectorXd p_leg(4);
    p_leg << thigh_length, calf_length, foot_length, heel_length; 

    // MatrixXd r_mat = lfv.transpose() - p_hips;
    MatrixXd r_mat = lfv.transpose() - pc_curr.replicate(1, 4);
    // calculate joint velocities
    Vector3d ww_curr = R_curr*wb_curr; // angular velocity in world frame
    VectorXd qd_b(6);
    qd_b << dpc_curr, ww_curr; // floating base velocities
    MatrixXd lfd_mat = lfdv_comm.transpose(); // matrix form of end-effector velocities
    VectorXd r_mat_right = r_mat.col(0); // right front end-effector position relative to hip
    VectorXd lfd_mat_right = lfd_mat.col(0); // right front end-effector velocity in world frame
    VectorXd lfdv_comm_hip_r = dash_utils::world_to_robot_task_vel(qd_b, R_curr, r_mat_right, lfd_mat_right); // right ee vel in hip frame
    
    VectorXd r_mat_left = r_mat.col(2); // left front end-effector position relative to hip
    VectorXd lfd_mat_left = lfd_mat.col(2); // left front end-effector velocity in world frame
    VectorXd lfdv_comm_hip_l = dash_utils::world_to_robot_task_vel(qd_b, R_curr, r_mat_left, lfd_mat_left); // left ee vel in hip frame

    lfdv_comm_hip.row(0) = lfdv_comm_hip_r.transpose().head(3);
    lfdv_comm_hip.row(1) = lfdv_comm_hip_r.transpose().head(3);
    lfdv_comm_hip.row(2) = lfdv_comm_hip_l.transpose().head(3);
    lfdv_comm_hip.row(3) = lfdv_comm_hip_l.transpose().head(3);

    return lfdv_comm_hip;
}

void SRBMController::set_lfdv_hip(const MatrixXd& lfdv_hip)
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));

    // lfdv.row(0) = dash_utils::hipToWorld(lfdv_hip.row(0), hip_right_pos_world, hip_orientation_world);
    // lfdv.row(1) = dash_utils::hipToWorld(lfdv_hip.row(1), hip_right_pos_world, hip_orientation_world);
    // lfdv.row(2) = dash_utils::hipToWorld(lfdv_hip.row(2), hip_left_pos_world, hip_orientation_world);
    // lfdv.row(3) = dash_utils::hipToWorld(lfdv_hip.row(3), hip_left_pos_world, hip_orientation_world);

    Vector3d pc_curr = x.segment<3>(0);
    Vector3d dpc_curr = x.segment<3>(3);
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d wb_curr = x.segment<3>(15);

    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)
    // construct leg parameter vector
    VectorXd p_leg(4);
    p_leg << thigh_length, calf_length, foot_length, heel_length; 

    // MatrixXd r_mat = lfv.transpose() - p_hips;
    MatrixXd r_mat = lfv.transpose() - pc_curr.replicate(1, 4);
    // calculate joint velocities
    Vector3d ww_curr = R_curr*wb_curr; // angular velocity in world frame
    VectorXd qd_b(6);
    qd_b << dpc_curr, ww_curr; // floating base velocities
    MatrixXd lfd_mat = lfdv_hip.transpose(); // matrix form of end-effector velocities
    VectorXd r_mat_right = r_mat.col(0); // right front end-effector position relative to hip
    VectorXd lfd_mat_right = lfd_mat.col(0); // right front end-effector velocity in world frame
    VectorXd lfdv_world_r = dash_utils::robot_to_world_task_vel(qd_b, R_curr, r_mat_right, lfd_mat_right); // right ee vel in hip frame
    
    VectorXd r_mat_left = r_mat.col(2); // left front end-effector position relative to hip
    VectorXd lfd_mat_left = lfd_mat.col(2); // left front end-effector velocity in world frame
    VectorXd lfdv_world_l = dash_utils::robot_to_world_task_vel(qd_b, R_curr, r_mat_left, lfd_mat_left); // left ee vel in hip frame

    lfdv.row(0) = lfdv_world_r.transpose().head(3);
    lfdv.row(1) = lfdv_world_r.transpose().head(3);
    lfdv.row(2) = lfdv_world_l.transpose().head(3);
    lfdv.row(3) = lfdv_world_l.transpose().head(3);

}

MatrixXd SRBMController::get_lfddv_comm_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(18),x(19),x(20));
    MatrixXd lfddv_comm_hip(4,3);

    Vector3d pc_curr = x.segment<3>(0);
    Vector3d dpc_curr = x.segment<3>(3);
    VectorXd ddpc_curr = _ddpc;
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d wb_curr = x.segment<3>(15);

    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)
    // construct leg parameter vector
    VectorXd p_leg(4);
    p_leg << thigh_length, calf_length, foot_length, heel_length; 

    // MatrixXd r_mat = lfv.transpose() - p_hips;
    MatrixXd r_mat = lfv.transpose() - pc_curr.replicate(1, 4);
    // calculate joint velocities
    Vector3d ww_curr = R_curr*wb_curr; // angular velocity in world frame

    MatrixXd lfdd_mat = lfddv_comm.transpose(); // matrix form of end-effector velocities
    VectorXd r_mat_right = r_mat.col(0); // right front end-effector position relative to hip
    VectorXd lfdd_mat_right = lfdd_mat.col(0); // right front end-effector velocity in world frame
    VectorXd lfddv_comm_hip_r = dash_utils::world_to_robot_task_accel(ddpc_curr, R_curr, r_mat_right, lfdd_mat_right); // right ee vel in hip frame
    
    VectorXd r_mat_left = r_mat.col(2); // left front end-effector position relative to hip
    VectorXd lfdd_mat_left = lfdd_mat.col(2); // left front end-effector velocity in world frame
    VectorXd lfddv_comm_hip_l = dash_utils::world_to_robot_task_accel(ddpc_curr, R_curr, r_mat_left, lfdd_mat_left); // left ee vel in hip frame

    lfddv_comm_hip.row(0) = lfddv_comm_hip_r.transpose();
    lfddv_comm_hip.row(1) = lfddv_comm_hip_r.transpose();
    lfddv_comm_hip.row(2) = lfddv_comm_hip_l.transpose();
    lfddv_comm_hip.row(3) = lfddv_comm_hip_l.transpose();

    return lfddv_comm_hip;
}

void SRBMController::set_human_dyn_data_without_forces(const Human_dyn_data& new_hdd)
{
    human_dyn_data.xH = new_hdd.xH;
    human_dyn_data.dxH = new_hdd.dxH;
    human_dyn_data.pxH = new_hdd.pxH;
    human_dyn_data.yH = new_hdd.yH;
    human_dyn_data.dyH = new_hdd.dyH;
    human_dyn_data.pyH = new_hdd.pyH;
    human_dyn_data.fxH_R = new_hdd.fxH_R;
    human_dyn_data.fyH_R = new_hdd.fyH_R;
    human_dyn_data.fzH_R = new_hdd.fzH_R;
    human_dyn_data.fxH_L = new_hdd.fxH_L;
    human_dyn_data.fyH_L = new_hdd.fyH_L;
    human_dyn_data.fzH_L = new_hdd.fzH_L;
    human_dyn_data.fdxH_R = new_hdd.fdxH_R;
    human_dyn_data.fdyH_R = new_hdd.fdyH_R;
    human_dyn_data.fdzH_R = new_hdd.fdzH_R;
    human_dyn_data.fdxH_L = new_hdd.fdxH_L;
    human_dyn_data.fdyH_L = new_hdd.fdyH_L;
    human_dyn_data.fdzH_L = new_hdd.fdzH_L;
}

void SRBMController::set_hmi_forces(const Human_dyn_data& data)
{
    human_dyn_data.FxH_hmi = data.FxH_hmi;
    human_dyn_data.FyH_hmi = data.FyH_hmi;
    human_dyn_data.FxH_spring = data.FxH_spring;
}

MatrixXd SRBMController::get_foot_orientation_wrt_body(VectorXd q_leg)
{
    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)

    // Construct leg parameter vector
    Eigen::Vector4d p_leg(thigh_length, calf_length, foot_length, heel_length);

    Matrix3d R_body_to_foot = dash_kin::fcn_HTM0lf1(q_leg, p_leg).block<3,3>(0,0);

    return R_body_to_foot;
}

double SRBMController::get_CoM_z(MatrixXd lfv_hip,VectorXd gnd_contacts, Vector3d EA)
{

   double W = 0.252;
    double CoM2H_z_dist = 0.18;

    Vector3d CoM2hr(0,-W/2.0,-CoM2H_z_dist);
    Vector3d CoM2hl(0, W/2.0,-CoM2H_z_dist);

    Vector3d right_front_in_hip = lfv_hip.row(0);
    Vector3d right_back_in_hip = lfv_hip.row(1);
    Vector3d left_front_in_hip = lfv_hip.row(2);
    Vector3d left_back_in_hip = lfv_hip.row(3);

    Eigen::Vector3d right_front_in_CoM = right_front_in_hip+CoM2hr;
    Eigen::Vector3d right_back_in_CoM = right_back_in_hip+CoM2hr;
    Eigen::Vector3d left_front_in_CoM = left_front_in_hip+CoM2hl;
    Eigen::Vector3d left_back_in_CoM = left_back_in_hip+CoM2hl;

    Matrix3d Rwb;
    Rwb = AngleAxisd(EA(0), Vector3d::UnitX())
        * AngleAxisd(EA(1), Vector3d::UnitY())
        * AngleAxisd(EA(2), Vector3d::UnitZ());

    double right_front_z = (Rwb*right_front_in_CoM)(2);
    double right_back_z  = (Rwb*right_back_in_CoM)(2);
    double left_front_z  = (Rwb*left_front_in_CoM)(2);
    double left_back_z   = (Rwb*left_back_in_CoM)(2);

    // double right_front_z = (dash_utils::hipToWorld(right_front_in_CoM, Vector3d(0,0,0), EA))(2);
    // double right_back_z  = (dash_utils::hipToWorld(right_back_in_CoM, Vector3d(0,0,0), EA))(2);
    // double left_front_z  = (dash_utils::hipToWorld(left_front_in_CoM, Vector3d(0,0,0), EA))(2);
    // double left_back_z   = (dash_utils::hipToWorld(left_back_in_CoM, Vector3d(0,0,0), EA))(2);
    Vector4d z_heights(right_front_z,right_back_z,left_front_z,left_back_z);
    z_plotting = -z_heights.array()-srb_params.hLIP;
    double z_height = 0;
    if(gnd_contacts.sum() == 0) return -1;
    
    for(int i=0;i<gnd_contacts.size();i++)
    {
        if(gnd_contacts(i) == 1)
        {
            z_height += z_heights(i);
        }
    }
    z_height = z_height/gnd_contacts.sum();

    z_height = -z_height-srb_params.hLIP;

    return z_height;
}

void SRBMController::start_timer(){
    start_time = std::chrono::high_resolution_clock::now();
}

void SRBMController::end_timer(){
    end_time = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    std::chrono::nanoseconds elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);

    // Convert the elapsed time to milliseconds with a resolution of nanoseconds
    double elapsed_time_ms = static_cast<double>(elapsed_time.count()) / 1000000.0;

    std::chrono::nanoseconds elapsed_time_since_print = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - last_print_time);
    double elapsed_time_since_print_ms = static_cast<double>(elapsed_time_since_print.count()) / 1000000.0;
    
    if(elapsed_time_since_print_ms > 250){
        std::cout << "Elapsed time: " << elapsed_time_ms << " ms" << std::endl;
        last_print_time = end_time;
    }

}

void SRBMController::reset()
{
    FSM = 0;
    FSM_prev = 0;
    t = 0;
    x.setZero();
    SRB_state_ref.setZero();
    SRB_wrench_ref.setZero();
    tau_ext.setZero();
    net_external_wrench.setZero();
    tau.setZero();
    _EA.setZero();
    _dEA.setZero();
    _pc.setZero();
    _dpc.setZero();

    first_time_running_qp = true;
    first_time_planner = true;


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
    lfv_dsp_start = lfv0;

    for (int i = 0; i < 4; i++) {
		Jv_mat[i] = MatrixXd::Zero(3, 5);
	}

    // Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

    last_print_time = std::chrono::high_resolution_clock::now();
}