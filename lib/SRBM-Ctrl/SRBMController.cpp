#include "SRBMController.h"

// Constructor is specific to Tello Robot
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
}

// all inputs must be in the world frame
VectorXd SRBMController::update(Vector3d body_position, Vector3d body_linear_velocity,
                                 Vector3d body_orientation, Vector3d body_angular_velocity,
                                 MatrixXd q, MatrixXd qd, double t) 
{
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


    return update(x, q, qd, t);
}

VectorXd SRBMController::update(VectorXd srb_state, MatrixXd q, MatrixXd qd, double t) 
{
    
    // SRB FK
    MatrixXd torso_vertices(3,8);
    MatrixXd right_leg(3,5);
    MatrixXd left_leg(3,5);
    dash_kin::SRB_FK(torso_vertices, right_leg, left_leg, lfv, srb_params, x, q);
    right_leg_last = right_leg;
    left_leg_last = left_leg;

    // SRB kinematics
	dash_kin::SRB_Kin(q, qd, Jv_mat, srb_params, x, lfv, lfdv);
    
	// SRB trajectory planner
	dash_planner::SRB_Traj_Planner(srb_params, human_dyn_data, traj_planner_dyn_data,
								   human_params, FSM, FSM_prev, t, x, lfv, lfdv, u, 
								   tau_ext, SRB_state_ref, SRB_wrench_ref, lfv_comm, lfdv_comm);

    // SRB controller
	dash_ctrl::SRB_Balance_Controller(u, tau, srb_params, FSM, x, lfv, qd, Jv_mat, u, SRB_wrench_ref);

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

    FSM_prev = FSM;
    // Return joint torques
    return tau;
}

MatrixXd SRBMController::get_lfv_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(0),x(1),x(2));
    MatrixXd lfv_hip(4,3);

    lfv_hip.row(0) = dash_utils::worldToHip(lfv.row(0).transpose(), hip_right_pos_world, hip_orientation_world);
    lfv_hip.row(1) = dash_utils::worldToHip(lfv.row(1).transpose(), hip_right_pos_world, hip_orientation_world);
    lfv_hip.row(2) = dash_utils::worldToHip(lfv.row(2).transpose(), hip_left_pos_world, hip_orientation_world);
    lfv_hip.row(3) = dash_utils::worldToHip(lfv.row(3).transpose(), hip_left_pos_world, hip_orientation_world);

    return lfv_hip;
}

MatrixXd SRBMController::get_lfdv_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(0),x(1),x(2));
    MatrixXd lfdv_hip(4,3);

    lfdv_hip.row(0) = dash_utils::worldToHip(lfdv.row(0).transpose(), hip_right_pos_world, hip_orientation_world);
    lfdv_hip.row(1) = dash_utils::worldToHip(lfdv.row(1).transpose(), hip_right_pos_world, hip_orientation_world);
    lfdv_hip.row(2) = dash_utils::worldToHip(lfdv.row(2).transpose(), hip_left_pos_world, hip_orientation_world);
    lfdv_hip.row(3) = dash_utils::worldToHip(lfdv.row(3).transpose(), hip_left_pos_world, hip_orientation_world);

    return lfdv_hip;
}

MatrixXd SRBMController::get_lfv_comm_hip()
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(0),x(1),x(2));
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
    Vector3d hip_orientation_world(x(0),x(1),x(2));
    MatrixXd lfdv_comm_hip(4,3);

    lfdv_comm_hip.row(0) = dash_utils::worldToHip(lfdv_comm.row(0).transpose(), hip_right_pos_world, hip_orientation_world);
    lfdv_comm_hip.row(1) = dash_utils::worldToHip(lfdv_comm.row(1).transpose(), hip_right_pos_world, hip_orientation_world);
    lfdv_comm_hip.row(2) = dash_utils::worldToHip(lfdv_comm.row(2).transpose(), hip_left_pos_world, hip_orientation_world);
    lfdv_comm_hip.row(3) = dash_utils::worldToHip(lfdv_comm.row(3).transpose(), hip_left_pos_world, hip_orientation_world);

    return lfdv_comm_hip;
}

void SRBMController::set_lfdv_hip(const MatrixXd& lfdv_hip)
{
    Vector3d hip_right_pos_world = right_leg_last.col(0);
    Vector3d hip_left_pos_world = left_leg_last.col(0);
    Vector3d hip_orientation_world(x(0),x(1),x(2));

    lfdv.row(0) = dash_utils::hipToWorld(lfdv_hip.row(0), hip_right_pos_world, hip_orientation_world);
    lfdv.row(1) = dash_utils::hipToWorld(lfdv_hip.row(1), hip_right_pos_world, hip_orientation_world);
    lfdv.row(2) = dash_utils::hipToWorld(lfdv_hip.row(2), hip_left_pos_world, hip_orientation_world);
    lfdv.row(3) = dash_utils::hipToWorld(lfdv_hip.row(3), hip_left_pos_world, hip_orientation_world);

}

