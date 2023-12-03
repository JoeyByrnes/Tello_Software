#include "tello_locomotion.h"


extern bool curve_fitting_complete;
extern bool data_ready_for_curve_fitting;
extern bool ready_for_new_curve_fit_data;
extern bool use_adaptive_step_time;
extern double dtime;
extern pthread_mutex_t tello_ctrl_mutex;

// // void TELLO_locomotion_ctrl(const mjModel* m, mjData* d)
// void TELLO_locomotion_ctrl(RoboDesignLab::DynamicRobot* robot, ctrlData cd)
// {
//     RoboDesignLab::DynamicRobot* tello = robot; // for name consistency

//     tello->_GRFs.right_front = cd.grf_rf;
//     tello->_GRFs.right_back = cd.grf_rb;
//     tello->_GRFs.left_front = cd.grf_lf;
//     tello->_GRFs.left_back = cd.grf_lb;

//     double t_end_stepping;
//     tello->controller->set_time(cd.t);

//     // Vector3d acc_no_g = subtractG(Vector3d(psiR,thetaR,phiR),Vector3d(acceleration[0],acceleration[1],acceleration[2]));
//     Vector3d imu_acc = Vector3d(cd.acceleration[0],cd.acceleration[1],cd.acceleration[2]);
//     Vector3d imu_gyro = Vector3d(cd.angular_velocity[0],cd.angular_velocity[1],cd.angular_velocity[2]);

// 	// Torso state vectors
//     VectorXd SRB_q(6);
//     VectorXd SRB_qd(6);
//     SRB_q << cd.xR, cd.yR, cd.zR, cd.phiR, cd.thetaR, cd.psiR;
//     SRB_qd << cd.xdR, cd.ydR, cd.zdR, cd.phidR, cd.thetadR, cd.psidR;
//     //pthread_mutex_lock(&plotting_mutex);
//     // CoM vector
//     pc_curr = SRB_q.head(3);
//     dpc_curr = SRB_qd.head(3);
//     MatrixXd pcom_row(1, 3);
//     pcom_row = pc_curr.transpose();
//     MatrixXd pcom_mat(4, 3);
//     pcom_mat = pcom_row.replicate(4, 1);     

//     // Leg joints and parameters vectors
//     VectorXd qLeg_l(5);
//     qLeg_l << cd.q1l, cd.q2l, cd.q3l, cd.q4l, cd.q5l;
//     VectorXd qLeg_r(5);
//     qLeg_r << cd.q1r, cd.q2r, cd.q3r, cd.q4r, cd.q5r;

//     VectorXd qdLeg_l(5);
//     qdLeg_l << cd.qd1l, cd.qd2l, cd.qd3l, cd.qd4l, cd.qd5l;
//     VectorXd qdLeg_r(5);
//     qdLeg_r << cd.qd1r, cd.qd2r, cd.qd3r, cd.qd4r, cd.qd5r;

//     // Compute rotation matrix from world to body frame
//     Matrix3d Rwb;
//     Rwb = AngleAxisd(cd.phiR, Vector3d::UnitX())
//         * AngleAxisd(cd.thetaR, Vector3d::UnitY())
//         * AngleAxisd(cd.psiR, Vector3d::UnitZ());
//     rotation_mat = Rwb;

//     // add gravity to IMU:
//     VectorXd imu_acc_world = Rwb*imu_acc;
//     imu_acc_world += Vector3d(0,0,9.81);

//     tello->controller->set_ddpc_world(imu_acc_world);

//     imu_acc = Rwb.transpose()*imu_acc_world;

//     tello->_acc = imu_acc;
//     tello->_gyro = imu_gyro;

// 	// Fill SRBM-Ctrl variables with Mujoco Variables here:
//     MatrixXd q(2,5);
//     MatrixXd qd(2,5);
// 	q.row(0) = qLeg_r;
// 	q.row(1) = qLeg_l;
//     qd.row(0) = qdLeg_r;
// 	qd.row(1) = qdLeg_l;
// 	Eigen::Map<Eigen::VectorXd> R_curr(Rwb.data(), Rwb.size());
// 	EA_curr = Vector3d(cd.phiR,cd.thetaR,cd.psiR);
//     dEA_curr = Vector3d(cd.phidR,cd.thetadR,cd.psidR);
//     VectorXd wb_curr = dash_utils::calc_wb(dEA_curr,EA_curr);

//     VectorXd qVec(10), qdVec(10);
//     qVec << q.row(1).transpose(), q.row(0).transpose();
//     qdVec << qd.row(1).transpose(), qd.row(0).transpose();

//     tello->sim_joint_pos << qVec;
//     tello->sim_joint_vel << qdVec;

//     VectorXd task_velocities = tello->joint_vel_to_task_vel(qdVec,tello->getJointPositions());

//     MatrixXd lfdv_hip(4,3);
//     lfdv_hip.row(2) = task_velocities.segment<3>(0);
//     lfdv_hip.row(3) = task_velocities.segment<3>(3);
//     lfdv_hip.row(0) = task_velocities.segment<3>(6);
//     lfdv_hip.row(1) = task_velocities.segment<3>(9);
//     if(simulation_mode == 1)
//     {
//         tello->controller->set_lfdv_hip(lfdv_hip);
//     }
    
//     VectorXd tau = tello->controller->update(pc_curr, dpc_curr, EA_curr, dEA_curr,q ,qd ,cd.t);

//     t_end_stepping = tello->controller->get_SRB_params().t_end_stepping;
//     if(tello->controller->get_sim_mode() == 2)
//     {
//         // do nothing for srbm sim
//     }
//     else
//     {

//         Vector3d target_front_left = tello->controller->get_lfv_comm_hip().row(2);
//         Vector3d target_back_left = tello->controller->get_lfv_comm_hip().row(3);
//         Vector3d target_front_right = tello->controller->get_lfv_comm_hip().row(0);
//         Vector3d target_back_right = tello->controller->get_lfv_comm_hip().row(1);

//         Vector3d target_front_left_vel = tello->controller->get_lfdv_comm_hip().row(2);
//         Vector3d target_back_left_vel = tello->controller->get_lfdv_comm_hip().row(3);
//         Vector3d target_front_right_vel = tello->controller->get_lfdv_comm_hip().row(0);
//         Vector3d target_back_right_vel = tello->controller->get_lfdv_comm_hip().row(1);

//         Vector3d target_front_left_accel = tello->controller->get_lfddv_comm_hip().row(2);
//         Vector3d target_back_left_accel = tello->controller->get_lfddv_comm_hip().row(3);
//         Vector3d target_front_right_accel = tello->controller->get_lfddv_comm_hip().row(0);
//         Vector3d target_back_right_accel = tello->controller->get_lfddv_comm_hip().row(1);

//         VectorXd vel_desired(12);
//         vel_desired  << target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel;

//         VectorXd pos_desired(12);
//         pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

//         VectorXd leg_inertia(5);
//         leg_inertia <<  0.0111, // straight leg (hip + thigh + shin + foot) inertia seen by hip yaw
//                         0.0926, // straight leg (thigh + shin + foot) inertia seen by hip roll
//                         0.0913, // straight leg (thigh + shin + foot) inertia seen by hip pitch
//                         0.0174, // straight leg (shin + foot) inertia seen by knee
//                         0.0030; // foot inertia seen by ankle
//         double inertia_accel_gain = 0.0;

//         VectorXd swing_leg_torques = VectorXd::Zero(10);
//         // if(abs(tello->controller->get_FSM()) == 1)
//         // {
//             // Set up configuration struct for Task Space Controller
//             swing_pd_config.use_single_jacoian = false;
//             swing_pd_config.side = BOTH_LEGS;
//             // if(tello->controller->get_FSM() == 1)
//             //     swing_pd_config.side = RIGHT_LEG;
//             // if(tello->controller->get_FSM() == -1)
//             //     swing_pd_config.side = LEFT_LEG;

//             swing_pd_config.ignore_joint_velocity = false;
//             swing_pd_config.task_ff_force = VectorXd::Zero(12);
//             swing_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
//             swing_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
//             swing_pd_config.setTaskKp(0,0,0);
//             swing_pd_config.setTaskKd(0,0,0);
//             // swing_pd_config.setJointKa(swing_conf.hip_yaw_Ka,swing_conf.hip_roll_Ka,swing_conf.hip_pitch_Ka,swing_conf.knee_Ka,swing_conf.ankle_Ka);
//             swing_pd_config.setJointKa(leg_inertia*inertia_accel_gain);
//             swing_pd_config.setFFAccel(target_front_left_accel,target_back_left_accel,target_front_right_accel,target_back_right_accel);
//             swing_pd_config.setJointKp(kp_vec_joint_swing);
//             swing_pd_config.setJointKd(kd_vec_joint_swing);
//             swing_pd_config.motor_kp = VectorXd::Zero(10);
//             swing_pd_config.motor_kd = VectorXd::Zero(10);
//             swing_leg_torques = tello->taskPD2(swing_pd_config);
//         // }

//         posture_pd_config = swing_pd_config;
//         posture_pd_config.ignore_joint_velocity = true;
//         posture_pd_config.side = BOTH_LEGS;
//         // if(tello->controller->get_FSM() == -1)
//         //     posture_pd_config.side = RIGHT_LEG;
//         // if(tello->controller->get_FSM() == 1)
//         //     posture_pd_config.side = LEFT_LEG;
//         // Re-using
//         posture_pd_config.task_ff_force = VectorXd::Zero(12);
//         posture_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
//         posture_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
//         posture_pd_config.setTaskKp(0,0,0);
//         posture_pd_config.setTaskKd(0,0,0);
//         posture_pd_config.setTaskKa(0,0,0);
//         posture_pd_config.setJointKp(kp_vec_joint_posture);
//         posture_pd_config.setJointKd(kd_vec_joint_posture);

//         VectorXd posture_ctrl_torques = tello->taskPD2(posture_pd_config);
        
//         // END TASK PD CODE ======================================+++++++++++++++++
//         VectorXd tau_LR(10);
//         tau_LR << tau.tail(5), tau.head(5);
//         tau_LR = tau_LR + posture_ctrl_torques;

//         VectorXd torques_left  = tello->swing_stance_mux(tau_LR.head(5), swing_leg_torques.head(5),
//                                                             0.01,tello->controller->get_isSwingToStanceRight(), 
//                                                             cd.t-tello->controller->get_transitionStartRight(), 
//                                                             0);
//         VectorXd torques_right = tello->swing_stance_mux(tau_LR.tail(5), swing_leg_torques.tail(5),
//                                                             0.01,tello->controller->get_isSwingToStanceLeft(),
//                                                             cd.t-tello->controller->get_transitionStartLeft(), 
//                                                             1);
//         VectorXd tau_LR_muxed(10);
//         tau_LR_muxed << torques_left,torques_right;
//         tau_ready = false;
//         pthread_mutex_lock(&tau_share_mutex);
//         tau_shared = tau_LR_muxed;
//         pthread_mutex_unlock(&tau_share_mutex);
//         tau_ready = true;
        
//         // tau is now ready to use with either the sim or hardware

//     }
    
// }

void* curve_fitting( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while(true)
    {
        handle_start_of_periodic_task(next);
        while(!data_ready_for_curve_fitting) usleep(5);
        data_ready_for_curve_fitting = false;
        ready_for_new_curve_fit_data = false;
        // curve fitting code here:
        pthread_mutex_lock(&tello_ctrl_mutex);
        double prev_step_duration = tello->controller->get_prev_step_duration();
        double prev_step_amplitude = tello->controller->get_prev_step_amplitude();
        Traj_planner_dyn_data traj_planner_dyn_data = tello->controller->get_traj_planner_dyn_data();
        Human_dyn_data human_dyn_data = tello->controller->get_human_dyn_data();
        VectorXd xdata = tello->controller->get_xdata();
        VectorXd ydata = tello->controller->get_ydata();
        VectorXd timevec = tello->controller->get_timevec();
        VectorXd AHvec = tello->controller->get_AHvec();
        pthread_mutex_unlock(&tello_ctrl_mutex);
        ready_for_new_curve_fit_data = true;

        double AH, end_time;
        double MIN_SWING_TIME = 0.25;
        double MAX_SWING_TIME = 0.4;
        double increasing_bound_limit = std::max((((double)xdata.size()-25)/1000.0),0.001);
        double x0[] = { 0.03 , MAX_SWING_TIME };
        double lb[] = { 0.01 , MAX_SWING_TIME - increasing_bound_limit};
        double ub[] = { 0.2 , MAX_SWING_TIME };
        coder::array<double, 2U> x_data = dash_utils::eigenVectorToCoderArray(xdata);
        coder::array<double, 2U> y_data = dash_utils::eigenVectorToCoderArray(ydata);
        step_z_curve_fit(x_data, y_data, x0, lb, ub, &AH, &end_time);
        

        if(end_time < 0.05) end_time = prev_step_duration;
        timevec.tail(99) = timevec.head(99).eval();
        timevec[0] = end_time;
        double timeval = dash_utils::smoothData(timevec, 0.8);

        if(AH < 0.008) AH = prev_step_amplitude;
        AHvec.tail(99) = AHvec.head(99).eval();
        AHvec[0] = AH;
        double AHval = dash_utils::smoothData(AHvec, 1.5);

        traj_planner_dyn_data.T_step_predicted = timeval;
        traj_planner_dyn_data.AH_step_predicted = AHval;



        // traj_planner_dyn_data.T_step = timeval;
        pthread_mutex_lock(&tello_ctrl_mutex);
        tello->controller->set_timevec(timevec);
        tello->controller->set_AHvec(AHvec);
        tello->controller->set_traj_planner_curve_params(traj_planner_dyn_data);
        pthread_mutex_unlock(&tello_ctrl_mutex);
        // end curve fitting
        curve_fitting_complete = true;

        handle_end_of_periodic_task(next,period);
    }
    return 0;
}


