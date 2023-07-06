#ifndef __SRBM_CONTROLLER_H__
#define __SRBM_CONTROLLER_H__

#include <Eigen/Dense>
#include "structs.h"
#include "utilities.h"
#include "controllers.h"
#include "initialization.h"
#include "kinematics.h"
#include "planner.h"
#include "dynamics.h"

using namespace Eigen;

struct SRBMState
{
    Vector3d body_position; // X, Y, Z
    Vector3d body_linear_velocity; // dX, dY, dZ
    Vector3d body_orientation; // Roll, Pitch, Yaw
    Vector3d body_angular_velocity; // dRoll, dPitch, dYaw
};

class SRBMController {
  public:
    SRBMController();
    SRBMController(int sim_mode);
    SRBMController(const SRBMController& other);
    ~SRBMController() {}

    VectorXd update(Vector3d body_position, Vector3d body_linear_velocity,
                                 Vector3d body_orientation, Vector3d body_angular_velocity,
                                 MatrixXd q, MatrixXd qd, double t);  
    VectorXd update(VectorXd srb_state, MatrixXd q, MatrixXd qd, double t);  

    VectorXd update_euler_integration(VectorXd srb_state, MatrixXd q, MatrixXd qd, double t); 

    // Getter and setter functions for SRB_Params
    SRB_Params get_SRB_params() const { return srb_params; }
    void set_SRB_params(const SRB_Params& params) { srb_params = params; }

    // Getter and setter functions for Human_params
    Human_params get_human_params() const { return human_params; }
    void set_human_params(const Human_params& params) { human_params = params; }

    // Getter and setter functions for Human_dyn_data
    Human_dyn_data get_human_dyn_data() const { return human_dyn_data; }
    void set_human_dyn_data(const Human_dyn_data& data) { human_dyn_data = data; }
    void set_human_dyn_data_without_forces(const Human_dyn_data& data);
    void set_hmi_forces(const Human_dyn_data& data);

    // Getter and setter functions for Traj_planner_dyn_data
    Traj_planner_dyn_data get_traj_planner_dyn_data() const { return traj_planner_dyn_data; }
    void set_traj_planner_dyn_data(const Traj_planner_dyn_data& data) { traj_planner_dyn_data = data; }
    void set_traj_planner_step_data(const Traj_planner_dyn_data& data) 
    { 
      traj_planner_dyn_data.step_z_history_L = data.step_z_history_L; 
      traj_planner_dyn_data.step_z_history_R = data.step_z_history_R; 
      traj_planner_dyn_data.curr_SSP_sample_count = data.curr_SSP_sample_count;
    }
    void set_traj_planner_curve_params(const Traj_planner_dyn_data& data) 
    { 
      traj_planner_dyn_data.AH_step_predicted = data.AH_step_predicted; 
      traj_planner_dyn_data.T_step_predicted = data.T_step_predicted; 
    }
    void set_traj_planner_human_foot_z_offsets(const Traj_planner_dyn_data& data) 
    { 
      traj_planner_dyn_data.step_z_offset_L = data.step_z_offset_L; 
      traj_planner_dyn_data.step_z_offset_R = data.step_z_offset_R; 
    }


    // Getter and setter functions for t
    double get_time() const { return t; }
    void set_time(double time) { t = time; }

    // Getter function for FSM
    int get_FSM() const { return FSM; }
    void set_FSM(int fsm) { FSM=fsm; }

    // Getter function for FSM_prev
    int get_FSM_prev() const { return FSM_prev; }

    // Getter and setter functions for x
    VectorXd get_x() const { return x; }
    void set_x(const VectorXd& vec) { x = vec; }

    // Getter and setter functions for q
    MatrixXd get_q() const { return q; }
    void set_q(const MatrixXd& mat) { q = mat; }

    // Getter and setter functions for qd
    MatrixXd get_qd() const { return qd; }
    void set_qd(const MatrixXd& mat) { qd = mat; }

    // Getter functions for lfv
    MatrixXd get_lfv_world() const { return lfv; }
    MatrixXd get_lfv_hip();
    MatrixXd get_lfv_dsp_start() const { return lfv_dsp_start; }
    MatrixXd get_ankles_hip();

    // Getter and setter functions for lfdv
    MatrixXd get_lfdv_world() const { return lfdv; }
    MatrixXd get_lfdv_hip();
    void set_lfdv_world(const MatrixXd& mat) { lfdv = mat; }
    void set_lfdv_hip(const MatrixXd& mat);

    // Getter functions for lfv_comm
    MatrixXd get_lfv_comm_world() const { return lfv_comm; }
    MatrixXd get_lfv_comm_hip();

    // Getter functions for lfdv_comm
    MatrixXd get_lfdv_comm_world() const { return lfdv_comm; }
    MatrixXd get_lfdv_comm_hip();

    // Getter functions for lfddv_comm
    MatrixXd get_lfddv_comm_world() const { return lfddv_comm; }
    MatrixXd get_lfddv_comm_hip();

    // Getter function for GRFs
    VectorXd get_GRFs() const { return u; }

    // Getter and setter functions for tau_ext
    VectorXd get_tau_ext() const { return tau_ext; }
    void set_tau_ext(const VectorXd& vec) { tau_ext = vec; }

    // Getter function for tau
    VectorXd get_joint_torques() const { return tau; }

    // Getter and setter functions for SRB_state_ref
    MatrixXd get_SRB_state_ref() const { return SRB_state_ref; }
    void set_SRB_state_ref(const MatrixXd& mat) { SRB_state_ref = mat; }

    // Getter and setter functions for SRB_wrench_ref
    VectorXd get_SRB_wrench_ref() const { return SRB_wrench_ref; }
    void set_SRB_wrench_ref(const VectorXd& vec) { SRB_wrench_ref = vec; }

    // Getter function for Jv_mat
    MatrixXd get_Jv_mat(int index) const { return Jv_mat[index]; }

    // Getter and setter functions for lfv0
    MatrixXd get_lfv0() const { return lfv0; }
    void set_lfv0(const MatrixXd& mat) { lfv0 = mat; }

    // Getter and setter functions for lfdv0
    MatrixXd get_lfdv0() const { return lfdv0; }
    void set_lfdv0(const MatrixXd& mat) { lfdv0 = mat; }

    // Getter and setter functions for q0
    MatrixXd get_q0() const { return q0; }
    void set_q0(const MatrixXd& mat) { q0 = mat; }

    // Getter and setter functions for qd0
    MatrixXd get_qd0() const { return qd0; }
    void set_qd0(const MatrixXd& mat) { qd0 = mat; }

    // Getter and setter functions for u0
    VectorXd get_u0() const { return u0; }
    void set_u0(const VectorXd& vec) { u0 = vec; }

    // Getter and setter functions for x0
    VectorXd get_x0() const { return x0; }
    void set_x0(const VectorXd& vec) { x0 = vec; }

    // Getter and setter functions for x0
    VectorXd get_ddpc_world() const { return _ddpc; }
    void set_ddpc_world(const Vector3d& vec) { _ddpc = vec; }

    // Getter function for right_leg_last
    MatrixXd get_right_leg_last() const { return right_leg_last; }

    // Getter function for left_leg_last
    MatrixXd get_left_leg_last() const { return left_leg_last; }

    // Getter function for isSwingToStanceLeft
    bool get_isSwingToStanceLeft() const { return isSwingToStanceLeft; }

    // Getter function for isSwingToStanceRight
    bool get_isSwingToStanceRight() const { return isSwingToStanceRight; }

    // Getter function for transitionStartLeft
    double get_transitionStartLeft() const { return transitionStartLeft; }

    // Getter function for transitionStartRight
    double get_transitionStartRight() const { return transitionStartRight; }

    // Getter function for rotation matrices from body to feet
    MatrixXd get_foot_orientation_wrt_body(VectorXd q_leg);

    // Getter function for Center of Mass Z from Ground (only works on flat ground)
    double get_CoM_z(MatrixXd lfv_hip,VectorXd gnd_contacts, Vector3d EA);

    // Getter function for _EA
    Eigen::Vector3d get_EA() const {return _EA;}

    // Getter function for _dEA
    Eigen::Vector3d get_dEA() const {return _dEA;}

    // Getter function for _pc
    Eigen::Vector3d get_pc() const {return _pc;}

    // Getter function for _dpc
    Eigen::Vector3d get_dpc() const {return _dpc;}

    // Getter function for step_z_history_L
    Eigen::VectorXd getStepZHistoryL() const { return step_z_history_L;}

    // Setter function for step_z_history_L
    void setStepZHistoryL(const Eigen::VectorXd& newStepZHistoryL) {step_z_history_L = newStepZHistoryL;}
    void updateStepZHistoryL(const double val)
    {
      dash_utils::updateAndShift(step_z_history_L,val);
    }

    // Getter function for step_z_history_R
    Eigen::VectorXd getStepZHistoryR() const {return step_z_history_R;}

    // Setter function for step_z_history_R
    void setStepZHistoryR(const Eigen::VectorXd& newStepZHistoryR) {step_z_history_R = newStepZHistoryR;}
    void updateStepZHistoryR(const double val) 
    {
      dash_utils::updateAndShift(step_z_history_R,val);
    }

    Eigen::VectorXd getStepTimeHistory() const {return step_time_history;}
    void updateStepTimeHistory(const double val) 
    {
      dash_utils::updateAndShift(step_time_history,val);
    }



    // Duration measurement functions for debugging
    void start_timer();
    void end_timer();

    // Getter and setter functions for simulation_mode
    double get_sim_mode() const { return simulation_mode; }
    void set_sim_mode(int sim_mode) { simulation_mode = sim_mode; }

    void reset();

    void enable_human_ctrl(){ enable_human_dyn_data = true;}
    void disable_human_ctrl(){ enable_human_dyn_data = false;}
    bool is_human_ctrl_enabled(){return enable_human_dyn_data; }
    bool human_playback_enabled(){return using_human_playback; }

    void set_prev_step_duration(double dur){prev_step_duration = dur;}
    double get_prev_step_duration(){return prev_step_duration;}
    void set_prev_step_amplitude(double ah){prev_step_amplitude = ah;}
    double get_prev_step_amplitude(){return prev_step_amplitude;}

    void set_xdata(VectorXd data){xdata = data;}
    VectorXd get_xdata(){return xdata;}
    void set_ydata(VectorXd data){ydata = data;}
    VectorXd get_ydata(){return ydata;}
    void set_timevec(VectorXd data){timevec = data;}
    VectorXd get_timevec(){return timevec;}
    void set_AHvec(VectorXd data){AHvec = data;}
    VectorXd get_AHvec(){return AHvec;}

    bool enable_human_dyn_data = false;

  private:

    SRB_Params srb_params;
    Human_params human_params;
    Human_dyn_data human_dyn_data;
    Traj_planner_dyn_data traj_planner_dyn_data;
    double t = 0;
    int FSM = 0;
    int FSM_prev = 0;
    VectorXd net_external_wrench = VectorXd::Zero(6);
    VectorXd x_next = VectorXd::Zero(21);

    Vector3d _EA;
    Vector3d _dEA;
    Vector3d _pc;
    Vector3d _dpc;
    Vector3d _ddpc;

    VectorXd x = VectorXd(21);
    MatrixXd q = MatrixXd(2,5); 
    MatrixXd qd = MatrixXd(2,5); 

    MatrixXd lfv = MatrixXd(4,3); 
    MatrixXd lfv_dsp_start = MatrixXd(4,3); 
    MatrixXd lfdv = MatrixXd(4,3); 
    MatrixXd lfv_comm = MatrixXd(4,3); 
    MatrixXd lfdv_comm = MatrixXd(4,3); 
    MatrixXd lfddv_comm = MatrixXd(4,3); 
    VectorXd u = VectorXd(12);

    // Initial States
    MatrixXd lfv0= MatrixXd(4,3);
    MatrixXd lfdv0= MatrixXd(4,3);
    MatrixXd q0 = MatrixXd(2,5); 
    MatrixXd qd0 = MatrixXd(2,5); 
    VectorXd u0 = VectorXd(12);
    VectorXd x0 = VectorXd(21);

    VectorXd tau_ext = VectorXd::Zero(6);
    VectorXd tau;
    MatrixXd SRB_state_ref = MatrixXd(6,2); 
    VectorXd SRB_wrench_ref  = VectorXd(6);
    MatrixXd Jv_mat[4];

    MatrixXd right_leg_last = MatrixXd(3,5);
    MatrixXd left_leg_last = MatrixXd(3,5);

    bool isSwingToStanceLeft = true;
    bool isSwingToStanceRight = true;
    double transitionStartLeft = -10;
    double transitionStartRight = -10;

    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point end_time;
    std::chrono::high_resolution_clock::time_point last_print_time;

    int simulation_mode = 1;

    bool using_human_playback = false;

    VectorXd step_z_history_L = VectorXd::Zero(1000); // last one second of step z data
    VectorXd step_z_history_R = VectorXd::Zero(1000);
    VectorXd step_time_history = VectorXd::Zero(1000);

    VectorXd xdata; // data for curve fitting
    VectorXd ydata;

    double prev_step_duration = 0.4;
    double prev_step_amplitude = 0.03;
    Eigen::VectorXd timevec = VectorXd(100);
    Eigen::VectorXd AHvec = VectorXd(100);


};

#endif