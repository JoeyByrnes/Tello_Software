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
    ~SRBMController() {}

    VectorXd update(Vector3d body_position, Vector3d body_linear_velocity,
                                 Vector3d body_orientation, Vector3d body_angular_velocity,
                                 MatrixXd q, MatrixXd qd, double t);  
    VectorXd update(VectorXd srb_state, MatrixXd q, MatrixXd qd, double t);  

    // Getter and setter functions for SRB_Params
    SRB_Params get_SRB_params() const { return srb_params; }
    void set_SRB_params(const SRB_Params& params) { srb_params = params; }

    // Getter and setter functions for Human_params
    Human_params get_human_params() const { return human_params; }
    void set_human_params(const Human_params& params) { human_params = params; }

    // Getter and setter functions for Human_dyn_data
    Human_dyn_data get_human_dyn_data() const { return human_dyn_data; }
    void set_human_dyn_data(const Human_dyn_data& data) { human_dyn_data = data; }

    // Getter and setter functions for Traj_planner_dyn_data
    Traj_planner_dyn_data get_traj_planner_dyn_data() const { return traj_planner_dyn_data; }
    void set_traj_planner_dyn_data(const Traj_planner_dyn_data& data) { traj_planner_dyn_data = data; }

    // Getter and setter functions for t
    double get_time() const { return t; }
    void set_time(double time) { t = time; }

    // Getter function for FSM
    int get_FSM() const { return FSM; }

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

  private:

    SRB_Params srb_params;
    Human_params human_params;
    Human_dyn_data human_dyn_data;
    Traj_planner_dyn_data traj_planner_dyn_data;
    double t = 0;
    int FSM = 0;
    int FSM_prev = 0;

    VectorXd x = VectorXd(21);
    MatrixXd q = MatrixXd(2,5); 
    MatrixXd qd = MatrixXd(2,5); 

    MatrixXd lfv = MatrixXd(4,3); 
    MatrixXd lfv_dsp_start = MatrixXd(4,3); 
    MatrixXd lfdv = MatrixXd(4,3); 
    MatrixXd lfv_comm = MatrixXd(4,3); 
    MatrixXd lfdv_comm = MatrixXd(4,3); 
    VectorXd u = VectorXd(12);

    // Initial States
    MatrixXd lfv0= MatrixXd(4,3);
    MatrixXd lfdv0= MatrixXd(4,3);
    MatrixXd q0 = MatrixXd(2,5); 
    MatrixXd qd0 = MatrixXd(2,5); 
    VectorXd u0 = VectorXd(12);
    VectorXd x0 = VectorXd(21);

    VectorXd tau_ext; 
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


};