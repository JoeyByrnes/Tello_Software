#include <Eigen/Dense>
#include <math.h>
#include "structs.h"
#include "utilities.h"
#include "qpOASES.hpp"
#include <limits.h>
#include "dynamics.h"

using namespace qpOASES;

namespace dash_ctrl
{
    
    void Human_Whole_Body_Dyn_Telelocomotion(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                            SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                            int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext);

    void Human_Whole_Body_Dyn_Telelocomotion_v2(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                            SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                            int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext);  

    void Human_Whole_Body_Dyn_Telelocomotion_v3(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                            SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                            int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext, VectorXd u);  

    void Human_Whole_Body_Dyn_Telelocomotion_v4(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, Human_dyn_data& human_dyn_data, 
                                            HMI_extended_data hmi_extended_data, SRB_Params srb_params, Human_params human_params, Traj_planner_dyn_data& traj_planner_dyn_data, 
                                            int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd tau_ext); 

    void bilateral_teleop_law(VectorXd LIPR_params, VectorXd LIPH_params, VectorXd LIPR_dyn, VectorXd LIPH_dyn, 
                            double FH, double FR_ext_est, double FB_gain, double& FR, double& FH_hmi);

    void LIP_ang_mom_strat(double& FxR, double& FyR, MatrixXd& lfv_comm, MatrixXd& lfddv_comm, MatrixXd& lfdv_comm,
                            SRB_Params srb_params, Traj_planner_dyn_data traj_planner_dyn_data, 
                            int FSM, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv);

    void sw2CoM_end_step_strategy(MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, const SRB_Params srb_params, const Traj_planner_dyn_data& traj_planner_dyn_data, const int FSM, 
                                const double s, const VectorXd& x, MatrixXd& lfv, MatrixXd& lfdv, const Vector2d& sw2CoM_end_step_des);

    void sw_teleop_step_strategy(MatrixXd& lfv_comm, MatrixXd& lfdv_comm, MatrixXd& lfddv_comm, const SRB_Params srb_params, const Human_params human_params, 
                                const Traj_planner_dyn_data& traj_planner_dyn_data, const Human_dyn_data& human_dyn_data, 
                                const int FSM, const double s, const double swxf, MatrixXd& lfv, MatrixXd& lfdv, VectorXd x);                                 
    
    void SRB_Balance_Controller(VectorXd& u, VectorXd& tau_legs, SRB_Params srb_params, int FSM, VectorXd x, 
                            MatrixXd lfv, MatrixXd qd, MatrixXd* Jv_mat, VectorXd u0, VectorXd SRB_wrench_ref);

    VectorXd SRB_force_distribution_QP(SRB_Params srb_params,int FSM,VectorXd x,MatrixXd lfv,MatrixXd qd,MatrixXd* Jv_mat,
                                    VectorXd u0,VectorXd tau_SRB_des);

    VectorXd calc_joint_torques(VectorXd x, MatrixXd* Jv_mat, VectorXd GRFs);
    
    void cost_quadratic_coeff_matrices(const Eigen::VectorXd coeff, double C, const Eigen::VectorXi& idx, 
                                    int num_opt_vars_act, Eigen::MatrixXd& Q, Eigen::VectorXd& f);
    void SRB_force_distribution_QP_constraints(MatrixXd &A, VectorXd &b, MatrixXd &Aeq, VectorXd &beq, MatrixXd &LB, 
                                            MatrixXd &UB, const SRB_Params &srb_params, const int FSM, const VectorXd &x, 
                                            const MatrixXd &qd, const MatrixXd* Jv_mat);
    void friction_cone_constraints(SRB_Params srb_params, MatrixXd& A_GRF_fric_nopull, VectorXd& ubA_GRF_fric_nopull, int& num_const_total);
    
    void parallel_actuation_constraints(SRB_Params srb_params,const VectorXd& x,const MatrixXd& qd,const MatrixXd* Jv_mat,
                                    MatrixXd& A_GRF_torque_lim,VectorXd& ubA_GRF_torque_lim,int& num_const_total);

    VectorXd SRB_PD_Wrench_Controller(SRB_Params srb_params, VectorXd x, MatrixXd SRB_state_ref, MatrixXd SRB_wrench_FF);
    
    void opt_stepping_controller(double& uk, VectorXd xk, VectorXd xk_des, double T_SSP, double T_DSP, double w);
    
}
