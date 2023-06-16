#include <Eigen/Dense>
#include <math.h>
#include <string>
#include "structs.h"
#include "utilities.h"
#include "controllers.h"
#include "dynamics.h"

namespace  dash_planner
{
    void SRB_6DoF_Test(std::string& recording_file_name, double& sim_time, SRB_Params& srb_params, MatrixXd lfv, char DoF, int num_tests);

    int SRB_FSM(SRB_Params srb_params,Traj_planner_dyn_data traj_planner_dyn_data, Human_dyn_data human_dyn_data, int FSM_prev, double t, MatrixXd lfv, VectorXd u);

    void SRB_Init_Traj_Planner_Data(Traj_planner_dyn_data& traj_planner_dyn_data, SRB_Params srb_params, Human_params human_params, VectorXd x0, MatrixXd lfv0);

    void SRB_Traj_Planner(SRB_Params srb_params, Human_dyn_data &human_dyn_data, Traj_planner_dyn_data &traj_planner_dyn_data, Human_params human_params,
                            int &FSM, int FSM_prev, double t, VectorXd x, MatrixXd lfv, MatrixXd lfdv, VectorXd u, VectorXd tau_ext, MatrixXd &SRB_state_ref,
                            VectorXd &SRB_wrench_ref, MatrixXd &lfv_comm, MatrixXd &lfdv_comm, MatrixXd &lfddv_comm);

    void traj_planner_dyn_data_gen(SRB_Params& srb_params, Human_params& human_params, Traj_planner_dyn_data& traj_planner_dyn_data, Human_dyn_data human_dyn_data,double t,int FSM_prev,int FSM, VectorXd x, MatrixXd lfv);

    void gen_vel_trapz_traj(const VectorXd& t_waypts, const VectorXd& v_waypts, VectorXd& t_traj, VectorXd& v_traj);
    void gen_smooth_traj(const VectorXd& t_waypts, const VectorXd& v_waypts, VectorXd& t_traj, VectorXd& v_traj);

    void SRB_LIP_vel_traj(double des_walking_speed, VectorXd& t_traj, VectorXd& v_traj, double& t_beg_stepping_time, double& t_end_stepping_time);
}

