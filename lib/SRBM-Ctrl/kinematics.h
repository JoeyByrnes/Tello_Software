#include <Eigen/Dense>
#include <math.h>
#include "structs.h"
#include "utilities.h"
#include <algorithm>
#include <cmath>

#define NUM_EE_POINTS 4 // two per foot
#define POSITION_VEC_SIZE 3 // x, y, z components
#define RIGHT_EE_IDX 0 // idx of front right line foot pt
#define LEFT_EE_IDX 2 // idx of front left line foot pt
#define LEG_DOF 5 // leg number of degrees of freedom


#define CoM_X_Offset 0.010

using namespace Eigen;

namespace  dash_kin
{
    MatrixXd fcn_HTM0lf1(VectorXd q, VectorXd p);
    MatrixXd fcn_HTM0lf2(VectorXd q, VectorXd p);
    MatrixXd fcn_HTM01(VectorXd q, VectorXd p);
    MatrixXd fcn_HTM02(VectorXd q, VectorXd p);
    MatrixXd fcn_HTM03(VectorXd q, VectorXd p);
    MatrixXd fcn_lf1_J(VectorXd q, VectorXd p);
    MatrixXd fcn_lf2_J(VectorXd q, VectorXd p);

    void SRB_FK(MatrixXd& torso_vertices, MatrixXd& right_leg, MatrixXd& left_leg, MatrixXd& lfv, SRB_Params srb_params, VectorXd x, MatrixXd q);
    MatrixXd SRB_IK(SRB_Params srb_params, VectorXd CoM, MatrixXd R, MatrixXd lfv);
    void SRB_Kin(MatrixXd& q, MatrixXd& qd, MatrixXd* Jv_mat, SRB_Params srb_params, VectorXd x, MatrixXd lfv, MatrixXd lfdv);

    MatrixXd compute_torso_vertices_locations(MatrixXd HTMwd2com,VectorXd p_torso);
    MatrixXd leg_FK(MatrixXd HTMwd2hip, VectorXd q, VectorXd p_leg);

    double Paden_Kahan_subproblem1(const Vector3d& p_PK_subp1, const Vector3d& q_PK_subp1, const Vector3d& r_PK_subp1, const Vector3d& omega_PK_subp1);
    void Paden_Kahan_subproblem2(Vector2d& sol1_PK_subp2,Vector2d& sol2_PK_subp2, Vector3d p_PK_subp2, Vector3d q_PK_subp2, Vector3d r_PK_subp2, Vector3d omega1_PK_subp2, Vector3d omega2_PK_subp2);

    VectorXd SRB_Leg_IK(SRB_Params& srb_params, VectorXd& lf1, VectorXd& lf2);
    VectorXd Assess_PKsubp2_sols(Vector2d sol1,Vector2d sol2,Vector3d n1,Vector3d n2,Vector2d q1_lim,Vector2d q2_lim);

    VectorXd calc_joint_vel(VectorXd qd_b, Matrix3d Rwb,Vector3d x_torso, VectorXd x_dot_trans, MatrixXd J_leg);
}
 