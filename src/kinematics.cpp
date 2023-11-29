#include "kinematics.h"

#define DEGREE_TO_RADIANS (M_PI/180.0)

using namespace Eigen;

static double atan2_safe(double u0, double u1)
{
  double y;
  int i;
  int i1;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = 0;
  } 
  else if (std::isinf(u0) && std::isinf(u1)) {
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2(i, i1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = M_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(M_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

VectorXd fcn_ik_q_2_p(const VectorXd &q)
{
    VectorXd p(5);

    // First actuation group:
    // hip yaw (q1) - hip yaw actuator (p1)
    p(0) = q(0);

    // Second actuation group: hip differential
    // {hip roll (q2), hip pitch (q3)} - {actuator2 (p2), actuator3 (p3)}
    double a1 = 57.0/2500.0 - (8.0*cos(q(1))*sin(q(2)))/625.0 - (7.0*sin(q(1)))/625.0;
    double b1 = -(8.0*cos(q(2)))/625.0;
    double c1 = (49.0*cos(q(1)))/5000.0 + (399.0*sin(q(1)))/20000.0 +
        (57.0*cos(q(1))*sin(q(2)))/2500.0 - (7.0*sin(q(1))*sin(q(2)))/625.0 - 3021.0/160000.0;
    double d1 = sqrt(pow(a1,2) + pow(b1,2) - pow(c1,2));
    double a2 = (7.0*sin(q(1)))/625.0 - (8.0*cos(q(1))*sin(q(2)))/625.0 + 57.0/2500.0;
    double b2 = -(8.0*cos(q(2)))/625.0;
    double c2 = (49.0*cos(q(1)))/5000.0 - (399.0*sin(q(1)))/20000.0 +
        (57.0*cos(q(1))*sin(q(2)))/2500.0 + (7.0*sin(q(1))*sin(q(2)))/625.0 - 3021.0/160000.0;
    double d2 = sqrt(pow(a2,2) + pow(b2,2) - pow(c2,2));

    p(1) = atan2_safe(-b1*d1 + a1*c1, b1*c1+a1*d1);
    p(2) = atan2_safe(-b2*d2 + a2*c2, b2*c2+a2*d2);

    // Third actuation group: knee-ankle differential
    // {knee pitch (q4), ankle pitch (q5)} - {actuator4 (p4), actuator5 (p5)}
    double alpha_0 = (180.0-59.16)*M_PI/180.0;  // alpha_0
    double beta_0 = (180.0-66.0)*M_PI/180.0;   // beta_0
    double beta_ = q(4) + beta_0;
    double a3 = - (21.0 * sin(M_PI/9.0)) / 6250.0 - (7.0 * sin(beta_)) / 2500.0;
    double b3 = 13.0 / 625.0 - (7.0 * cos(beta_)) / 2500.0 - (21.0 * cos(M_PI/9.0)) / 6250.0;
    double c3 = (273.0 * cos(M_PI/9)) / 12500.0 + (91.0 * cos(beta_)) / 5000.0 - 
        (147.0 * cos(M_PI/9.0) * cos(beta_)) / 50000.0 - 
        (147.0 * sin(M_PI/9.0) * sin(beta_)) / 50000.0 - 163349.0 / 6250000.0;
    double d3 = sqrt(pow(a3, 2) + pow(b3, 2) - pow(c3, 2));
    double alpha_ = atan2_safe(b3 * d3 + a3 * c3, b3 * c3 - a3 * d3);

    p(3) = q(3) - (alpha_ - alpha_0);
    p(4) = q(3) + (alpha_ - alpha_0);

    return p;
}


Eigen::MatrixXd fcn_Jaco_dq_2_dp(const Eigen::VectorXd& eig_q)
{
  double q[5];
  for(int i = 0; i<5; i++){
    q[i] = eig_q(i);
  }
  double J[25];
  double J_tmp;
  double a1;
  double a1_tmp_tmp;
  double a2;
  double b1_tmp;
  double b1_tmp_tmp;
  double b_a1_tmp_tmp;
  double c1;
  double c2;
  double c_a1_tmp_tmp;
  double d1;
  double d2;
  double p__idx_1;
  double p__idx_2;

  /*  first actuation group  */
  /*  hip yaw (q1) - hip yaw actuator (p1) */
  /*  second actuation group - hip differential */
  /*  {hip roll (q2), hip pitch (q3)} - {actuator2 (p2), actuator3 (p3)} */
  a1_tmp_tmp = cos(q[1]);
  b_a1_tmp_tmp = sin(q[1]);
  c_a1_tmp_tmp = sin(q[2]);
  a1 = (0.0228 - 8.0 * a1_tmp_tmp * c_a1_tmp_tmp / 625.0) -
        7.0 * b_a1_tmp_tmp / 625.0;
  b1_tmp_tmp = cos(q[2]);
  b1_tmp = -(8.0 * b1_tmp_tmp) / 625.0;
  c1 = (((49.0 * a1_tmp_tmp / 5000.0 + 399.0 * b_a1_tmp_tmp / 20000.0) +
          57.0 * a1_tmp_tmp * c_a1_tmp_tmp / 2500.0) -
        7.0 * sin(q[1]) * c_a1_tmp_tmp / 625.0) -
        0.01888125;
  d2 = b1_tmp * b1_tmp;
  d1 = sqrt((a1 * a1 + d2) - c1 * c1);
  a2 = (7.0 * sin(q[1]) / 625.0 - 8.0 * cos(q[1]) * sin(q[2]) / 625.0) + 0.0228;
  c2 = (((49.0 * cos(q[1]) / 5000.0 - 399.0 * sin(q[1]) / 20000.0) +
          57.0 * cos(q[1]) * sin(q[2]) / 2500.0) +
        7.0 * sin(q[1]) * sin(q[2]) / 625.0) -
        0.01888125;
  d2 = sqrt((a2 * a2 + d2) - c2 * c2);
  p__idx_1 = atan2_safe(-b1_tmp * d1 + a1 * c1, b1_tmp * c1 + a1 * d1);
  p__idx_2 = atan2_safe(-b1_tmp * d2 + a2 * c2, b1_tmp * c2 + a2 * d2);
  /*  third actuation group - knee-ankle differential */
  /*  {knee pitch (q4), ankle pitch (q5)} - {actuator4 (p4), actuator5 (p5)} */
  /*  alpha_0 */
  /*  beta_0 */
  d2 = cos(q[4] + 1.9896753472735356);
  b1_tmp = sin(q[4] + 1.9896753472735356);
  d1 = (7.0 * d2 / 2500.0 + 0.0031573672058406521) - 0.0208;
  a1 = (((147.0 * d2 * 0.93969262078590843 / 50000.0 - 0.020522886837964237) -
          91.0 * d2 / 5000.0) +
        147.0 * b1_tmp * 0.34202014332566871 / 50000.0) +
        0.02613584;
  d2 = 7.0 * b1_tmp / 2500.0 + 0.001149187681574247;
  c1 = sqrt((d1 * d1 - a1 * a1) + d2 * d2);
  d2 = atan2_safe(d2 * a1 - c1 * d1, d1 * a1 + d2 * c1);
  a1 = 140.0 * sin(d2 - (q[4] + 1.9896753472735356));
  c2 = ((168.0 * sin(d2 - 0.3490658503988659) - 1040.0 * sin(d2)) + a1) /
        ((147.0 * sin((q[4] + 1.9896753472735356) - 0.3490658503988659) -
          910.0 * b1_tmp) +
        a1);
  memset(&J[0], 0, 25U * sizeof(double));
  J[0] = 1.0;
  J_tmp = sin(p__idx_1);
  a1 = cos(p__idx_1);
  c1 = 224.0 * a1_tmp_tmp;
  d1 = c1 * c_a1_tmp_tmp;
  d2 = 456.0 * b_a1_tmp_tmp * c_a1_tmp_tmp;
  J[6] =
      -(((((196.0 * b_a1_tmp_tmp - 399.0 * a1_tmp_tmp) -
            224.0 * a1_tmp_tmp * J_tmp) +
          d1) +
          d2) +
        256.0 * J_tmp * b_a1_tmp_tmp * c_a1_tmp_tmp) /
      (((456.0 * a1 - 224.0 * a1 * b_a1_tmp_tmp) + 256.0 * b1_tmp_tmp * J_tmp) -
        256.0 * a1 * a1_tmp_tmp * c_a1_tmp_tmp);
  b1_tmp = 32.0 * a1;
  a2 = 32.0 * a1_tmp_tmp * b1_tmp_tmp;
  J[11] =
      (((57.0 * a1_tmp_tmp * b1_tmp_tmp - b1_tmp * c_a1_tmp_tmp) -
        28.0 * b1_tmp_tmp * b_a1_tmp_tmp) +
        a2 * J_tmp) /
      (((57.0 * a1 - 28.0 * a1 * b_a1_tmp_tmp) + 32.0 * b1_tmp_tmp * J_tmp) -
        b1_tmp * a1_tmp_tmp * c_a1_tmp_tmp);
  J_tmp = sin(p__idx_2);
  a1 = cos(p__idx_2);
  J[7] =
      -(((((399.0 * a1_tmp_tmp + 196.0 * b_a1_tmp_tmp) + c1 * J_tmp) - d1) +
          d2) +
        256.0 * J_tmp * b_a1_tmp_tmp * c_a1_tmp_tmp) /
      (((456.0 * a1 + 224.0 * a1 * b_a1_tmp_tmp) + 256.0 * cos(q[2]) * J_tmp) -
        256.0 * a1 * a1_tmp_tmp * c_a1_tmp_tmp);
  c1 = 32.0 * a1;
  J[12] = (((57.0 * cos(q[1]) * b1_tmp_tmp - c1 * c_a1_tmp_tmp) +
            28.0 * cos(q[2]) * b_a1_tmp_tmp) +
            a2 * J_tmp) /
          (((57.0 * a1 + 28.0 * a1 * b_a1_tmp_tmp) + 32.0 * cos(q[2]) * J_tmp) -
            c1 * a1_tmp_tmp * c_a1_tmp_tmp);
  J[18] = 1.0;
  J[23] = 1.0 / c2;
  J[19] = 1.0;
  J[24] = -1.0 / c2;

  // Create a 5x5 Eigen::MatrixXd object
  Eigen::MatrixXd Jaco(5, 5);

  // Copy the values from the Jacobian array to the Eigen::MatrixXd object
  for (int j = 0; j < 5; j++) {
      for (int i = 0; i < 5; i++) {
          Jaco(i, j) = J[j * 5 + i];
      }
  }

  return Jaco;

}

Eigen::VectorXd tello_leg_IK(Eigen::VectorXd& lf1, Eigen::VectorXd& lf2)
{
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0576;
  srb_params.foot_length = 0.060;

  return dash_kin::SRB_Leg_IK(srb_params,lf1,lf2);
}

Eigen::VectorXd tello_leg_IK_biped(const Eigen::VectorXd& task_positions)
{
  VectorXd lf1_left = task_positions.segment(0,3);
  VectorXd lf2_left = task_positions.segment(3,3);
  VectorXd lf1_right = task_positions.segment(6,3);
  VectorXd lf2_right = task_positions.segment(9,3);

  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-20*DEGREE_TO_RADIANS,20*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-15*DEGREE_TO_RADIANS,15*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0576;
  srb_params.foot_length = 0.060;

  VectorXd q_left = dash_kin::SRB_Leg_IK(srb_params,lf1_left,lf2_left);
  VectorXd q_right = dash_kin::SRB_Leg_IK(srb_params,lf1_right,lf2_right);
  VectorXd q(10);
  q << q_left, q_right;
  return q;
}

Eigen::VectorXd tello_leg_IK_pointFoot(const Eigen::VectorXd& pos)
{
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0576; // used to be 0.0485
  srb_params.foot_length = 0.060;
  VectorXd lf1 = pos + Vector3d(srb_params.foot_length,0,0);
  VectorXd lf2 = pos - Vector3d(srb_params.foot_length,0,0);

  return dash_kin::SRB_Leg_IK(srb_params,lf1,lf2);
}

Eigen::MatrixXd fcn_Jaco_dq_2_dT_front(const Eigen::VectorXd& q){
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0576;
  srb_params.foot_length = 0.120;

  VectorXd p = Vector4d(srb_params.thigh_length,srb_params.calf_length,srb_params.foot_length,srb_params.heel_length);
  return dash_kin::fcn_lf1_J(q, p);
}
Eigen::MatrixXd fcn_Jaco_dq_2_dT_back(const Eigen::VectorXd& q){
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0576;
  srb_params.foot_length = 0.120;

  VectorXd p = Vector4d(srb_params.thigh_length,srb_params.calf_length,srb_params.foot_length,srb_params.heel_length);
  return dash_kin::fcn_lf2_J(q, p);
}


VectorXd fk_motors_to_joints(const VectorXd& p)
{
  Eigen::Matrix<double,5,1> q;
  // HIP IE
  q(0) = p(0);
  // HIP AA
  double ad = 0.013435*pow(p(1), 6) - 0.046602*pow(p(1), 5)*p(2) - 0.033703*pow(p(1), 5)
           + 0.036565*pow(p(1), 4)*pow(p(2), 2) + 0.14746*pow(p(1), 4)*p(2) + 0.022238*pow(p(1), 4)
           + 0.027702*pow(p(1), 3)*pow(p(2), 3) - 0.22557*pow(p(1), 3)*pow(p(2), 2) - 0.049307*pow(p(1), 3)*p(2)
           - 0.054554*pow(p(1), 3) - 0.082618*pow(p(1), 2)*pow(p(2), 4) + 0.23134*pow(p(1), 2)*pow(p(2), 3)
           + 0.020987*pow(p(1), 2)*pow(p(2), 2) - 0.088424*pow(p(1), 2)*p(2) - 0.0022161*pow(p(1), 2)
           + 0.069313*p(1)*pow(p(2), 5) - 0.15597*p(1)*pow(p(2), 4) + 0.022842*p(1)*pow(p(2), 3)
           + 0.088701*p(1)*pow(p(2), 2) + 0.00041012*p(1)*p(2) + 0.57241*p(1) - 0.018189*pow(p(2), 6)
           + 0.036455*pow(p(2), 5) - 0.015988*pow(p(2), 4) + 0.054298*pow(p(2), 3)
           + 0.0014228*pow(p(2), 2) - 0.57243*p(2) + 3.703e-05;
  q(1) = ad;
  // HIP FE
  double fe = 0.14104 * std::pow(p(1), 6) 
           - 0.76434 * std::pow(p(1), 5) * p(2) 
           - 0.02453 * std::pow(p(1), 5) 
           + 1.8141 * std::pow(p(1), 4) * std::pow(p(2), 2) 
           - 0.056086 * std::pow(p(1), 4) * p(2) 
           - 0.044641 * std::pow(p(1), 4) 
           - 2.4487 * std::pow(p(1), 3) * std::pow(p(2), 3) 
           + 0.27126 * std::pow(p(1), 3) * std::pow(p(2), 2) 
           + 0.26961 * std::pow(p(1), 3) * p(2) 
           + 0.021561 * std::pow(p(1), 3) 
           + 1.8163 * std::pow(p(1), 2) * std::pow(p(2), 4) 
           - 0.23275 * std::pow(p(1), 2) * std::pow(p(2), 3) 
           - 0.42397 * std::pow(p(1), 2) * std::pow(p(2), 2) 
           - 0.033613 * std::pow(p(1), 2) * p(2) 
           - 0.034881 * std::pow(p(1), 2) 
           - 0.76579 * p(1) * std::pow(p(2), 5) 
           + 0.19587 * p(1) * std::pow(p(2), 4) 
           + 0.26225 * p(1) * std::pow(p(2), 3) 
           - 0.064396 * p(1) * std::pow(p(2), 2) 
           - 0.016903 * p(1) * p(2) 
           + 0.51084 * p(1) 
           + 0.14082 * std::pow(p(2), 6) 
           - 0.074759 * std::pow(p(2), 5) 
           - 0.039853 * std::pow(p(2), 4) 
           + 0.031557 * std::pow(p(2), 3) 
           - 0.035741 * std::pow(p(2), 2) 
           + 0.51062 * p(2) 
           - 0.15647;
  q(2) = fe;
  // KNEE
  double kn = 0.5*p(3) + 0.5*p(4);
  q(3) = kn;
  // ANKLE
  double a = -0.0072907 * pow(p(3), 4)
           + 0.029163 * pow(p(3), 3) * p(4)
           + 0.0090622 * pow(p(3), 3)
           - 0.043745 * pow(p(3), 2) * pow(p(4), 2)
           - 0.027187 * pow(p(3), 2) * p(4)
           - 0.011608 * pow(p(3), 2)
           + 0.029163 * p(3) * pow(p(4), 3)
           + 0.027187 * pow(p(3), 1) * pow(p(4), 2)
           + 0.023217 * p(3) * p(4)
           - 0.53853 * p(3)
           - 0.0072908 * pow(p(4), 4)
           - 0.0090621 * pow(p(4), 3)
           - 0.011609 * pow(p(4), 2)
           + 0.53853 * p(4)
           - 0.0013079;
  q(4) = a;

  return q;
}

Eigen::VectorXd fk_joints_to_task_leg(const Eigen::VectorXd& q)
{
  Eigen::MatrixXd HTMwd2hip = Eigen::MatrixXd::Identity(4,4);

  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0576;
  srb_params.foot_length = 0.120;

  VectorXd p = Vector4d(srb_params.thigh_length,srb_params.calf_length,srb_params.foot_length,srb_params.heel_length);


  Eigen::MatrixXd fk = dash_kin::leg_FK(HTMwd2hip, q, p);

  Eigen::VectorXd lf1 = fk.col(3);
  Eigen::VectorXd lf2 = fk.col(4);

  Eigen::VectorXd task_positions(6);
  task_positions << lf1, lf2;

  return task_positions;
}

Eigen::VectorXd fk_joints_to_task(const Eigen::VectorXd& q)
{
  Eigen::VectorXd q_left = q.segment(0,5);
  Eigen::VectorXd q_right = q.segment(5,5);

  Eigen::VectorXd task_pos_left = fk_joints_to_task_leg(q_left);
  Eigen::VectorXd task_pos_right = fk_joints_to_task_leg(q_right);
  Eigen::VectorXd task_positions(12);
  task_positions << task_pos_left, task_pos_right;
  return task_positions;
}

Eigen::VectorXd ik_joints_to_motors(const Eigen::VectorXd& q)
{
  Eigen::VectorXd q_left(5), q_right(5);
  q_left = q.segment(0,5);
  q_right = q.segment(5,5);
  VectorXd p_left = fcn_ik_q_2_p(q_left);
  VectorXd p_right = fcn_ik_q_2_p(q_right);
  VectorXd p(10);
  p << p_left , p_right;
  return p;
}


// ============= CODER =========================================================================================



// =============================================================================================================

Eigen::MatrixXd fcn_Jaco_dp_2_dq(const Eigen::VectorXd& eig_q)
{
  double q[5];
  for(int i = 0; i<5; i++){
    q[i] = eig_q(i);
  }
  double J[25];
  double J_p_hip[4];
  double J_q_hip[4];
  double J_q_hip_tmp;
  double a1;
  double a1_tmp_tmp;
  double a2;
  double b1_tmp;
  double b1_tmp_tmp;
  double b_J_q_hip_tmp;
  double b_a1_tmp_tmp;
  double b_b1_tmp_tmp;
  double c1;
  double c2;
  double c_a1_tmp_tmp;
  double d1;
  double d2;
  double lambda_;
  double p__idx_2;
  int r1;
  int r2;
  // if (!isInitialized_fcn_Jaco_dp_2_dq) {
  //   fcn_Jaco_dp_2_dq_initialize();             // JB REMOVED
  // }
  /*  first actuation group  */
  /*  hip yaw (q1) - hip yaw actuator (p1) */
  /*  second actuation group - hip differential */
  /*  {hip roll (q2), hip pitch (q3)} - {actuator2 (p2), actuator3 (p3)} */
  a1_tmp_tmp = cos(q[1]);
  b_a1_tmp_tmp = sin(q[1]);
  c_a1_tmp_tmp = sin(q[2]);
  a1 = (0.0228 - 8.0 * a1_tmp_tmp * c_a1_tmp_tmp / 625.0) -
       7.0 * b_a1_tmp_tmp / 625.0;
  b1_tmp_tmp = cos(q[2]);
  b_b1_tmp_tmp = 8.0 * b1_tmp_tmp;
  b1_tmp = -b_b1_tmp_tmp / 625.0;
  c1 = (((49.0 * a1_tmp_tmp / 5000.0 + 399.0 * b_a1_tmp_tmp / 20000.0) +
         57.0 * a1_tmp_tmp * c_a1_tmp_tmp / 2500.0) -
        7.0 * sin(q[1]) * c_a1_tmp_tmp / 625.0) -
       0.01888125;
  d2 = b1_tmp * b1_tmp;
  d1 = sqrt((a1 * a1 + d2) - c1 * c1);
  a2 = (7.0 * sin(q[1]) / 625.0 - 8.0 * cos(q[1]) * sin(q[2]) / 625.0) + 0.0228;
  c2 = (((49.0 * cos(q[1]) / 5000.0 - 399.0 * sin(q[1]) / 20000.0) +
         57.0 * cos(q[1]) * sin(q[2]) / 2500.0) +
        7.0 * sin(q[1]) * sin(q[2]) / 625.0) -
       0.01888125;
  d2 = sqrt((a2 * a2 + d2) - c2 * c2);
  d1 = atan2_safe(-b1_tmp * d1 + a1 * c1, b1_tmp * c1 + a1 * d1);
  p__idx_2 = atan2_safe(-b1_tmp * d2 + a2 * c2, b1_tmp * c2 + a2 * d2);
  /*  third actuation group - knee-ankle differential */
  /*  {knee pitch (q4), ankle pitch (q5)} - {actuator4 (p4), actuator5 (p5)} */
  /*  alpha_0 */
  /*  beta_0 */
  d2 = cos(q[4] + 1.9896753472735356);
  c1 = sin(q[4] + 1.9896753472735356);
  c2 = (7.0 * d2 / 2500.0 + 0.0031573672058406521) - 0.0208;
  d2 = (((147.0 * d2 * 0.93969262078590843 / 50000.0 - 0.020522886837964237) -
         91.0 * d2 / 5000.0) +
        147.0 * c1 * 0.34202014332566871 / 50000.0) +
       0.02613584;
  a1 = 7.0 * c1 / 2500.0 + 0.001149187681574247;
  a2 = sqrt((c2 * c2 - d2 * d2) + a1 * a1);
  d2 = atan2_safe(a1 * d2 - a2 * c2, c2 * d2 + a1 * a2);
  b1_tmp = 140.0 * sin(d2 - (q[4] + 1.9896753472735356));
  lambda_ =
      ((168.0 * sin(d2 - 0.3490658503988659) - 1040.0 * sin(d2)) + b1_tmp) /
      ((147.0 * sin((q[4] + 1.9896753472735356) - 0.3490658503988659) -
        910.0 * c1) +
       b1_tmp);
  d2 = sin(d1);
  b1_tmp = 7.0 * a1_tmp_tmp;
  a2 = 399.0 * a1_tmp_tmp / 20000.0;
  c2 = 49.0 * b_a1_tmp_tmp / 5000.0;
  J_q_hip_tmp = b1_tmp * c_a1_tmp_tmp / 625.0;
  b_J_q_hip_tmp = 57.0 * b_a1_tmp_tmp * c_a1_tmp_tmp / 2500.0;
  J_q_hip[0] = ((((c2 - a2) - 7.0 * a1_tmp_tmp * d2 / 625.0) + J_q_hip_tmp) +
                b_J_q_hip_tmp) +
               8.0 * d2 * b_a1_tmp_tmp * c_a1_tmp_tmp / 625.0;
  a1 = cos(d1);
  c1 = 8.0 * a1;
  J_q_hip[2] =
      ((c1 * c_a1_tmp_tmp / 625.0 - 57.0 * a1_tmp_tmp * b1_tmp_tmp / 2500.0) +
       7.0 * b1_tmp_tmp * b_a1_tmp_tmp / 625.0) -
      8.0 * a1_tmp_tmp * b1_tmp_tmp * d2 / 625.0;
  d1 = sin(p__idx_2);
  J_q_hip[1] =
      ((((a2 + c2) + b1_tmp * d1 / 625.0) - J_q_hip_tmp) + b_J_q_hip_tmp) +
      8.0 * d1 * b_a1_tmp_tmp * c_a1_tmp_tmp / 625.0;
  b1_tmp = cos(p__idx_2);
  a2 = 8.0 * b1_tmp;
  J_q_hip[3] =
      ((a2 * c_a1_tmp_tmp / 625.0 - 57.0 * cos(q[1]) * b1_tmp_tmp / 2500.0) -
       7.0 * cos(q[2]) * b_a1_tmp_tmp / 625.0) -
      8.0 * cos(q[1]) * b1_tmp_tmp * d1 / 625.0;
  J_q_hip[0] = -J_q_hip[0];
  J_p_hip[1] = 0.0;
  J_q_hip[1] = -J_q_hip[1];
  J_p_hip[2] = 0.0;
  J_q_hip[2] = -J_q_hip[2];
  J_q_hip[3] = -J_q_hip[3];
  J_p_hip[0] = ((57.0 * a1 / 2500.0 - 7.0 * a1 * b_a1_tmp_tmp / 625.0) +
                8.0 * b1_tmp_tmp * d2 / 625.0) -
               c1 * a1_tmp_tmp * c_a1_tmp_tmp / 625.0;
  J_p_hip[3] = ((57.0 * b1_tmp / 2500.0 + 7.0 * b1_tmp * b_a1_tmp_tmp / 625.0) +
                b_b1_tmp_tmp * d1 / 625.0) -
               a2 * a1_tmp_tmp * c_a1_tmp_tmp / 625.0;
  if (fabs(J_q_hip[1]) > fabs(J_q_hip[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }
  d2 = J_q_hip[r2] / J_q_hip[r1];
  b1_tmp = J_q_hip[r1 + 2];
  a1 = J_q_hip[r2 + 2] - d2 * b1_tmp;
  a2 = (J_p_hip[r2] - J_p_hip[r1] * d2) / a1;
  c1 = a2;
  d1 = (J_p_hip[r1] - a2 * b1_tmp) / J_q_hip[r1];
  c2 = J_p_hip[r1 + 2];
  a2 = (J_p_hip[r2 + 2] - c2 * d2) / a1;
  memset(&J[0], 0, 25U * sizeof(double));
  J[0] = 1.0;
  J[6] = d1;
  J[7] = c1;
  J[11] = (c2 - a2 * b1_tmp) / J_q_hip[r1];
  J[12] = a2;
  J[18] = 0.5;
  J[23] = 0.5;
  J[19] = lambda_ / 2.0;
  J[24] = -lambda_ / 2.0;

  // double *J = new double[25];
  // fcn_Jaco_dp_2_dq(q,J);

  // Create a 5x5 Eigen::MatrixXd object
  Eigen::MatrixXd Jaco(5, 5);

  // Copy the values from the Jacobian array to the Eigen::MatrixXd object
  for (int j = 0; j < 5; j++) {
      for (int i = 0; i < 5; i++) {
          Jaco(i, j) = J[j * 5 + i];
      }
  }

  return Jaco;

}


// MATLAB CODER INVERSE TASK JACOBIAN: ================================================================================================
MatrixXd fcn_lf1_Jv_dot(const VectorXd& eig_q, const VectorXd& eig_qd)
{

  double q[5];
  for(int i = 0; i<5; i++){
    q[i] = eig_q(i);
  }
  double qd[5];
  for(int i = 0; i<5; i++){
    qd[i] = eig_qd(i);
  }
  double p[4];
  p[0] = 0.2286;
  p[1] = 0.260;
  p[2] = 0.0576;
  p[3] = 0.060;

  double lf1_Jv_dot[15];

  double ab_lf1_Jv_dot_tmp_tmp;
  double b_lf1_Jv_dot_tmp;
  double b_lf1_Jv_dot_tmp_tmp;
  double b_lf1_Jv_dot_tmp_tmp_tmp;
  double bb_lf1_Jv_dot_tmp_tmp;
  double c_lf1_Jv_dot_tmp;
  double c_lf1_Jv_dot_tmp_tmp;
  double c_lf1_Jv_dot_tmp_tmp_tmp;
  double cb_lf1_Jv_dot_tmp_tmp;
  double d_lf1_Jv_dot_tmp;
  double d_lf1_Jv_dot_tmp_tmp;
  double d_lf1_Jv_dot_tmp_tmp_tmp;
  double db_lf1_Jv_dot_tmp_tmp;
  double e_lf1_Jv_dot_tmp;
  double e_lf1_Jv_dot_tmp_tmp;
  double e_lf1_Jv_dot_tmp_tmp_tmp;
  double eb_lf1_Jv_dot_tmp_tmp;
  double f_lf1_Jv_dot_tmp;
  double f_lf1_Jv_dot_tmp_tmp;
  double fb_lf1_Jv_dot_tmp_tmp;
  double g_lf1_Jv_dot_tmp;
  double g_lf1_Jv_dot_tmp_tmp;
  double gb_lf1_Jv_dot_tmp_tmp;
  double h_lf1_Jv_dot_tmp;
  double h_lf1_Jv_dot_tmp_tmp;
  double hb_lf1_Jv_dot_tmp_tmp;
  double i_lf1_Jv_dot_tmp;
  double i_lf1_Jv_dot_tmp_tmp;
  double ib_lf1_Jv_dot_tmp_tmp;
  double j_lf1_Jv_dot_tmp;
  double j_lf1_Jv_dot_tmp_tmp;
  double jb_lf1_Jv_dot_tmp_tmp;
  double k_lf1_Jv_dot_tmp_tmp;
  double kb_lf1_Jv_dot_tmp_tmp;
  double l_lf1_Jv_dot_tmp_tmp;
  double lf1_Jv_dot_tmp;
  double lf1_Jv_dot_tmp_tmp;
  double lf1_Jv_dot_tmp_tmp_tmp;
  double lf1_Jv_dot_tmp_tmp_tmp_tmp;
  double m_lf1_Jv_dot_tmp_tmp;
  double n_lf1_Jv_dot_tmp_tmp;
  double o_lf1_Jv_dot_tmp_tmp;
  double p_lf1_Jv_dot_tmp_tmp;
  double q_lf1_Jv_dot_tmp_tmp;
  double r_lf1_Jv_dot_tmp_tmp;
  double s_lf1_Jv_dot_tmp_tmp;
  double t_lf1_Jv_dot_tmp_tmp;
  double u_lf1_Jv_dot_tmp_tmp;
  double v_lf1_Jv_dot_tmp_tmp;
  double w_lf1_Jv_dot_tmp_tmp;
  double x_lf1_Jv_dot_tmp_tmp;
  double y_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot_tmp_tmp = std::cos(q[0]);
  b_lf1_Jv_dot_tmp_tmp = std::cos(q[2]);
  c_lf1_Jv_dot_tmp_tmp = std::sin(q[0]);
  d_lf1_Jv_dot_tmp_tmp = std::sin(q[1]);
  e_lf1_Jv_dot_tmp_tmp = std::sin(q[2]);
  f_lf1_Jv_dot_tmp_tmp = std::cos(q[3]);
  g_lf1_Jv_dot_tmp_tmp = std::sin(q[3]);
  h_lf1_Jv_dot_tmp_tmp = std::cos(q[4]);
  i_lf1_Jv_dot_tmp_tmp = std::sin(q[4]);
  j_lf1_Jv_dot_tmp_tmp = std::cos(q[1]);
  lf1_Jv_dot_tmp_tmp_tmp = lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  k_lf1_Jv_dot_tmp_tmp = c_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp -
                         lf1_Jv_dot_tmp_tmp_tmp * d_lf1_Jv_dot_tmp_tmp;
  b_lf1_Jv_dot_tmp_tmp_tmp = b_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp;
  l_lf1_Jv_dot_tmp_tmp = b_lf1_Jv_dot_tmp_tmp_tmp + lf1_Jv_dot_tmp_tmp *
                                                        d_lf1_Jv_dot_tmp_tmp *
                                                        e_lf1_Jv_dot_tmp_tmp;
  m_lf1_Jv_dot_tmp_tmp = qd[0] * lf1_Jv_dot_tmp_tmp;
  c_lf1_Jv_dot_tmp_tmp_tmp = qd[1] * lf1_Jv_dot_tmp_tmp;
  n_lf1_Jv_dot_tmp_tmp = c_lf1_Jv_dot_tmp_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  o_lf1_Jv_dot_tmp_tmp = qd[2] * lf1_Jv_dot_tmp_tmp;
  d_lf1_Jv_dot_tmp_tmp_tmp =
      qd[0] * b_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot_tmp_tmp_tmp_tmp = qd[2] * b_lf1_Jv_dot_tmp_tmp;
  e_lf1_Jv_dot_tmp_tmp_tmp = lf1_Jv_dot_tmp_tmp_tmp_tmp * c_lf1_Jv_dot_tmp_tmp;
  p_lf1_Jv_dot_tmp_tmp =
      (((m_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp +
         e_lf1_Jv_dot_tmp_tmp_tmp) -
        n_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp) +
       d_lf1_Jv_dot_tmp_tmp_tmp * d_lf1_Jv_dot_tmp_tmp) +
      o_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp;
  q_lf1_Jv_dot_tmp_tmp = qd[0] * c_lf1_Jv_dot_tmp_tmp;
  r_lf1_Jv_dot_tmp_tmp = o_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  s_lf1_Jv_dot_tmp_tmp = m_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  t_lf1_Jv_dot_tmp_tmp = qd[2] * c_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot_tmp =
      (((s_lf1_Jv_dot_tmp_tmp - t_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
        n_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
       r_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp) -
      q_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp;
  n_lf1_Jv_dot_tmp_tmp = p[1] * qd[3];
  u_lf1_Jv_dot_tmp_tmp = qd[4] * h_lf1_Jv_dot_tmp_tmp;
  v_lf1_Jv_dot_tmp_tmp = qd[4] * i_lf1_Jv_dot_tmp_tmp;
  w_lf1_Jv_dot_tmp_tmp = qd[3] * f_lf1_Jv_dot_tmp_tmp;
  x_lf1_Jv_dot_tmp_tmp = qd[3] * g_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot[0] =
      (((((p[2] *
               (((h_lf1_Jv_dot_tmp_tmp *
                      (((g_lf1_Jv_dot_tmp_tmp * p_lf1_Jv_dot_tmp_tmp -
                         f_lf1_Jv_dot_tmp_tmp *
                             ((((m_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp -
                                 qd[2] * c_lf1_Jv_dot_tmp_tmp *
                                     e_lf1_Jv_dot_tmp_tmp) +
                                qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                    e_lf1_Jv_dot_tmp_tmp) +
                               o_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                                   d_lf1_Jv_dot_tmp_tmp) -
                              qd[0] * c_lf1_Jv_dot_tmp_tmp *
                                  d_lf1_Jv_dot_tmp_tmp *
                                  e_lf1_Jv_dot_tmp_tmp)) +
                        w_lf1_Jv_dot_tmp_tmp * k_lf1_Jv_dot_tmp_tmp) +
                       x_lf1_Jv_dot_tmp_tmp * l_lf1_Jv_dot_tmp_tmp) +
                  i_lf1_Jv_dot_tmp_tmp *
                      (((f_lf1_Jv_dot_tmp_tmp * p_lf1_Jv_dot_tmp_tmp +
                         g_lf1_Jv_dot_tmp_tmp * lf1_Jv_dot_tmp) +
                        w_lf1_Jv_dot_tmp_tmp * l_lf1_Jv_dot_tmp_tmp) -
                       x_lf1_Jv_dot_tmp_tmp * k_lf1_Jv_dot_tmp_tmp)) +
                 u_lf1_Jv_dot_tmp_tmp *
                     (f_lf1_Jv_dot_tmp_tmp * k_lf1_Jv_dot_tmp_tmp +
                      g_lf1_Jv_dot_tmp_tmp * l_lf1_Jv_dot_tmp_tmp)) +
                v_lf1_Jv_dot_tmp_tmp *
                    (f_lf1_Jv_dot_tmp_tmp * l_lf1_Jv_dot_tmp_tmp -
                     g_lf1_Jv_dot_tmp_tmp * k_lf1_Jv_dot_tmp_tmp)) /
               2.0 +
           p[3] *
               (((h_lf1_Jv_dot_tmp_tmp *
                      (((std::cos(q[3]) *
                             ((((qd[0] * std::cos(q[0]) * std::sin(q[2]) +
                                 qd[2] * std::cos(q[2]) * std::sin(q[0])) -
                                qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                    std::cos(q[2])) +
                               qd[0] * std::cos(q[2]) * std::sin(q[0]) *
                                   std::sin(q[1])) +
                              qd[2] * std::cos(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2])) +
                         std::sin(q[3]) *
                             ((((qd[0] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[2] * std::sin(q[0]) * std::sin(q[2])) +
                                qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                    std::sin(q[2])) +
                               qd[2] * std::cos(q[0]) * std::cos(q[2]) *
                                   std::sin(q[1])) -
                              qd[0] * std::sin(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2]))) +
                        qd[3] * std::cos(q[3]) *
                            (std::cos(q[2]) * std::sin(q[0]) +
                             std::cos(q[0]) * std::sin(q[1]) *
                                 std::sin(q[2]))) -
                       qd[3] * std::sin(q[3]) *
                           (std::sin(q[0]) * std::sin(q[2]) -
                            std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]))) -
                  i_lf1_Jv_dot_tmp_tmp *
                      (((std::sin(q[3]) *
                             ((((qd[0] * std::cos(q[0]) * std::sin(q[2]) +
                                 qd[2] * std::cos(q[2]) * std::sin(q[0])) -
                                qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                    std::cos(q[2])) +
                               qd[0] * std::cos(q[2]) * std::sin(q[0]) *
                                   std::sin(q[1])) +
                              qd[2] * std::cos(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2])) -
                         std::cos(q[3]) *
                             ((((qd[0] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[2] * std::sin(q[0]) * std::sin(q[2])) +
                                qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                    std::sin(q[2])) +
                               qd[2] * std::cos(q[0]) * std::cos(q[2]) *
                                   std::sin(q[1])) -
                              qd[0] * std::sin(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2]))) +
                        qd[3] * std::cos(q[3]) *
                            (std::sin(q[0]) * std::sin(q[2]) -
                             std::cos(q[0]) * std::cos(q[2]) *
                                 std::sin(q[1]))) +
                       qd[3] * std::sin(q[3]) *
                           (std::cos(q[2]) * std::sin(q[0]) +
                            std::cos(q[0]) * std::sin(q[1]) *
                                std::sin(q[2])))) +
                 u_lf1_Jv_dot_tmp_tmp *
                     (std::cos(q[3]) *
                          (std::cos(q[2]) * std::sin(q[0]) +
                           std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                      std::sin(q[3]) *
                          (std::sin(q[0]) * std::sin(q[2]) -
                           std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])))) -
                v_lf1_Jv_dot_tmp_tmp *
                    (std::cos(q[3]) *
                         (std::sin(q[0]) * std::sin(q[2]) -
                          std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])) +
                     std::sin(q[3]) *
                         (std::cos(q[2]) * std::sin(q[0]) +
                          std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2]))))) +
          p[0] * p_lf1_Jv_dot_tmp_tmp) +
         p[1] * f_lf1_Jv_dot_tmp_tmp * p_lf1_Jv_dot_tmp_tmp) +
        p[1] * g_lf1_Jv_dot_tmp_tmp * lf1_Jv_dot_tmp) +
       n_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp * l_lf1_Jv_dot_tmp_tmp) -
      n_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp * k_lf1_Jv_dot_tmp_tmp;
  b_lf1_Jv_dot_tmp = m_lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  y_lf1_Jv_dot_tmp_tmp = qd[2] * j_lf1_Jv_dot_tmp_tmp;
  ab_lf1_Jv_dot_tmp_tmp = qd[3] * j_lf1_Jv_dot_tmp_tmp;
  c_lf1_Jv_dot_tmp = p[1] * qd[1];
  d_lf1_Jv_dot_tmp = qd[0] * p[1];
  e_lf1_Jv_dot_tmp = p[0] * qd[0];
  f_lf1_Jv_dot_tmp = p[0] * qd[1];
  g_lf1_Jv_dot_tmp = p[0] * qd[2];
  bb_lf1_Jv_dot_tmp_tmp = p[1] * qd[2];
  cb_lf1_Jv_dot_tmp_tmp = y_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  db_lf1_Jv_dot_tmp_tmp = ab_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  eb_lf1_Jv_dot_tmp_tmp = qd[1] * b_lf1_Jv_dot_tmp_tmp;
  fb_lf1_Jv_dot_tmp_tmp = j_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  gb_lf1_Jv_dot_tmp_tmp = bb_lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  hb_lf1_Jv_dot_tmp_tmp = n_lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot[3] =
      (((((((((((p[3] *
                     (((h_lf1_Jv_dot_tmp_tmp *
                            (((((((m_lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
                                       e_lf1_Jv_dot_tmp_tmp *
                                       g_lf1_Jv_dot_tmp_tmp -
                                   b_lf1_Jv_dot_tmp * b_lf1_Jv_dot_tmp_tmp *
                                       f_lf1_Jv_dot_tmp_tmp) +
                                  eb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                      c_lf1_Jv_dot_tmp_tmp *
                                      d_lf1_Jv_dot_tmp_tmp) +
                                 cb_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                     g_lf1_Jv_dot_tmp_tmp) +
                                y_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                    c_lf1_Jv_dot_tmp_tmp *
                                    e_lf1_Jv_dot_tmp_tmp) +
                               db_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                   g_lf1_Jv_dot_tmp_tmp) +
                              ab_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                  c_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) -
                             qd[1] * c_lf1_Jv_dot_tmp_tmp *
                                 d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                                 g_lf1_Jv_dot_tmp_tmp) +
                        i_lf1_Jv_dot_tmp_tmp *
                            (((((((qd[0] * std::cos(q[0]) * std::cos(q[1]) *
                                       std::cos(q[2]) * g_lf1_Jv_dot_tmp_tmp +
                                   b_lf1_Jv_dot_tmp * f_lf1_Jv_dot_tmp_tmp *
                                       e_lf1_Jv_dot_tmp_tmp) +
                                  cb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                      c_lf1_Jv_dot_tmp_tmp) +
                                 db_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                     c_lf1_Jv_dot_tmp_tmp) -
                                eb_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                    d_lf1_Jv_dot_tmp_tmp *
                                    g_lf1_Jv_dot_tmp_tmp) -
                               qd[1] * f_lf1_Jv_dot_tmp_tmp *
                                   c_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                                   e_lf1_Jv_dot_tmp_tmp) -
                              y_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                  e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) -
                             ab_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                 e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp)) +
                       u_lf1_Jv_dot_tmp_tmp *
                           (fb_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                g_lf1_Jv_dot_tmp_tmp +
                            j_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                c_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp)) +
                      v_lf1_Jv_dot_tmp_tmp *
                          (fb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                               c_lf1_Jv_dot_tmp_tmp -
                           j_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                               e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp)) -
                 p[2] *
                     (((h_lf1_Jv_dot_tmp_tmp *
                            (((((((qd[0] * std::cos(q[0]) * std::cos(q[1]) *
                                       std::cos(q[2]) * std::sin(q[3]) +
                                   qd[0] * std::cos(q[0]) * std::cos(q[1]) *
                                       std::cos(q[3]) * std::sin(q[2])) +
                                  qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                                      std::cos(q[3]) * std::sin(q[0])) +
                                 qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                                     std::cos(q[3]) * std::sin(q[0])) -
                                qd[1] * std::cos(q[2]) * std::sin(q[0]) *
                                    std::sin(q[1]) * std::sin(q[3])) -
                               qd[1] * std::cos(q[3]) * std::sin(q[0]) *
                                   std::sin(q[1]) * std::sin(q[2])) -
                              qd[2] * std::cos(q[1]) * std::sin(q[0]) *
                                  std::sin(q[2]) * std::sin(q[3])) -
                             qd[3] * std::cos(q[1]) * std::sin(q[0]) *
                                 std::sin(q[2]) * std::sin(q[3])) -
                        i_lf1_Jv_dot_tmp_tmp *
                            (((((((qd[0] * std::cos(q[0]) * std::cos(q[1]) *
                                       std::sin(q[2]) * std::sin(q[3]) -
                                   qd[0] * std::cos(q[0]) * std::cos(q[1]) *
                                       std::cos(q[2]) * std::cos(q[3])) +
                                  qd[1] * std::cos(q[2]) * std::cos(q[3]) *
                                      std::sin(q[0]) * std::sin(q[1])) +
                                 qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                                     std::sin(q[0]) * std::sin(q[3])) +
                                qd[2] * std::cos(q[1]) * std::cos(q[3]) *
                                    std::sin(q[0]) * std::sin(q[2])) +
                               qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                                   std::sin(q[0]) * std::sin(q[3])) +
                              qd[3] * std::cos(q[1]) * std::cos(q[3]) *
                                  std::sin(q[0]) * std::sin(q[2])) -
                             qd[1] * std::sin(q[0]) * std::sin(q[1]) *
                                 std::sin(q[2]) * std::sin(q[3]))) +
                       u_lf1_Jv_dot_tmp_tmp *
                           (std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) *
                                std::sin(q[0]) -
                            std::cos(q[1]) * std::sin(q[0]) * std::sin(q[2]) *
                                std::sin(q[3]))) -
                      v_lf1_Jv_dot_tmp_tmp *
                          (std::cos(q[1]) * std::cos(q[2]) * std::sin(q[0]) *
                               std::sin(q[3]) +
                           std::cos(q[1]) * std::cos(q[3]) * std::sin(q[0]) *
                               std::sin(q[2]))) /
                     2.0) -
                e_lf1_Jv_dot_tmp * lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
                    b_lf1_Jv_dot_tmp_tmp) +
               f_lf1_Jv_dot_tmp * b_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                   d_lf1_Jv_dot_tmp_tmp) +
              g_lf1_Jv_dot_tmp * j_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                  e_lf1_Jv_dot_tmp_tmp) -
             d_lf1_Jv_dot_tmp * lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
                 b_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp) +
            qd[0] * p[1] * std::cos(q[0]) * std::cos(q[1]) *
                e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) +
           c_lf1_Jv_dot_tmp * b_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
               c_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp) +
          gb_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
              g_lf1_Jv_dot_tmp_tmp) +
         gb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
             e_lf1_Jv_dot_tmp_tmp) +
        hb_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
            g_lf1_Jv_dot_tmp_tmp) +
       hb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
           e_lf1_Jv_dot_tmp_tmp) -
      c_lf1_Jv_dot_tmp * c_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
          e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp;
  ib_lf1_Jv_dot_tmp_tmp = lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp +
                          b_lf1_Jv_dot_tmp_tmp_tmp * d_lf1_Jv_dot_tmp_tmp;
  jb_lf1_Jv_dot_tmp_tmp = lf1_Jv_dot_tmp_tmp_tmp - c_lf1_Jv_dot_tmp_tmp *
                                                       d_lf1_Jv_dot_tmp_tmp *
                                                       e_lf1_Jv_dot_tmp_tmp;
  kb_lf1_Jv_dot_tmp_tmp = qd[1] * j_lf1_Jv_dot_tmp_tmp;
  b_lf1_Jv_dot_tmp =
      (((d_lf1_Jv_dot_tmp_tmp_tmp +
         o_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
        m_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
       kb_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
      e_lf1_Jv_dot_tmp_tmp_tmp * d_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot_tmp_tmp_tmp = kb_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp;
  d_lf1_Jv_dot_tmp_tmp_tmp =
      (((r_lf1_Jv_dot_tmp_tmp - q_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
        s_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp) +
       lf1_Jv_dot_tmp_tmp_tmp * c_lf1_Jv_dot_tmp_tmp) -
      t_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp;
  q_lf1_Jv_dot_tmp_tmp = p[1] * std::cos(q[3]);
  e_lf1_Jv_dot_tmp_tmp_tmp = q_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp;
  r_lf1_Jv_dot_tmp_tmp = p[1] * std::sin(q[3]);
  h_lf1_Jv_dot_tmp = r_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp_tmp;
  s_lf1_Jv_dot_tmp_tmp = p[1] * qd[3] * std::cos(q[3]);
  i_lf1_Jv_dot_tmp = s_lf1_Jv_dot_tmp_tmp * ib_lf1_Jv_dot_tmp_tmp;
  t_lf1_Jv_dot_tmp_tmp = p[1] * qd[3] * std::sin(q[3]);
  j_lf1_Jv_dot_tmp = t_lf1_Jv_dot_tmp_tmp * jb_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot[6] =
      (((((p[3] *
               (((h_lf1_Jv_dot_tmp_tmp *
                      (((f_lf1_Jv_dot_tmp_tmp *
                             ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                 o_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
                                m_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                                    e_lf1_Jv_dot_tmp_tmp) +
                               qd[1] * j_lf1_Jv_dot_tmp_tmp *
                                   c_lf1_Jv_dot_tmp_tmp *
                                   e_lf1_Jv_dot_tmp_tmp) +
                              qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                  d_lf1_Jv_dot_tmp_tmp) +
                         g_lf1_Jv_dot_tmp_tmp *
                             ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[0] * std::sin(q[0]) *
                                     e_lf1_Jv_dot_tmp_tmp) +
                                qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                    d_lf1_Jv_dot_tmp_tmp) +
                               kb_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                                   c_lf1_Jv_dot_tmp_tmp) -
                              qd[2] * std::sin(q[0]) * d_lf1_Jv_dot_tmp_tmp *
                                  e_lf1_Jv_dot_tmp_tmp)) +
                        w_lf1_Jv_dot_tmp_tmp * ib_lf1_Jv_dot_tmp_tmp) +
                       x_lf1_Jv_dot_tmp_tmp * jb_lf1_Jv_dot_tmp_tmp) +
                  i_lf1_Jv_dot_tmp_tmp *
                      (((f_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp_tmp -
                         g_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp) +
                        w_lf1_Jv_dot_tmp_tmp * jb_lf1_Jv_dot_tmp_tmp) -
                       x_lf1_Jv_dot_tmp_tmp * ib_lf1_Jv_dot_tmp_tmp)) +
                 u_lf1_Jv_dot_tmp_tmp *
                     (f_lf1_Jv_dot_tmp_tmp * ib_lf1_Jv_dot_tmp_tmp +
                      g_lf1_Jv_dot_tmp_tmp * jb_lf1_Jv_dot_tmp_tmp)) +
                v_lf1_Jv_dot_tmp_tmp *
                    (f_lf1_Jv_dot_tmp_tmp * jb_lf1_Jv_dot_tmp_tmp -
                     g_lf1_Jv_dot_tmp_tmp * ib_lf1_Jv_dot_tmp_tmp)) -
           p[2] *
               (((h_lf1_Jv_dot_tmp_tmp *
                      (((std::cos(q[3]) *
                             ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                    std::sin(q[1])) +
                               qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                   std::sin(q[0])) -
                              qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2])) -
                         std::sin(q[3]) *
                             ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                 qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                    std::sin(q[2])) +
                               qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                   std::sin(q[2])) +
                              qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                  std::sin(q[1]))) +
                        qd[3] * std::cos(q[3]) *
                            (std::cos(q[0]) * std::cos(q[2]) -
                             std::sin(q[0]) * std::sin(q[1]) *
                                 std::sin(q[2]))) -
                       qd[3] * std::sin(q[3]) *
                           (std::cos(q[0]) * std::sin(q[2]) +
                            std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]))) -
                  i_lf1_Jv_dot_tmp_tmp *
                      (((std::cos(q[3]) *
                             ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                 qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                    std::sin(q[2])) +
                               qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                   std::sin(q[2])) +
                              qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                  std::sin(q[1])) +
                         std::sin(q[3]) *
                             ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                    std::sin(q[1])) +
                               qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                   std::sin(q[0])) -
                              qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2]))) +
                        qd[3] * std::cos(q[3]) *
                            (std::cos(q[0]) * std::sin(q[2]) +
                             std::cos(q[2]) * std::sin(q[0]) *
                                 std::sin(q[1]))) +
                       qd[3] * std::sin(q[3]) *
                           (std::cos(q[0]) * std::cos(q[2]) -
                            std::sin(q[0]) * std::sin(q[1]) *
                                std::sin(q[2])))) +
                 u_lf1_Jv_dot_tmp_tmp *
                     (std::cos(q[3]) *
                          (std::cos(q[0]) * std::cos(q[2]) -
                           std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                      std::sin(q[3]) *
                          (std::cos(q[0]) * std::sin(q[2]) +
                           std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])))) -
                v_lf1_Jv_dot_tmp_tmp *
                    (std::cos(q[3]) *
                         (std::cos(q[0]) * std::sin(q[2]) +
                          std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])) +
                     std::sin(q[3]) *
                         (std::cos(q[0]) * std::cos(q[2]) -
                          std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])))) /
               2.0) +
          p[0] * b_lf1_Jv_dot_tmp) +
         e_lf1_Jv_dot_tmp_tmp_tmp) +
        h_lf1_Jv_dot_tmp) +
       i_lf1_Jv_dot_tmp) +
      j_lf1_Jv_dot_tmp;
  b_lf1_Jv_dot_tmp_tmp_tmp =
      p[3] * (((std::cos(q[4]) *
                    (((std::cos(q[3]) *
                           ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                               qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                              qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2])) +
                             qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                 std::sin(q[2])) +
                            qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                std::sin(q[1])) +
                       std::sin(q[3]) *
                           ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                               qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                              qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                  std::sin(q[1])) +
                             qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                 std::sin(q[0])) -
                            qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                std::sin(q[2]))) +
                      qd[3] * std::cos(q[3]) *
                          (std::cos(q[0]) * std::sin(q[2]) +
                           std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]))) +
                     qd[3] * std::sin(q[3]) *
                         (std::cos(q[0]) * std::cos(q[2]) -
                          std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]))) +
                std::sin(q[4]) *
                    (((std::cos(q[3]) *
                           ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                               qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                              qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                  std::sin(q[1])) +
                             qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                 std::sin(q[0])) -
                            qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                std::sin(q[2])) -
                       std::sin(q[3]) *
                           ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                               qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                              qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2])) +
                             qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                 std::sin(q[2])) +
                            qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                std::sin(q[1]))) +
                      qd[3] * std::cos(q[3]) *
                          (std::cos(q[0]) * std::cos(q[2]) -
                           std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]))) -
                     qd[3] * std::sin(q[3]) *
                         (std::cos(q[0]) * std::sin(q[2]) +
                          std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])))) +
               qd[4] * std::cos(q[4]) *
                   (std::cos(q[3]) *
                        (std::cos(q[0]) * std::sin(q[2]) +
                         std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])) +
                    std::sin(q[3]) *
                        (std::cos(q[0]) * std::cos(q[2]) -
                         std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])))) +
              qd[4] * std::sin(q[4]) *
                  (std::cos(q[3]) *
                       (std::cos(q[0]) * std::cos(q[2]) -
                        std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                   std::sin(q[3]) *
                       (std::cos(q[0]) * std::sin(q[2]) +
                        std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])))) -
      p[2] *
          (((std::cos(q[4]) *
                 (((std::cos(q[3]) *
                        ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                            qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                           qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                               std::sin(q[1])) +
                          qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                              std::sin(q[0])) -
                         qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                             std::sin(q[2])) -
                    std::sin(q[3]) *
                        ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                            qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                           qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                               std::sin(q[2])) +
                          qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                              std::sin(q[2])) +
                         qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                             std::sin(q[1]))) +
                   qd[3] * std::cos(q[3]) *
                       (std::cos(q[0]) * std::cos(q[2]) -
                        std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]))) -
                  qd[3] * std::sin(q[3]) *
                      (std::cos(q[0]) * std::sin(q[2]) +
                       std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]))) -
             std::sin(q[4]) *
                 (((std::cos(q[3]) *
                        ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                            qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                           qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                               std::sin(q[2])) +
                          qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                              std::sin(q[2])) +
                         qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                             std::sin(q[1])) +
                    std::sin(q[3]) *
                        ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                            qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                           qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                               std::sin(q[1])) +
                          qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                              std::sin(q[0])) -
                         qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                             std::sin(q[2]))) +
                   qd[3] * std::cos(q[3]) *
                       (std::cos(q[0]) * std::sin(q[2]) +
                        std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]))) +
                  qd[3] * std::sin(q[3]) *
                      (std::cos(q[0]) * std::cos(q[2]) -
                       std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])))) +
            qd[4] * std::cos(q[4]) *
                (std::cos(q[3]) *
                     (std::cos(q[0]) * std::cos(q[2]) -
                      std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                 std::sin(q[3]) *
                     (std::cos(q[0]) * std::sin(q[2]) +
                      std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])))) -
           qd[4] * std::sin(q[4]) *
               (std::cos(q[3]) *
                    (std::cos(q[0]) * std::sin(q[2]) +
                     std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])) +
                std::sin(q[3]) *
                    (std::cos(q[0]) * std::cos(q[2]) -
                     std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])))) /
          2.0;
  lf1_Jv_dot[9] = (((b_lf1_Jv_dot_tmp_tmp_tmp + e_lf1_Jv_dot_tmp_tmp_tmp) +
                    h_lf1_Jv_dot_tmp) +
                   i_lf1_Jv_dot_tmp) +
                  j_lf1_Jv_dot_tmp;
  lf1_Jv_dot[12] = b_lf1_Jv_dot_tmp_tmp_tmp;
  lf1_Jv_dot[1] =
      (((((r_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp -
           p[2] *
               (((std::cos(q[4]) *
                      (((std::cos(q[3]) *
                             ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                 qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                    std::sin(q[2])) +
                               qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                   std::sin(q[2])) +
                              qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                  std::sin(q[1])) +
                         std::sin(q[3]) *
                             ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                    std::sin(q[1])) +
                               qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                   std::sin(q[0])) -
                              qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2]))) +
                        qd[3] * std::cos(q[3]) *
                            (std::cos(q[0]) * std::sin(q[2]) +
                             std::cos(q[2]) * std::sin(q[0]) *
                                 std::sin(q[1]))) +
                       qd[3] * std::sin(q[3]) *
                           (std::cos(q[0]) * std::cos(q[2]) -
                            std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]))) +
                  std::sin(q[4]) *
                      (((std::cos(q[3]) *
                             ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                 qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                    std::sin(q[1])) +
                               qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                   std::sin(q[0])) -
                              qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                  std::sin(q[2])) -
                         std::sin(q[3]) *
                             ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                 qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                                qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                    std::sin(q[2])) +
                               qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                   std::sin(q[2])) +
                              qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                  std::sin(q[1]))) +
                        qd[3] * std::cos(q[3]) *
                            (std::cos(q[0]) * std::cos(q[2]) -
                             std::sin(q[0]) * std::sin(q[1]) *
                                 std::sin(q[2]))) -
                       qd[3] * std::sin(q[3]) *
                           (std::cos(q[0]) * std::sin(q[2]) +
                            std::cos(q[2]) * std::sin(q[0]) *
                                std::sin(q[1])))) +
                 qd[4] * std::cos(q[4]) *
                     (std::cos(q[3]) *
                          (std::cos(q[0]) * std::sin(q[2]) +
                           std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])) +
                      std::sin(q[3]) *
                          (std::cos(q[0]) * std::cos(q[2]) -
                           std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])))) +
                qd[4] * std::sin(q[4]) *
                    (std::cos(q[3]) *
                         (std::cos(q[0]) * std::cos(q[2]) -
                          std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                     std::sin(q[3]) *
                         (std::cos(q[0]) * std::sin(q[2]) +
                          std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])))) /
               2.0) -
          p[3] *
              (((std::cos(q[4]) *
                     (((std::cos(q[3]) *
                            ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                               qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                   std::sin(q[1])) +
                              qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                  std::sin(q[0])) -
                             qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                 std::sin(q[2])) -
                        std::sin(q[3]) *
                            ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                               qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                   std::sin(q[2])) +
                              qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                  std::sin(q[2])) +
                             qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                 std::sin(q[1]))) +
                       qd[3] * std::cos(q[3]) *
                           (std::cos(q[0]) * std::cos(q[2]) -
                            std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]))) -
                      qd[3] * std::sin(q[3]) *
                          (std::cos(q[0]) * std::sin(q[2]) +
                           std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]))) -
                 std::sin(q[4]) *
                     (((std::cos(q[3]) *
                            ((((qd[0] * std::cos(q[2]) * std::sin(q[0]) +
                                qd[2] * std::cos(q[0]) * std::sin(q[2])) +
                               qd[0] * std::cos(q[0]) * std::sin(q[1]) *
                                   std::sin(q[2])) +
                              qd[1] * std::cos(q[1]) * std::sin(q[0]) *
                                  std::sin(q[2])) +
                             qd[2] * std::cos(q[2]) * std::sin(q[0]) *
                                 std::sin(q[1])) +
                        std::sin(q[3]) *
                            ((((qd[2] * std::cos(q[0]) * std::cos(q[2]) -
                                qd[0] * std::sin(q[0]) * std::sin(q[2])) +
                               qd[0] * std::cos(q[0]) * std::cos(q[2]) *
                                   std::sin(q[1])) +
                              qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                  std::sin(q[0])) -
                             qd[2] * std::sin(q[0]) * std::sin(q[1]) *
                                 std::sin(q[2]))) +
                       qd[3] * std::cos(q[3]) *
                           (std::cos(q[0]) * std::sin(q[2]) +
                            std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1]))) +
                      qd[3] * std::sin(q[3]) *
                          (std::cos(q[0]) * std::cos(q[2]) -
                           std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])))) +
                qd[4] * std::cos(q[4]) *
                    (std::cos(q[3]) *
                         (std::cos(q[0]) * std::cos(q[2]) -
                          std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                     std::sin(q[3]) *
                         (std::cos(q[0]) * std::sin(q[2]) +
                          std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])))) -
               qd[4] * std::sin(q[4]) *
                   (std::cos(q[3]) *
                        (std::cos(q[0]) * std::sin(q[2]) +
                         std::cos(q[2]) * std::sin(q[0]) * std::sin(q[1])) +
                    std::sin(q[3]) *
                        (std::cos(q[0]) * std::cos(q[2]) -
                         std::sin(q[0]) * std::sin(q[1]) * std::sin(q[2]))))) -
         q_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp_tmp) -
        p[0] * d_lf1_Jv_dot_tmp_tmp_tmp) -
       s_lf1_Jv_dot_tmp_tmp * jb_lf1_Jv_dot_tmp_tmp) +
      t_lf1_Jv_dot_tmp_tmp * ib_lf1_Jv_dot_tmp_tmp;
  m_lf1_Jv_dot_tmp_tmp = qd[0] * j_lf1_Jv_dot_tmp_tmp;
  b_lf1_Jv_dot_tmp = o_lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  x_lf1_Jv_dot_tmp_tmp = qd[3] * lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  ib_lf1_Jv_dot_tmp_tmp = lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot[4] =
      (((((((((((d_lf1_Jv_dot_tmp * j_lf1_Jv_dot_tmp_tmp *
                     c_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                     g_lf1_Jv_dot_tmp_tmp -
                 p[3] *
                     (((h_lf1_Jv_dot_tmp_tmp *
                            (((((((m_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                                       f_lf1_Jv_dot_tmp_tmp *
                                       c_lf1_Jv_dot_tmp_tmp +
                                   c_lf1_Jv_dot_tmp_tmp_tmp *
                                       b_lf1_Jv_dot_tmp_tmp *
                                       f_lf1_Jv_dot_tmp_tmp *
                                       d_lf1_Jv_dot_tmp_tmp) +
                                  o_lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
                                      b_lf1_Jv_dot_tmp_tmp *
                                      g_lf1_Jv_dot_tmp_tmp) +
                                 b_lf1_Jv_dot_tmp * f_lf1_Jv_dot_tmp_tmp *
                                     e_lf1_Jv_dot_tmp_tmp) +
                                x_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                                    g_lf1_Jv_dot_tmp_tmp) +
                               x_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                   e_lf1_Jv_dot_tmp_tmp) -
                              m_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp *
                                  e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) -
                             c_lf1_Jv_dot_tmp_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                                 e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) -
                        i_lf1_Jv_dot_tmp_tmp *
                            (((((((qd[0] * std::cos(q[1]) * std::cos(q[2]) *
                                       c_lf1_Jv_dot_tmp_tmp *
                                       g_lf1_Jv_dot_tmp_tmp -
                                   qd[3] * std::cos(q[0]) * std::cos(q[1]) *
                                       std::cos(q[2]) * f_lf1_Jv_dot_tmp_tmp) -
                                  qd[2] * std::cos(q[0]) * std::cos(q[1]) *
                                      std::cos(q[2]) * f_lf1_Jv_dot_tmp_tmp) +
                                 m_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                     c_lf1_Jv_dot_tmp_tmp *
                                     e_lf1_Jv_dot_tmp_tmp) +
                                qd[1] * std::cos(q[0]) * std::cos(q[2]) *
                                    d_lf1_Jv_dot_tmp_tmp *
                                    g_lf1_Jv_dot_tmp_tmp) +
                               c_lf1_Jv_dot_tmp_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                   d_lf1_Jv_dot_tmp_tmp *
                                   e_lf1_Jv_dot_tmp_tmp) +
                              b_lf1_Jv_dot_tmp * e_lf1_Jv_dot_tmp_tmp *
                                  g_lf1_Jv_dot_tmp_tmp) +
                             x_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                                 g_lf1_Jv_dot_tmp_tmp)) +
                       u_lf1_Jv_dot_tmp_tmp *
                           (ib_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                                g_lf1_Jv_dot_tmp_tmp +
                            ib_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                e_lf1_Jv_dot_tmp_tmp)) +
                      v_lf1_Jv_dot_tmp_tmp *
                          (std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) *
                               f_lf1_Jv_dot_tmp_tmp -
                           ib_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                               g_lf1_Jv_dot_tmp_tmp))) -
                e_lf1_Jv_dot_tmp * j_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                    c_lf1_Jv_dot_tmp_tmp) -
               f_lf1_Jv_dot_tmp * lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                   d_lf1_Jv_dot_tmp_tmp) -
              g_lf1_Jv_dot_tmp * lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
                  e_lf1_Jv_dot_tmp_tmp) -
             qd[0] * p[1] * std::cos(q[1]) * b_lf1_Jv_dot_tmp_tmp *
                 f_lf1_Jv_dot_tmp_tmp * c_lf1_Jv_dot_tmp_tmp) -
            c_lf1_Jv_dot_tmp * lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
                f_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp) -
           bb_lf1_Jv_dot_tmp_tmp * lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
               b_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) -
          p[1] * qd[2] * std::cos(q[0]) * std::cos(q[1]) *
              f_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) -
         n_lf1_Jv_dot_tmp_tmp * lf1_Jv_dot_tmp_tmp * j_lf1_Jv_dot_tmp_tmp *
             b_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) -
        p[1] * qd[3] * std::cos(q[0]) * std::cos(q[1]) * f_lf1_Jv_dot_tmp_tmp *
            e_lf1_Jv_dot_tmp_tmp) -
       p[2] *
           (((h_lf1_Jv_dot_tmp_tmp *
                  (((((((qd[0] * std::cos(q[1]) * std::cos(q[2]) *
                             std::sin(q[0]) * std::sin(q[3]) -
                         qd[3] * std::cos(q[0]) * std::cos(q[1]) *
                             std::cos(q[2]) * std::cos(q[3])) -
                        qd[2] * std::cos(q[0]) * std::cos(q[1]) *
                            std::cos(q[2]) * std::cos(q[3])) +
                       qd[0] * std::cos(q[1]) * std::cos(q[3]) *
                           std::sin(q[0]) * std::sin(q[2])) +
                      qd[1] * std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]) *
                          std::sin(q[3])) +
                     qd[1] * std::cos(q[0]) * std::cos(q[3]) * std::sin(q[1]) *
                         std::sin(q[2])) +
                    qd[2] * std::cos(q[0]) * std::cos(q[1]) * std::sin(q[2]) *
                        std::sin(q[3])) +
                   qd[3] * std::cos(q[0]) * std::cos(q[1]) * std::sin(q[2]) *
                       std::sin(q[3])) +
              i_lf1_Jv_dot_tmp_tmp *
                  (((((((qd[0] * std::cos(q[1]) * std::cos(q[2]) *
                             std::cos(q[3]) * std::sin(q[0]) +
                         qd[1] * std::cos(q[0]) * std::cos(q[2]) *
                             std::cos(q[3]) * std::sin(q[1])) +
                        qd[2] * std::cos(q[0]) * std::cos(q[1]) *
                            std::cos(q[2]) * std::sin(q[3])) +
                       qd[2] * std::cos(q[0]) * std::cos(q[1]) *
                           std::cos(q[3]) * std::sin(q[2])) +
                      qd[3] * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[2]) *
                          std::sin(q[3])) +
                     qd[3] * std::cos(q[0]) * std::cos(q[1]) * std::cos(q[3]) *
                         std::sin(q[2])) -
                    qd[0] * std::cos(q[1]) * std::sin(q[0]) * std::sin(q[2]) *
                        std::sin(q[3])) -
                   qd[1] * std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2]) *
                       std::sin(q[3]))) -
             u_lf1_Jv_dot_tmp_tmp * (std::cos(q[0]) * std::cos(q[1]) *
                                         std::cos(q[2]) * std::cos(q[3]) -
                                     std::cos(q[0]) * std::cos(q[1]) *
                                         std::sin(q[2]) * std::sin(q[3]))) +
            v_lf1_Jv_dot_tmp_tmp * (std::cos(q[0]) * std::cos(q[1]) *
                                        std::cos(q[2]) * std::sin(q[3]) +
                                    std::cos(q[0]) * std::cos(q[1]) *
                                        std::cos(q[3]) * std::sin(q[2]))) /
           2.0) +
      p[1] * qd[1] * std::cos(q[0]) * d_lf1_Jv_dot_tmp_tmp *
          e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp;
  b_lf1_Jv_dot_tmp =
      p[3] * (((std::cos(q[4]) *
                    (((std::sin(q[3]) *
                           ((((qd[0] * std::cos(q[0]) * std::sin(q[2]) +
                               qd[2] * std::cos(q[2]) * std::sin(q[0])) -
                              qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                  std::cos(q[2])) +
                             qd[0] * std::cos(q[2]) * std::sin(q[0]) *
                                 std::sin(q[1])) +
                            qd[2] * std::cos(q[0]) * std::sin(q[1]) *
                                std::sin(q[2])) -
                       std::cos(q[3]) *
                           ((((qd[0] * std::cos(q[0]) * std::cos(q[2]) -
                               qd[2] * std::sin(q[0]) * std::sin(q[2])) +
                              qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                  std::sin(q[2])) +
                             qd[2] * std::cos(q[0]) * std::cos(q[2]) *
                                 std::sin(q[1])) -
                            qd[0] * std::sin(q[0]) * std::sin(q[1]) *
                                std::sin(q[2]))) +
                      qd[3] * std::cos(q[3]) *
                          (std::sin(q[0]) * std::sin(q[2]) -
                           std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]))) +
                     qd[3] * std::sin(q[3]) *
                         (std::cos(q[2]) * std::sin(q[0]) +
                          std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2]))) +
                std::sin(q[4]) *
                    (((std::cos(q[3]) *
                           ((((qd[0] * std::cos(q[0]) * std::sin(q[2]) +
                               qd[2] * std::cos(q[2]) * std::sin(q[0])) -
                              qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                  std::cos(q[2])) +
                             qd[0] * std::cos(q[2]) * std::sin(q[0]) *
                                 std::sin(q[1])) +
                            qd[2] * std::cos(q[0]) * std::sin(q[1]) *
                                std::sin(q[2])) +
                       std::sin(q[3]) *
                           ((((qd[0] * std::cos(q[0]) * std::cos(q[2]) -
                               qd[2] * std::sin(q[0]) * std::sin(q[2])) +
                              qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                                  std::sin(q[2])) +
                             qd[2] * std::cos(q[0]) * std::cos(q[2]) *
                                 std::sin(q[1])) -
                            qd[0] * std::sin(q[0]) * std::sin(q[1]) *
                                std::sin(q[2]))) +
                      qd[3] * std::cos(q[3]) *
                          (std::cos(q[2]) * std::sin(q[0]) +
                           std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2]))) -
                     qd[3] * std::sin(q[3]) *
                         (std::sin(q[0]) * std::sin(q[2]) -
                          std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])))) +
               qd[4] * std::cos(q[4]) *
                   (std::cos(q[3]) *
                        (std::sin(q[0]) * std::sin(q[2]) -
                         std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])) +
                    std::sin(q[3]) *
                        (std::cos(q[2]) * std::sin(q[0]) +
                         std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])))) +
              qd[4] * std::sin(q[4]) *
                  (std::cos(q[3]) *
                       (std::cos(q[2]) * std::sin(q[0]) +
                        std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                   std::sin(q[3]) *
                       (std::sin(q[0]) * std::sin(q[2]) -
                        std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])))) -
      p[2] *
          (((std::cos(q[4]) *
                 (((std::cos(q[3]) *
                        ((((qd[0] * std::cos(q[0]) * std::sin(q[2]) +
                            qd[2] * std::cos(q[2]) * std::sin(q[0])) -
                           qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                               std::cos(q[2])) +
                          qd[0] * std::cos(q[2]) * std::sin(q[0]) *
                              std::sin(q[1])) +
                         qd[2] * std::cos(q[0]) * std::sin(q[1]) *
                             std::sin(q[2])) +
                    std::sin(q[3]) *
                        ((((qd[0] * std::cos(q[0]) * std::cos(q[2]) -
                            qd[2] * std::sin(q[0]) * std::sin(q[2])) +
                           qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                               std::sin(q[2])) +
                          qd[2] * std::cos(q[0]) * std::cos(q[2]) *
                              std::sin(q[1])) -
                         qd[0] * std::sin(q[0]) * std::sin(q[1]) *
                             std::sin(q[2]))) +
                   qd[3] * std::cos(q[3]) *
                       (std::cos(q[2]) * std::sin(q[0]) +
                        std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2]))) -
                  qd[3] * std::sin(q[3]) *
                      (std::sin(q[0]) * std::sin(q[2]) -
                       std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]))) -
             std::sin(q[4]) *
                 (((std::sin(q[3]) *
                        ((((qd[0] * std::cos(q[0]) * std::sin(q[2]) +
                            qd[2] * std::cos(q[2]) * std::sin(q[0])) -
                           qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                               std::cos(q[2])) +
                          qd[0] * std::cos(q[2]) * std::sin(q[0]) *
                              std::sin(q[1])) +
                         qd[2] * std::cos(q[0]) * std::sin(q[1]) *
                             std::sin(q[2])) -
                    std::cos(q[3]) *
                        ((((qd[0] * std::cos(q[0]) * std::cos(q[2]) -
                            qd[2] * std::sin(q[0]) * std::sin(q[2])) +
                           qd[1] * std::cos(q[0]) * std::cos(q[1]) *
                               std::sin(q[2])) +
                          qd[2] * std::cos(q[0]) * std::cos(q[2]) *
                              std::sin(q[1])) -
                         qd[0] * std::sin(q[0]) * std::sin(q[1]) *
                             std::sin(q[2]))) +
                   qd[3] * std::cos(q[3]) *
                       (std::sin(q[0]) * std::sin(q[2]) -
                        std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1]))) +
                  qd[3] * std::sin(q[3]) *
                      (std::cos(q[2]) * std::sin(q[0]) +
                       std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])))) +
            qd[4] * std::cos(q[4]) *
                (std::cos(q[3]) *
                     (std::cos(q[2]) * std::sin(q[0]) +
                      std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])) -
                 std::sin(q[3]) *
                     (std::sin(q[0]) * std::sin(q[2]) -
                      std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])))) -
           qd[4] * std::sin(q[4]) *
               (std::cos(q[3]) *
                    (std::sin(q[0]) * std::sin(q[2]) -
                     std::cos(q[0]) * std::cos(q[2]) * std::sin(q[1])) +
                std::sin(q[3]) *
                    (std::cos(q[2]) * std::sin(q[0]) +
                     std::cos(q[0]) * std::sin(q[1]) * std::sin(q[2])))) /
          2.0;
  d_lf1_Jv_dot_tmp = r_lf1_Jv_dot_tmp_tmp * p_lf1_Jv_dot_tmp_tmp;
  e_lf1_Jv_dot_tmp = q_lf1_Jv_dot_tmp_tmp * lf1_Jv_dot_tmp;
  d_lf1_Jv_dot_tmp_tmp_tmp = s_lf1_Jv_dot_tmp_tmp * k_lf1_Jv_dot_tmp_tmp;
  e_lf1_Jv_dot_tmp_tmp_tmp = t_lf1_Jv_dot_tmp_tmp * l_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot[7] =
      ((((b_lf1_Jv_dot_tmp - p[0] * lf1_Jv_dot_tmp) + d_lf1_Jv_dot_tmp) -
        e_lf1_Jv_dot_tmp) +
       d_lf1_Jv_dot_tmp_tmp_tmp) +
      e_lf1_Jv_dot_tmp_tmp_tmp;
  lf1_Jv_dot[10] = (((b_lf1_Jv_dot_tmp + d_lf1_Jv_dot_tmp) - e_lf1_Jv_dot_tmp) +
                    d_lf1_Jv_dot_tmp_tmp_tmp) +
                   e_lf1_Jv_dot_tmp_tmp_tmp;
  lf1_Jv_dot[13] = b_lf1_Jv_dot_tmp;
  lf1_Jv_dot[2] = 0.0;
  lf1_Jv_dot[5] =
      ((((((((f_lf1_Jv_dot_tmp * j_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp -
              p[3] *
                  (((h_lf1_Jv_dot_tmp_tmp *
                         (((((kb_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                                  g_lf1_Jv_dot_tmp_tmp -
                              lf1_Jv_dot_tmp_tmp_tmp * f_lf1_Jv_dot_tmp_tmp) +
                             lf1_Jv_dot_tmp_tmp_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                                 g_lf1_Jv_dot_tmp_tmp) +
                            qd[2] * f_lf1_Jv_dot_tmp_tmp *
                                d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
                           qd[3] * b_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                               g_lf1_Jv_dot_tmp_tmp) +
                          w_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                              e_lf1_Jv_dot_tmp_tmp) +
                     i_lf1_Jv_dot_tmp_tmp *
                         (((((lf1_Jv_dot_tmp_tmp_tmp * g_lf1_Jv_dot_tmp_tmp +
                              kb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                  e_lf1_Jv_dot_tmp_tmp) +
                             lf1_Jv_dot_tmp_tmp_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                                 d_lf1_Jv_dot_tmp_tmp) +
                            qd[3] * std::cos(q[2]) * f_lf1_Jv_dot_tmp_tmp *
                                d_lf1_Jv_dot_tmp_tmp) -
                           qd[2] * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                               g_lf1_Jv_dot_tmp_tmp) -
                          qd[3] * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                              g_lf1_Jv_dot_tmp_tmp)) -
                    v_lf1_Jv_dot_tmp_tmp *
                        (d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                             g_lf1_Jv_dot_tmp_tmp -
                         b_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp *
                             d_lf1_Jv_dot_tmp_tmp)) +
                   u_lf1_Jv_dot_tmp_tmp *
                       (b_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                            g_lf1_Jv_dot_tmp_tmp +
                        f_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                            e_lf1_Jv_dot_tmp_tmp))) -
             g_lf1_Jv_dot_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) -
            p[2] *
                (((i_lf1_Jv_dot_tmp_tmp *
                       (((((qd[1] * std::cos(q[1]) * std::sin(q[2]) *
                                std::sin(q[3]) -
                            qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                std::cos(q[3])) +
                           qd[2] * std::cos(q[2]) * std::sin(q[1]) *
                               std::sin(q[3])) +
                          qd[2] * std::cos(q[3]) * std::sin(q[1]) *
                              std::sin(q[2])) +
                         qd[3] * std::cos(q[2]) * std::sin(q[1]) *
                             std::sin(q[3])) +
                        qd[3] * std::cos(q[3]) * std::sin(q[1]) *
                            std::sin(q[2])) -
                   h_lf1_Jv_dot_tmp_tmp *
                       (((((qd[1] * std::cos(q[1]) * std::cos(q[2]) *
                                std::sin(q[3]) +
                            qd[1] * std::cos(q[1]) * std::cos(q[3]) *
                                std::sin(q[2])) +
                           qd[2] * std::cos(q[2]) * std::cos(q[3]) *
                               std::sin(q[1])) +
                          qd[3] * std::cos(q[2]) * std::cos(q[3]) *
                              std::sin(q[1])) -
                         qd[2] * std::sin(q[1]) * std::sin(q[2]) *
                             std::sin(q[3])) -
                        qd[3] * std::sin(q[1]) * std::sin(q[2]) *
                            std::sin(q[3]))) +
                  v_lf1_Jv_dot_tmp_tmp *
                      (std::cos(q[2]) * std::sin(q[1]) * std::sin(q[3]) +
                       std::cos(q[3]) * std::sin(q[1]) * std::sin(q[2]))) +
                 u_lf1_Jv_dot_tmp_tmp *
                     (std::sin(q[1]) * std::sin(q[2]) * std::sin(q[3]) -
                      std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1]))) /
                2.0) +
           c_lf1_Jv_dot_tmp * j_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp *
               f_lf1_Jv_dot_tmp_tmp) -
          p[1] * qd[1] * std::cos(q[1]) * e_lf1_Jv_dot_tmp_tmp *
              g_lf1_Jv_dot_tmp_tmp) -
         bb_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
             g_lf1_Jv_dot_tmp_tmp) -
        bb_lf1_Jv_dot_tmp_tmp * f_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
            e_lf1_Jv_dot_tmp_tmp) -
       n_lf1_Jv_dot_tmp_tmp * b_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
           g_lf1_Jv_dot_tmp_tmp) -
      s_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot[8] =
      ((((((((p[2] *
                  (((h_lf1_Jv_dot_tmp_tmp *
                         (((((qd[1] * std::cos(q[2]) * std::cos(q[3]) *
                                  d_lf1_Jv_dot_tmp_tmp +
                              cb_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) +
                             qd[2] * std::cos(q[1]) * std::cos(q[3]) *
                                 e_lf1_Jv_dot_tmp_tmp) +
                            db_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) +
                           qd[3] * std::cos(q[1]) * std::cos(q[3]) *
                               e_lf1_Jv_dot_tmp_tmp) -
                          qd[1] * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                              g_lf1_Jv_dot_tmp_tmp) -
                     i_lf1_Jv_dot_tmp_tmp *
                         (((((eb_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
                                  g_lf1_Jv_dot_tmp_tmp -
                              qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                                  std::cos(q[3])) -
                             qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                                 std::cos(q[3])) +
                            qd[1] * std::cos(q[3]) * d_lf1_Jv_dot_tmp_tmp *
                                e_lf1_Jv_dot_tmp_tmp) +
                           y_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                               g_lf1_Jv_dot_tmp_tmp) +
                          ab_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                              g_lf1_Jv_dot_tmp_tmp)) +
                    u_lf1_Jv_dot_tmp_tmp *
                        (fb_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp +
                         std::cos(q[1]) * std::cos(q[3]) *
                             e_lf1_Jv_dot_tmp_tmp)) +
                   v_lf1_Jv_dot_tmp_tmp *
                       (std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                        j_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp *
                            g_lf1_Jv_dot_tmp_tmp)) /
                  2.0 -
              p[3] * (((h_lf1_Jv_dot_tmp_tmp *
                            (((((qd[1] * std::cos(q[2]) * std::sin(q[1]) *
                                     std::sin(q[3]) -
                                 qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                                     std::cos(q[3])) -
                                qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                                    std::cos(q[3])) +
                               qd[1] * std::cos(q[3]) * std::sin(q[1]) *
                                   std::sin(q[2])) +
                              qd[2] * std::cos(q[1]) * std::sin(q[2]) *
                                  std::sin(q[3])) +
                             qd[3] * std::cos(q[1]) * std::sin(q[2]) *
                                 std::sin(q[3])) +
                        i_lf1_Jv_dot_tmp_tmp *
                            (((((qd[1] * std::cos(q[2]) * std::cos(q[3]) *
                                     std::sin(q[1]) +
                                 qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                                     std::sin(q[3])) +
                                qd[2] * std::cos(q[1]) * std::cos(q[3]) *
                                    std::sin(q[2])) +
                               qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                                   std::sin(q[3])) +
                              qd[3] * std::cos(q[1]) * std::cos(q[3]) *
                                  std::sin(q[2])) -
                             qd[1] * std::sin(q[1]) * std::sin(q[2]) *
                                 std::sin(q[3]))) -
                       u_lf1_Jv_dot_tmp_tmp *
                           (std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                            std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]))) +
                      v_lf1_Jv_dot_tmp_tmp *
                          (std::cos(q[1]) * std::cos(q[2]) * std::sin(q[3]) +
                           std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2])))) -
             f_lf1_Jv_dot_tmp * d_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp) +
            p[0] * qd[2] * std::cos(q[1]) * b_lf1_Jv_dot_tmp_tmp) +
           p[1] * qd[2] * std::cos(q[1]) * std::cos(q[2]) *
               f_lf1_Jv_dot_tmp_tmp) +
          p[1] * qd[3] * std::cos(q[1]) * std::cos(q[2]) *
              f_lf1_Jv_dot_tmp_tmp) -
         p[1] * qd[1] * std::cos(q[2]) * d_lf1_Jv_dot_tmp_tmp *
             g_lf1_Jv_dot_tmp_tmp) -
        c_lf1_Jv_dot_tmp * f_lf1_Jv_dot_tmp_tmp * d_lf1_Jv_dot_tmp_tmp *
            e_lf1_Jv_dot_tmp_tmp) -
       gb_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp) -
      hb_lf1_Jv_dot_tmp_tmp * e_lf1_Jv_dot_tmp_tmp * g_lf1_Jv_dot_tmp_tmp;
  lf1_Jv_dot_tmp =
      p[2] *
          (((std::cos(q[4]) *
                 (((((qd[1] * std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1]) +
                      qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                          std::sin(q[3])) +
                     qd[2] * std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2])) +
                    qd[3] * std::cos(q[1]) * std::cos(q[2]) * std::sin(q[3])) +
                   qd[3] * std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2])) -
                  qd[1] * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[3])) -
             std::sin(q[4]) *
                 (((((qd[1] * std::cos(q[2]) * std::sin(q[1]) * std::sin(q[3]) -
                      qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                          std::cos(q[3])) -
                     qd[2] * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3])) +
                    qd[1] * std::cos(q[3]) * std::sin(q[1]) * std::sin(q[2])) +
                   qd[2] * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3])) +
                  qd[3] * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]))) +
            qd[4] * std::cos(q[4]) *
                (std::cos(q[1]) * std::cos(q[2]) * std::sin(q[3]) +
                 std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2]))) +
           qd[4] * std::sin(q[4]) *
               (std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]))) /
          2.0 -
      p[3] *
          (((std::cos(q[4]) *
                 (((((qd[1] * std::cos(q[2]) * std::sin(q[1]) * std::sin(q[3]) -
                      qd[3] * std::cos(q[1]) * std::cos(q[2]) *
                          std::cos(q[3])) -
                     qd[2] * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3])) +
                    qd[1] * std::cos(q[3]) * std::sin(q[1]) * std::sin(q[2])) +
                   qd[2] * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3])) +
                  qd[3] * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3])) +
             std::sin(q[4]) *
                 (((((qd[1] * std::cos(q[2]) * std::cos(q[3]) * std::sin(q[1]) +
                      qd[2] * std::cos(q[1]) * std::cos(q[2]) *
                          std::sin(q[3])) +
                     qd[2] * std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2])) +
                    qd[3] * std::cos(q[1]) * std::cos(q[2]) * std::sin(q[3])) +
                   qd[3] * std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2])) -
                  qd[1] * std::sin(q[1]) * std::sin(q[2]) * std::sin(q[3]))) -
            qd[4] * std::cos(q[4]) *
                (std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3]) -
                 std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]))) +
           qd[4] * std::sin(q[4]) *
               (std::cos(q[1]) * std::cos(q[2]) * std::sin(q[3]) +
                std::cos(q[1]) * std::cos(q[3]) * std::sin(q[2])));
  lf1_Jv_dot[11] =
      (((((lf1_Jv_dot_tmp +
           p[1] * qd[2] * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3])) +
          p[1] * qd[3] * std::cos(q[1]) * std::cos(q[2]) * std::cos(q[3])) -
         p[1] * qd[1] * std::cos(q[2]) * std::sin(q[1]) * std::sin(q[3])) -
        p[1] * qd[1] * std::cos(q[3]) * std::sin(q[1]) * std::sin(q[2])) -
       p[1] * qd[2] * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3])) -
      p[1] * qd[3] * std::cos(q[1]) * std::sin(q[2]) * std::sin(q[3]);
  lf1_Jv_dot[14] = lf1_Jv_dot_tmp;

  // Create a 3x5 Eigen::MatrixXd object
  Eigen::MatrixXd Jaco(3, 5);

  // Copy the values from the Jacobian array to the Eigen::MatrixXd object
  for (int j = 0; j < 5; j++) {
      for (int i = 0; i < 3; i++) {
          Jaco(i, j) = lf1_Jv_dot[j * 3 + i];
      }
  }

  return Jaco;
}

#define TELLO_THIGH_LENGTH 0.228
#define TELLO_CALF_LENGTH 0.260
#define TELLO_HEEL_LENGTH 0.0576
#define TELLO_FOOT_LENGTH 0.120

