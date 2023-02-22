#include "kinematics.h"
#include "srbm_structs.h"

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

double fcn_q5_2_alpha(double q5) { // checked for .0
    double num1 = ((7.0 * sin((19.0 * M_PI) / 30.0 + q5)) / 2500.0 + (21.0 * sin(M_PI / 9.0)) / 6250.0);
    double num2 = (147.0 * cos((19.0 * M_PI) / 30.0 + q5) * cos(M_PI / 9.0)) / 50000.0 -
                  (273.0 * cos(M_PI / 9.0)) / 12500.0 - (91.0 * cos((19.0 * M_PI) / 30.0 + q5)) / 5000.0 +
                  (147.0 * sin((19.0 * M_PI) / 30.0 + q5) * sin(M_PI / 9.0)) / 50000.0 + 163349.0 / 6250000.0;
    double den1 = ((7.0 * cos((19.0 * M_PI) / 30.0 + q5)) / 2500.0 + (21.0 * cos(M_PI / 9.0)) / 6250.0 - 13.0 / 625.0);
    double den2 = (147.0 * cos((19.0 * M_PI) / 30.0 + q5) * cos(M_PI / 9.0)) / 50000.0 -
                  (273.0 * cos(M_PI / 9.0)) / 12500.0 - (91.0 * cos((19.0 * M_PI) / 30.0 + q5)) / 5000.0 +
                  (147.0 * sin((19.0 * M_PI) / 30.0 + q5) * sin(M_PI / 9.0)) / 50000.0 + 163349.0 / 6250000.0;
    double den3 = pow(pow((7.0 * sin((19.0 * M_PI) / 30.0 + q5)) / 2500.0 + (21.0 * sin(M_PI / 9.0)) / 6250.0, 2) +
                      pow((7.0 * cos((19.0 * M_PI) / 30.0 + q5)) / 2500.0 + (21.0 * cos(M_PI / 9.0)) / 6250.0 - 13.0 / 625.0, 2) -
                      pow((147.0 * cos((19.0 * M_PI) / 30.0 + q5) * cos(M_PI / 9.0)) / 50000.0 -
                          (273.0 * cos(M_PI / 9.0)) / 12500.0 - (91.0 * cos((19.0 * M_PI) / 30.0 + q5)) / 5000.0 +
                          (147.0 * sin((19.0 * M_PI) / 30.0 + q5) * sin(M_PI / 9.0)) / 50000.0 + 163349.0 / 6250000.0, 2),
                    0.5);
    double out = atan2_safe(num1 * num2 - den1 * den2, den1 * den2 + num1 * den3);
    return out;
}

double fcn_q5_2_beta(double q5) { // checked for .0
    return q5 + (19.0 * M_PI) / 30.0;
}

double fcn_alphabeta2_lambda(double alpha, double beta) { // checked for .0
    return (168.0 * sin(alpha - M_PI / 9.0) - 1040.0 * sin(alpha) + 
            140.0 * sin(alpha - beta)) / 
           (147.0 * sin(beta - M_PI / 9.0) - 910.0 * sin(beta) + 140.0 * sin(alpha - beta));
}

Eigen::MatrixXd fcn_Jaco_dq_2_dp(const Eigen::VectorXd& q)
{
    double alpha_ = fcn_q5_2_alpha(q(4));
    double beta_ = fcn_q5_2_beta(q(4));
    double lambda_ = fcn_alphabeta2_lambda(alpha_, beta_);
    Eigen::VectorXd p_ = fcn_ik_q_2_p(q);

    Eigen::MatrixXd J(5, 5);
    J.setZero();

    J(0, 0) = 1.0;
    J(1, 1) = -(196.0 * sin(q(1)) - 399.0 * cos(q(1)) - 224.0 * cos(q(1)) * sin(p_(1)) + 
              224.0 * cos(q(1)) * sin(q(2)) + 456.0 * sin(q(1)) * sin(q(2)) + 256.0 * sin(p_(1)) * 
              sin(q(1)) * sin(q(2))) / (456.0 * cos(p_(1)) - 224.0 * cos(p_(1)) * sin(q(1)) + 
              256.0 * cos(q(2)) * sin(p_(1)) - 256.0 * cos(p_(1)) * cos(q(1)) * sin(q(2)));
    J(1, 2) = (57.0 * cos(q(1)) * cos(q(2)) - 32.0 * cos(p_(1)) * sin(q(2)) - 28.0 * cos(q(2)) * 
              sin(q(1)) + 32.0 * cos(q(1)) * cos(q(2)) * sin(p_(1))) / (57.0 * cos(p_(1)) - 28.0 * 
              cos(p_(1)) * sin(q(1)) + 32.0 * cos(q(2)) * sin(p_(1)) - 32.0 * cos(p_(1)) * 
              cos(q(1)) * sin(q(2)));
    J(2, 1) = -(399.0 * cos(q(1)) + 196.0 * sin(q(1)) + 224.0 * cos(q(1)) * sin(p_(2)) - 224.0 * 
              cos(q(1)) * sin(q(2)) + 456.0 * sin(q(1)) * sin(q(2)) + 256.0 * sin(p_(2)) * 
              sin(q(1)) * sin(q(2))) / (456.0 * cos(p_(2)) + 224.0 * cos(p_(2)) * sin(q(1)) + 
              256.0 * cos(q(2)) * sin(p_(2)) - 256.0 * cos(p_(2)) * cos(q(1)) * sin(q(2)));
    J(2, 2) = (57.0 * cos(q(1)) * cos(q(2)) - 32.0 * cos(p_(2)) * sin(q(2)) + 28.0 * cos(q(2)) * 
              sin(q(1)) + 32.0 * cos(q(1)) * cos(q(2)) * sin(p_(2))) / (57.0 * cos(p_(2)) + 28.0 * 
              cos(p_(2)) * sin(q(1)) + 32.0 * cos(q(2)) * sin(p_(2)) - 32.0 * cos(p_(2)) * 
              cos(q(1)) * sin(q(2)));
    J(3, 3) = 1.0;
    J(3, 4) = 1.0 / lambda_;
    J(4, 3) = 1.0;
    J(4, 4) = -1.0 / lambda_;

    return J;
}

Eigen::MatrixXd fcn_Jaco_dp_2_dq(const Eigen::VectorXd& q) {
  VectorXd p = fcn_ik_q_2_p(q);
  double alpha = fcn_q5_2_alpha(q(4));
  double beta = fcn_q5_2_beta(q(4));
  double lambda = fcn_alphabeta2_lambda(alpha, beta);

  MatrixXd J_q_hip(2,2);
  J_q_hip(0,0) = (49.0*sin(q(1)))/5000.0 - (399.0*cos(q(1)))/20000.0 - (7.0*cos(q(1))*sin(p(1)))/625.0
                  + (7.0*cos(q(1))*sin(q(2)))/625.0 + (57.0*sin(q(1))*sin(q(2)))/2500.0
                  + (8.0*sin(p(1))*sin(q(1))*sin(q(2)))/625.0;
  J_q_hip(0,1) = (8.0*cos(p(1))*sin(q(2)))/625.0 - (57.0*cos(q(1))*cos(q(2)))/2500.0
                  + (7.0*cos(q(2))*sin(q(1)))/625.0 - (8.0*cos(q(1))*cos(q(2))*sin(p(1)))/625.0;
  J_q_hip(1,0) = (399.0*cos(q(1)))/20000.0 + (49.0*sin(q(1)))/5000.0 + (7.0*cos(q(1))*sin(p(2)))/625.0
                  - (7.0*cos(q(1))*sin(q(2)))/625.0 + (57.0*sin(q(1))*sin(q(2)))/2500.0
                  + (8.0*sin(p(2))*sin(q(1))*sin(q(2)))/625.0;
  J_q_hip(1,1) = (8.0*cos(p(2))*sin(q(2)))/625.0 - (57.0*cos(q(1))*cos(q(2)))/2500.0
                  - (7.0*cos(q(2))*sin(q(1)))/625.0 - (8.0*cos(q(1))*cos(q(2))*sin(p(2)))/625.0;

  MatrixXd J_p_hip(2,2);
  J_p_hip(0,0) = (57.0*cos(p(1)))/2500.0 - (7.0*cos(p(1))*sin(q(1)))/625.0 + (8.0*cos(q(2))*sin(p(1)))/625.0
                  - (8.0*cos(p(1))*cos(q(1))*sin(q(2)))/625.0;
  J_p_hip(1,1) = (57.0*cos(p(2)))/2500.0 + (7.0*cos(p(2))*sin(q(1)))/625.0
                  + (8.0*cos(q(2))*sin(p(2)))/625.0 - (8.0*cos(p(2))*cos(q(1))*sin(q(2)))/625.0;
  MatrixXd J_dp_2_dq_hip = -J_q_hip.colPivHouseholderQr().solve(J_p_hip); //J_dp_2_dq_hip = -J_q_hip.lu().solve(J_p_hip);
  MatrixXd J = MatrixXd::Zero(5, 5);
  J(0, 0) = 1.0;
  J.block(1, 1, 2, 2) = J_dp_2_dq_hip;
  J(3, 3) = 0.5;
  J(3, 4) = 0.5;
  J(4, 3) = lambda/2.0;
  J(4, 4) = -lambda/2.0;

  return J;
}

Eigen::VectorXd tello_leg_IK(Eigen::Vector3d& lf1, Eigen::Vector3d& lf2)
{
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0485;
  srb_params.foot_length = 0.060;

  return dash_kin::SRB_Leg_IK(srb_params,lf1,lf2);
}

Eigen::VectorXd tello_leg_IK_pointFoot(const Eigen::VectorXd& pos)
{
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0485;
  srb_params.foot_length = 0.060;
  Vector3d lf1 = pos + Vector3d(srb_params.foot_length,0,0);
  Vector3d lf2 = pos - Vector3d(srb_params.foot_length,0,0);

  return dash_kin::SRB_Leg_IK(srb_params,lf1,lf2);
}

Eigen::MatrixXd fcn_Jaco_dq_2_dT_front(const Eigen::VectorXd& q){
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0485;
  srb_params.foot_length = 0.060;

  VectorXd p = Vector4d(srb_params.thigh_length,srb_params.calf_length,srb_params.foot_length,srb_params.heel_length);
  return dash_kin::fcn_lf1_J(q, p);
}
Eigen::MatrixXd fcn_Jaco_dq_2_dT_back(const Eigen::VectorXd& q){
  SRB_Params srb_params;
  srb_params.q1_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.q2_lim = Vector2d(-65*DEGREE_TO_RADIANS,65*DEGREE_TO_RADIANS);
  srb_params.thigh_length = 0.2286;
  srb_params.calf_length = 0.260;
  srb_params.heel_length = 0.0485;
  srb_params.foot_length = 0.060;

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
  double kn = 3.111e-07 * pow(p(3), 2) 
            + 6.1188e-07 * p(3) * p(4) 
            + 0.5 * p(3) 
            + 3.0396e-07 * pow(p(4), 2) 
            + 0.5 * p(4) 
            + 4.5219e-07; 
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