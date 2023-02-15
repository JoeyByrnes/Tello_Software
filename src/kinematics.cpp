#include "kinematics.h"

using namespace Eigen;

// VectorXd fcn_ik_q_2_p_eig(const VectorXd &q)
// {
//     VectorXd p(5);
//     double q_arr[5];
//     double p_arr[5];

//     for (int i = 0; i < 5; i++) {
//         q_arr[i] = q(i);
//     }
//     fcn_ik_q_2_p(q_arr,p_arr);

//     for(int i=0; i<p.size(); i++) {
//         p(i) = p_arr[i];
//     }

//     return p;
// }
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
