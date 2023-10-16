#include "kinematics.h"

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

MatrixXd dash_kin::fcn_HTM0lf1(VectorXd q, VectorXd p)
{
    MatrixXd eigen_HTM0lf1 = MatrixXd::Zero(4,4);
    double HTM0lf1[16];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE
    
    HTM0lf1[0] =
                cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) - sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) - 
                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) + cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) - sin(q_data[4]) *
                (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) + cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) - sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));

    HTM0lf1[4] = -cos(q_data[1]) * sin(q_data[0]);
    HTM0lf1[8] =
                sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) - sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) + cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) + cos(q_data[4]) *
                (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) + cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) - sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf1[12] =
    (((-(p_data[2] * (sin(q_data[4]) *
                        (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))) -
                    cos(q_data[4]) *
                        (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1]))))) /
            2.0 -
        p_data[3] * (sin(q_data[4]) *
                        (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                        sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1]))) +
                    cos(q_data[4]) *
                        (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1])) +
                        sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))))) -
    p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                    cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
    p_data[1] * cos(q_data[3]) *
        (cos(q_data[0]) * sin(q_data[2]) +
        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
    p_data[1] * sin(q_data[3]) *
        (cos(q_data[0]) * cos(q_data[2]) -
        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM0lf1[1] =
    cos(q_data[4]) *
        (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
        sin(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
    sin(q_data[4]) *
        (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
        sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf1[5] = cos(q_data[0]) * cos(q_data[1]);
    HTM0lf1[9] =
    sin(q_data[4]) *
        (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
        sin(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
    cos(q_data[4]) *
        (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
        sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf1[13] =
    (((-(p_data[2] * (sin(q_data[4]) *
                        (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))) -
                    cos(q_data[4]) *
                        (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1]))))) /
            2.0 -
        p_data[3] * (sin(q_data[4]) *
                        (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                        sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1]))) +
                    cos(q_data[4]) *
                        (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1])) +
                        sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))))) -
    p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                    cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
    p_data[1] * cos(q_data[3]) *
        (sin(q_data[0]) * sin(q_data[2]) -
        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
    p_data[1] * sin(q_data[3]) *
        (cos(q_data[2]) * sin(q_data[0]) +
        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM0lf1[2] =
    -cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                        cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) -
    sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]));
    HTM0lf1[6] = sin(q_data[1]);
    HTM0lf1[10] =
    cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
    sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                    cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]));
    HTM0lf1[14] =
    (((p_data[1] * cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
        p_data[3] * (cos(q_data[4]) *
                        (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                        cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
                    sin(q_data[4]) *
                        (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                        cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])))) -
    p_data[0] * cos(q_data[1]) * cos(q_data[2])) -
    p_data[1] * cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3])) -
    p_data[2] *
        (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
        sin(q_data[4]) *
            (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
            cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]))) /
        2.0;
    HTM0lf1[3] = 0.0;
    HTM0lf1[7] = 0.0;
    HTM0lf1[11] = 0.0;
    HTM0lf1[15] = 1.0;

    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_HTM0lf1 = Eigen::Map<MatrixXd>(HTM0lf1, 4, 4);
    return eigen_HTM0lf1;
}

MatrixXd dash_kin::fcn_HTM0lf2(VectorXd q, VectorXd p)
{
    MatrixXd eigen_HTM0lf2 = MatrixXd::Zero(4,4);
    double HTM0lf2[16];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE

    HTM0lf2[0] =
        cos(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (cos(q_data[0]) * sin(q_data[2]) +
                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        sin(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf2[4] = -cos(q_data[1]) * sin(q_data[0]);
    HTM0lf2[8] =
        sin(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (cos(q_data[0]) * sin(q_data[2]) +
                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) +
        cos(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf2[12] =
        (((p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])))) /
                2.0 -
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))) +
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))))) -
        p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM0lf2[1] =
        cos(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (sin(q_data[0]) * sin(q_data[2]) -
                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
        sin(q_data[4]) *
            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf2[5] = cos(q_data[0]) * cos(q_data[1]);
    HTM0lf2[9] =
        sin(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (sin(q_data[0]) * sin(q_data[2]) -
                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
        cos(q_data[4]) *
            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM0lf2[13] =
        (((p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])))) /
                2.0 -
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))) +
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))))) -
        p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
        p_data[1] * cos(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM0lf2[2] =
        -cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) -
        sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                        cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]));
    HTM0lf2[6] = sin(q_data[1]);
    HTM0lf2[10] =
        cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                        cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
        sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                        cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]));
    HTM0lf2[14] =
        (((p_data[2] *
                (cos(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                    cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
                sin(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]))) /
                2.0 -
            p_data[3] * (cos(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                            cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
                        sin(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])))) -
        p_data[0] * cos(q_data[1]) * cos(q_data[2])) -
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3])) +
        p_data[1] * cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]);
    HTM0lf2[3] = 0.0;
    HTM0lf2[7] = 0.0;
    HTM0lf2[11] = 0.0;
    HTM0lf2[15] = 1.0;


    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_HTM0lf2 = Eigen::Map<MatrixXd>(HTM0lf2, 4, 4);
    return eigen_HTM0lf2;
}

MatrixXd dash_kin::fcn_HTM01(VectorXd q, VectorXd p)
{
    MatrixXd eigen_HTM01 = MatrixXd::Zero(4,4);
    double HTM01[16];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE

    HTM01[0] = cos(q_data[0]) * cos(q_data[2]) -
                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]);
    HTM01[4] = -cos(q_data[1]) * sin(q_data[0]);
    HTM01[8] = cos(q_data[0]) * sin(q_data[2]) +
                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]);
    HTM01[12] = -p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]));
    HTM01[1] = cos(q_data[2]) * sin(q_data[0]) +
                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]);
    HTM01[5] = cos(q_data[0]) * cos(q_data[1]);
    HTM01[9] = sin(q_data[0]) * sin(q_data[2]) -
                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]);
    HTM01[13] = -p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]));
    HTM01[2] = -cos(q_data[1]) * sin(q_data[2]);
    HTM01[6] = sin(q_data[1]);
    HTM01[10] = cos(q_data[1]) * cos(q_data[2]);
    HTM01[14] = -p_data[0] * cos(q_data[1]) * cos(q_data[2]);
    HTM01[3] = 0.0;
    HTM01[7] = 0.0;
    HTM01[11] = 0.0;
    HTM01[15] = 1.0;

    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_HTM01 = Eigen::Map<MatrixXd>(HTM01, 4, 4);
    return eigen_HTM01;
}

MatrixXd dash_kin::fcn_HTM02(VectorXd q, VectorXd p)
{
    MatrixXd eigen_HTM02 = MatrixXd::Zero(4,4);
    double HTM02[16];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE

    HTM02[0] =
        cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
        sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]));
    HTM02[4] = -cos(q_data[1]) * sin(q_data[0]);
    HTM02[8] =
        cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
        sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM02[12] = (-p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) -
                p_data[1] * cos(q_data[3]) *
                    (cos(q_data[0]) * sin(q_data[2]) +
                    cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
                p_data[1] * sin(q_data[3]) *
                    (cos(q_data[0]) * cos(q_data[2]) -
                    sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM02[1] =
        cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
        sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]));
    HTM02[5] = cos(q_data[0]) * cos(q_data[1]);
    HTM02[9] =
        cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
        sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM02[13] = (-p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) -
                p_data[1] * cos(q_data[3]) *
                    (sin(q_data[0]) * sin(q_data[2]) -
                    cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
                p_data[1] * sin(q_data[3]) *
                    (cos(q_data[2]) * sin(q_data[0]) +
                    cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM02[2] = -cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) -
                cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]);
    HTM02[6] = sin(q_data[1]);
    HTM02[10] = cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]);
    HTM02[14] = (p_data[1] * cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
                p_data[1] * cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3])) -
                p_data[0] * cos(q_data[1]) * cos(q_data[2]);
    HTM02[3] = 0.0;
    HTM02[7] = 0.0;
    HTM02[11] = 0.0;
    HTM02[15] = 1.0;

    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_HTM02 = Eigen::Map<MatrixXd>(HTM02, 4, 4);
    return eigen_HTM02;
}

MatrixXd dash_kin::fcn_HTM03(VectorXd q, VectorXd p)
{
    MatrixXd eigen_HTM03 = MatrixXd::Zero(4,4);
    double HTM03[16];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE

    HTM03[0] =
        cos(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (cos(q_data[0]) * sin(q_data[2]) +
                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        sin(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM03[4] = -cos(q_data[1]) * sin(q_data[0]);
    HTM03[8] =
        sin(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (cos(q_data[0]) * sin(q_data[2]) +
                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) +
        cos(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM03[12] = (-p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                                cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) -
                p_data[1] * cos(q_data[3]) *
                    (cos(q_data[0]) * sin(q_data[2]) +
                    cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
                p_data[1] * sin(q_data[3]) *
                    (cos(q_data[0]) * cos(q_data[2]) -
                    sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM03[1] =
        cos(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (sin(q_data[0]) * sin(q_data[2]) -
                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
        sin(q_data[4]) *
            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM03[5] = cos(q_data[0]) * cos(q_data[1]);
    HTM03[9] =
        sin(q_data[4]) *
            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
            sin(q_data[3]) *
                (sin(q_data[0]) * sin(q_data[2]) -
                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
        cos(q_data[4]) *
            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])));
    HTM03[13] = (-p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                                cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) -
                p_data[1] * cos(q_data[3]) *
                    (sin(q_data[0]) * sin(q_data[2]) -
                    cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) -
                p_data[1] * sin(q_data[3]) *
                    (cos(q_data[2]) * sin(q_data[0]) +
                    cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    HTM03[2] =
        -cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) -
        sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                        cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]));
    HTM03[6] = sin(q_data[1]);
    HTM03[10] =
        cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                        cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
        sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                        cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]));
    HTM03[14] = (p_data[1] * cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
                p_data[1] * cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3])) -
                p_data[0] * cos(q_data[1]) * cos(q_data[2]);
    HTM03[3] = 0.0;
    HTM03[7] = 0.0;
    HTM03[11] = 0.0;
    HTM03[15] = 1.0;

    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_HTM03 = Eigen::Map<MatrixXd>(HTM03, 4, 4);
    return eigen_HTM03;
}

MatrixXd dash_kin::fcn_lf1_J(VectorXd q, VectorXd p)
{
    MatrixXd eigen_lf1_J = MatrixXd::Zero(6,5);
    double lf1_J[30];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE

    lf1_J[0] =
        (((p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])))) /
                2.0 +
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))) +
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))))) +
        p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
        p_data[1] * cos(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    lf1_J[6] =
        (((p_data[1] * cos(q_data[1]) * sin(q_data[0]) * sin(q_data[2]) *
                sin(q_data[3]) -
            p_data[3] * (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) *
                                            cos(q_data[3]) * sin(q_data[0]) -
                                        cos(q_data[1]) * sin(q_data[0]) *
                                            sin(q_data[2]) * sin(q_data[3])) -
                        sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) *
                                            sin(q_data[0]) * sin(q_data[3]) +
                                        cos(q_data[1]) * cos(q_data[3]) *
                                            sin(q_data[0]) * sin(q_data[2])))) -
        p_data[0] * cos(q_data[1]) * cos(q_data[2]) * sin(q_data[0])) -
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) *
            sin(q_data[0])) -
        p_data[2] *
            (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[0]) *
                                    sin(q_data[3]) +
                                cos(q_data[1]) * cos(q_data[3]) * sin(q_data[0]) *
                                    sin(q_data[2])) +
            sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) *
                                    sin(q_data[0]) -
                                cos(q_data[1]) * sin(q_data[0]) * sin(q_data[2]) *
                                    sin(q_data[3]))) /
            2.0;
    lf1_J[12] =
        (((p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])))) -
            p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
                2.0) -
        p_data[0] * (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]));
    lf1_J[18] =
        ((p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1])))) -
        p_data[2] *
            (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
            2.0) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]));
    lf1_J[24] =
        p_data[3] *
            (sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
            cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])))) -
        p_data[2] *
            (sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))) +
            cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])))) /
            2.0;
    lf1_J[1] =
        (((-(p_data[2] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))))) /
                2.0 -
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))) +
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))))) -
        p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    lf1_J[7] =
        (((p_data[2] *
                (cos(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                    cos(q_data[2]) * sin(q_data[3]) +
                                cos(q_data[0]) * cos(q_data[1]) *
                                    cos(q_data[3]) * sin(q_data[2])) +
                sin(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                    cos(q_data[2]) * cos(q_data[3]) -
                                cos(q_data[0]) * cos(q_data[1]) *
                                    sin(q_data[2]) * sin(q_data[3]))) /
                2.0 +
            p_data[3] * (cos(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                            cos(q_data[2]) * cos(q_data[3]) -
                                        cos(q_data[0]) * cos(q_data[1]) *
                                            sin(q_data[2]) * sin(q_data[3])) -
                        sin(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                            cos(q_data[2]) * sin(q_data[3]) +
                                        cos(q_data[0]) * cos(q_data[1]) *
                                            cos(q_data[3]) * sin(q_data[2])))) +
        p_data[0] * cos(q_data[0]) * cos(q_data[1]) * cos(q_data[2])) +
        p_data[1] * cos(q_data[0]) * cos(q_data[1]) * cos(q_data[2]) *
            cos(q_data[3])) -
        p_data[1] * cos(q_data[0]) * cos(q_data[1]) * sin(q_data[2]) *
            sin(q_data[3]);
    lf1_J[13] =
        (((p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])))) -
            p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
                2.0) -
        p_data[0] * (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]));
    lf1_J[19] =
        ((p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1])))) -
        p_data[2] *
            (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
            2.0) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]));
    lf1_J[25] =
        p_data[3] *
            (sin(q_data[4]) * (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
            cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])))) -
        p_data[2] *
            (sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))) +
            cos(q_data[4]) * (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])))) /
            2.0;
    lf1_J[2] = 0.0;
    lf1_J[8] =
        (((p_data[2] *
                (cos(q_data[4]) *
                    (cos(q_data[2]) * sin(q_data[1]) * sin(q_data[3]) +
                    cos(q_data[3]) * sin(q_data[1]) * sin(q_data[2])) -
                sin(q_data[4]) *
                    (sin(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
                    cos(q_data[2]) * cos(q_data[3]) * sin(q_data[1]))) /
                2.0 -
            p_data[3] * (cos(q_data[4]) *
                            (sin(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
                            cos(q_data[2]) * cos(q_data[3]) * sin(q_data[1])) +
                        sin(q_data[4]) *
                            (cos(q_data[2]) * sin(q_data[1]) * sin(q_data[3]) +
                            cos(q_data[3]) * sin(q_data[1]) * sin(q_data[2])))) +
        p_data[0] * cos(q_data[2]) * sin(q_data[1])) +
        p_data[1] * cos(q_data[2]) * cos(q_data[3]) * sin(q_data[1])) -
        p_data[1] * sin(q_data[1]) * sin(q_data[2]) * sin(q_data[3]);
    lf1_J[14] =
        (((p_data[3] * (cos(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
                        sin(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                            cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]))) -
            p_data[2] *
                (cos(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
                sin(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                    cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]))) /
                2.0) +
        p_data[0] * cos(q_data[1]) * sin(q_data[2])) +
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3])) +
        p_data[1] * cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]);
    lf1_J[20] =
        ((p_data[3] * (cos(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
                        sin(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                            cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]))) -
        p_data[2] *
            (cos(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
                sin(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                    cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]))) /
            2.0) +
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3])) +
        p_data[1] * cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]);
    lf1_J[26] =
        p_data[3] *
            (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                                cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
            sin(q_data[4]) *
                (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3]))) -
        p_data[2] *
            (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                                cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
            sin(q_data[4]) *
                (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]))) /
            2.0;
    lf1_J[3] = 0.0;
    lf1_J[9] = cos(q_data[0]);
    lf1_J[15] = -cos(q_data[1]) * sin(q_data[0]);
    lf1_J[21] = -cos(q_data[1]) * sin(q_data[0]);
    lf1_J[27] = -cos(q_data[1]) * sin(q_data[0]);
    lf1_J[4] = 0.0;
    lf1_J[10] = sin(q_data[0]);
    lf1_J[16] = cos(q_data[0]) * cos(q_data[1]);
    lf1_J[22] = cos(q_data[0]) * cos(q_data[1]);
    lf1_J[28] = cos(q_data[0]) * cos(q_data[1]);
    lf1_J[5] = 1.0;
    lf1_J[11] = 0.0;
    lf1_J[17] = sin(q_data[1]);
    lf1_J[23] = sin(q_data[1]);
    lf1_J[29] = sin(q_data[1]);

    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_lf1_J = Eigen::Map<MatrixXd>(lf1_J, 6, 5);
    return eigen_lf1_J;
}

MatrixXd dash_kin::fcn_lf2_J(VectorXd q, VectorXd p)
{
    MatrixXd eigen_lf2_J = MatrixXd::Zero(6,5);
    double lf2_J[30];
    double p_data[4];
    double q_data[5];
    for(int i = 0; i<5;i++){
        q_data[i] = q(i);
    }
    for(int i = 0; i<4;i++){
        p_data[i] = p(i);
    }
    //// START MATLAB CODER AUTO_GENERATED CODE

    lf2_J[0] =
        (((p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))) +
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])))) -
            p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])))) /
                2.0) +
        p_data[0] * (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
        p_data[1] * cos(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    lf2_J[6] =
        (((p_data[2] *
                (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) *
                                    sin(q_data[0]) * sin(q_data[3]) +
                                cos(q_data[1]) * cos(q_data[3]) *
                                    sin(q_data[0]) * sin(q_data[2])) +
                sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) *
                                    cos(q_data[3]) * sin(q_data[0]) -
                                cos(q_data[1]) * sin(q_data[0]) *
                                    sin(q_data[2]) * sin(q_data[3]))) /
                2.0 -
            p_data[3] * (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) *
                                            cos(q_data[3]) * sin(q_data[0]) -
                                        cos(q_data[1]) * sin(q_data[0]) *
                                            sin(q_data[2]) * sin(q_data[3])) -
                        sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) *
                                            sin(q_data[0]) * sin(q_data[3]) +
                                        cos(q_data[1]) * cos(q_data[3]) *
                                            sin(q_data[0]) * sin(q_data[2])))) -
        p_data[0] * cos(q_data[1]) * cos(q_data[2]) * sin(q_data[0])) -
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) *
            sin(q_data[0])) +
        p_data[1] * cos(q_data[1]) * sin(q_data[0]) * sin(q_data[2]) *
            sin(q_data[3]);
    lf2_J[12] =
        (((p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
                2.0 +
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))))) -
        p_data[0] * (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]));
    lf2_J[18] =
        ((p_data[2] *
            (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
            2.0 +
        p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                            sin(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                            cos(q_data[2]) * sin(q_data[0]) *
                                                sin(q_data[1]))))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]));
    lf2_J[24] =
        p_data[2] *
            (sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))) +
            cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])))) /
            2.0 +
        p_data[3] *
            (sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
            cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))));
    lf2_J[1] =
        (((p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[0]) * cos(q_data[2]) -
                        sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1])))) /
                2.0 -
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1]))) +
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[0]) * sin(q_data[2]) +
                                                cos(q_data[2]) * sin(q_data[0]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[0]) * cos(q_data[2]) -
                                                sin(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))))) -
        p_data[0] * (cos(q_data[0]) * sin(q_data[2]) +
                        cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[0]) * sin(q_data[2]) +
            cos(q_data[2]) * sin(q_data[0]) * sin(q_data[1]))) -
        p_data[1] * sin(q_data[3]) *
            (cos(q_data[0]) * cos(q_data[2]) -
            sin(q_data[0]) * sin(q_data[1]) * sin(q_data[2]));
    lf2_J[7] =
        (((p_data[3] * (cos(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                            cos(q_data[2]) * cos(q_data[3]) -
                                        cos(q_data[0]) * cos(q_data[1]) *
                                            sin(q_data[2]) * sin(q_data[3])) -
                        sin(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                            cos(q_data[2]) * sin(q_data[3]) +
                                        cos(q_data[0]) * cos(q_data[1]) *
                                            cos(q_data[3]) * sin(q_data[2]))) -
            p_data[2] *
                (cos(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                    cos(q_data[2]) * sin(q_data[3]) +
                                cos(q_data[0]) * cos(q_data[1]) *
                                    cos(q_data[3]) * sin(q_data[2])) +
                sin(q_data[4]) * (cos(q_data[0]) * cos(q_data[1]) *
                                    cos(q_data[2]) * cos(q_data[3]) -
                                cos(q_data[0]) * cos(q_data[1]) *
                                    sin(q_data[2]) * sin(q_data[3]))) /
                2.0) +
        p_data[0] * cos(q_data[0]) * cos(q_data[1]) * cos(q_data[2])) +
        p_data[1] * cos(q_data[0]) * cos(q_data[1]) * cos(q_data[2]) *
            cos(q_data[3])) -
        p_data[1] * cos(q_data[0]) * cos(q_data[1]) * sin(q_data[2]) *
            sin(q_data[3]);
    lf2_J[13] =
        (((p_data[2] *
                (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
                2.0 +
            p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))))) -
        p_data[0] * (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]));
    lf2_J[19] =
        ((p_data[2] *
            (sin(q_data[4]) *
                    (cos(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])) -
                    sin(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]))) +
                cos(q_data[4]) *
                    (cos(q_data[3]) *
                        (sin(q_data[0]) * sin(q_data[2]) -
                        cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1])) +
                    sin(q_data[3]) *
                        (cos(q_data[2]) * sin(q_data[0]) +
                        cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2])))) /
            2.0 +
        p_data[3] * (sin(q_data[4]) *
                            (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1])) +
                            sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2]))) -
                        cos(q_data[4]) *
                            (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                            cos(q_data[0]) * sin(q_data[1]) *
                                                sin(q_data[2])) -
                            sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                            cos(q_data[0]) * cos(q_data[2]) *
                                                sin(q_data[1]))))) -
        p_data[1] * cos(q_data[3]) *
            (cos(q_data[2]) * sin(q_data[0]) +
            cos(q_data[0]) * sin(q_data[1]) * sin(q_data[2]))) +
        p_data[1] * sin(q_data[3]) *
            (sin(q_data[0]) * sin(q_data[2]) -
            cos(q_data[0]) * cos(q_data[2]) * sin(q_data[1]));
    lf2_J[25] =
        p_data[2] *
            (sin(q_data[4]) * (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))) +
            cos(q_data[4]) * (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])))) /
            2.0 +
        p_data[3] *
            (sin(q_data[4]) * (cos(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1])) +
                                sin(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2]))) -
            cos(q_data[4]) * (cos(q_data[3]) * (cos(q_data[2]) * sin(q_data[0]) +
                                                cos(q_data[0]) * sin(q_data[1]) *
                                                    sin(q_data[2])) -
                                sin(q_data[3]) * (sin(q_data[0]) * sin(q_data[2]) -
                                                cos(q_data[0]) * cos(q_data[2]) *
                                                    sin(q_data[1]))));
    lf2_J[2] = 0.0;
    lf2_J[8] =
        (((p_data[0] * cos(q_data[2]) * sin(q_data[1]) -
            p_data[3] * (cos(q_data[4]) *
                            (sin(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
                            cos(q_data[2]) * cos(q_data[3]) * sin(q_data[1])) +
                        sin(q_data[4]) *
                            (cos(q_data[2]) * sin(q_data[1]) * sin(q_data[3]) +
                            cos(q_data[3]) * sin(q_data[1]) * sin(q_data[2])))) -
        p_data[2] *
            (cos(q_data[4]) *
                    (cos(q_data[2]) * sin(q_data[1]) * sin(q_data[3]) +
                    cos(q_data[3]) * sin(q_data[1]) * sin(q_data[2])) -
                sin(q_data[4]) *
                    (sin(q_data[1]) * sin(q_data[2]) * sin(q_data[3]) -
                    cos(q_data[2]) * cos(q_data[3]) * sin(q_data[1]))) /
            2.0) +
        p_data[1] * cos(q_data[2]) * cos(q_data[3]) * sin(q_data[1])) -
        p_data[1] * sin(q_data[1]) * sin(q_data[2]) * sin(q_data[3]);
    lf2_J[14] =
        (((p_data[2] *
                (cos(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
                sin(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                    cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]))) /
                2.0 +
            p_data[3] * (cos(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
                        sin(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                            cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])))) +
        p_data[0] * cos(q_data[1]) * sin(q_data[2])) +
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3])) +
        p_data[1] * cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]);
    lf2_J[20] =
        ((p_data[2] *
            (cos(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                    cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
                sin(q_data[4]) *
                    (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                    cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]))) /
            2.0 +
        p_data[3] * (cos(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                            cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
                        sin(q_data[4]) *
                            (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                            cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])))) +
        p_data[1] * cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3])) +
        p_data[1] * cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]);
    lf2_J[26] =
        p_data[2] *
            (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                                cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])) -
            sin(q_data[4]) *
                (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2]))) /
            2.0 +
        p_data[3] *
            (cos(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * sin(q_data[3]) +
                                cos(q_data[1]) * cos(q_data[3]) * sin(q_data[2])) +
            sin(q_data[4]) * (cos(q_data[1]) * cos(q_data[2]) * cos(q_data[3]) -
                                cos(q_data[1]) * sin(q_data[2]) * sin(q_data[3])));
    lf2_J[3] = 0.0;
    lf2_J[9] = cos(q_data[0]);
    lf2_J[15] = -cos(q_data[1]) * sin(q_data[0]);
    lf2_J[21] = -cos(q_data[1]) * sin(q_data[0]);
    lf2_J[27] = -cos(q_data[1]) * sin(q_data[0]);
    lf2_J[4] = 0.0;
    lf2_J[10] = sin(q_data[0]);
    lf2_J[16] = cos(q_data[0]) * cos(q_data[1]);
    lf2_J[22] = cos(q_data[0]) * cos(q_data[1]);
    lf2_J[28] = cos(q_data[0]) * cos(q_data[1]);
    lf2_J[5] = 1.0;
    lf2_J[11] = 0.0;
    lf2_J[17] = sin(q_data[1]);
    lf2_J[23] = sin(q_data[1]);
    lf2_J[29] = sin(q_data[1]);

    // END MATLAB CODER AUTO_GENERATED CODE
    eigen_lf2_J = Eigen::Map<MatrixXd>(lf2_J, 6, 5); // uses column moajor by default
    return eigen_lf2_J;
}

MatrixXd dash_kin::compute_torso_vertices_locations(MatrixXd HTMwd2com,VectorXd p_torso)
{
// Define HTM (vertice to CoM)
    Eigen::Matrix4d HTMcom2v1, HTMcom2v2, HTMcom2v3, HTMcom2v4, HTMcom2v5, HTMcom2v6, HTMcom2v7, HTMcom2v8;

    HTMcom2v1 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(p_torso(0)/2.0, p_torso(1)/2.0, -p_torso(3)), 0, 0, 0, 1;
    HTMcom2v2 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(p_torso(0)/2.0, -p_torso(1)/2.0, -p_torso(3)), 0, 0, 0, 1;
    HTMcom2v3 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(-p_torso(0)/2.0, p_torso(1)/2.0, -p_torso(3)), 0, 0, 0, 1;
    HTMcom2v4 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(-p_torso(0)/2.0, -p_torso(1)/2.0, -p_torso(3)), 0, 0, 0, 1;
    HTMcom2v5 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(p_torso(0)/2.0, p_torso(1)/2.0, p_torso(2) - p_torso(3)), 0, 0, 0, 1;
    HTMcom2v6 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(p_torso(0)/2.0, -p_torso(1)/2.0, p_torso(2) - p_torso(3)), 0, 0, 0, 1;
    HTMcom2v7 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(-p_torso(0)/2.0, p_torso(1)/2.0, p_torso(2) - p_torso(3)), 0, 0, 0, 1;
    HTMcom2v8 << Eigen::Matrix3d::Identity(), Eigen::Vector3d(-p_torso(0)/2.0, -p_torso(1)/2.0, p_torso(2) - p_torso(3)), 0, 0, 0, 1;

    // HTM calculations (vertice to world)
    Eigen::Matrix4d HTMwd2v1 = HTMwd2com * HTMcom2v1;
    Eigen::Matrix4d HTMwd2v2 = HTMwd2com * HTMcom2v2;
    Eigen::Matrix4d HTMwd2v3 = HTMwd2com * HTMcom2v3;
    Eigen::Matrix4d HTMwd2v4 = HTMwd2com * HTMcom2v4;
    Eigen::Matrix4d HTMwd2v5 = HTMwd2com * HTMcom2v5;
    Eigen::Matrix4d HTMwd2v6 = HTMwd2com * HTMcom2v6;
    Eigen::Matrix4d HTMwd2v7 = HTMwd2com * HTMcom2v7;
    Eigen::Matrix4d HTMwd2v8 = HTMwd2com * HTMcom2v8;

    
    // Store torso vertices in world frame
    Eigen::Matrix<double, 3, 8> torso_vertices;

    torso_vertices.col(0) = HTMwd2v1.block<3,1>(0,3);
    torso_vertices.col(1) = HTMwd2v2.block<3,1>(0,3);
    torso_vertices.col(2) = HTMwd2v3.block<3,1>(0,3);
    torso_vertices.col(3) = HTMwd2v4.block<3,1>(0,3);
    torso_vertices.col(4) = HTMwd2v5.block<3,1>(0,3);
    torso_vertices.col(5) = HTMwd2v6.block<3,1>(0,3);
    torso_vertices.col(6) = HTMwd2v7.block<3,1>(0,3);
    torso_vertices.col(7) = HTMwd2v8.block<3,1>(0,3);

    return torso_vertices;
}

MatrixXd dash_kin::leg_FK(MatrixXd HTMwd2hip, VectorXd q, VectorXd p_leg){

    Eigen::Matrix4d HTMwd2thigh, HTMwd2ankle, HTMwd2lf1, HTMwd2lf2;

    // compute thigh to world
    HTMwd2thigh = HTMwd2hip * fcn_HTM01(q, p_leg);

    // compute ankle to world
    HTMwd2ankle = HTMwd2hip * fcn_HTM03(q, p_leg);

    // compute toe1 to world
    HTMwd2lf1 = HTMwd2hip * fcn_HTM0lf1(q, p_leg);

    // compute toe2 to world
    HTMwd2lf2 = HTMwd2hip * fcn_HTM0lf2(q, p_leg);

    // compute leg positions in world frame for hip, knee, ankle, and end-effector (s)
    Eigen::Matrix<double, 3, 5> leg;
    leg.col(0) = HTMwd2hip.block<3, 1>(0, 3);
    leg.col(1) = HTMwd2thigh.block<3, 1>(0, 3);
    leg.col(2) = HTMwd2ankle.block<3, 1>(0, 3);
    leg.col(3) = HTMwd2lf1.block<3, 1>(0, 3);
    leg.col(4) = HTMwd2lf2.block<3, 1>(0, 3);

    return leg;

}

void dash_kin::SRB_FK(MatrixXd& torso_vertices, MatrixXd& right_leg, MatrixXd& left_leg, MatrixXd& lfv, SRB_Params srb_params, VectorXd x, MatrixXd q)
{
    // Parameters
    double L = srb_params.L; // body length in m (along x-direction) -- visualization only
    double W = srb_params.W; // body width in m (along y-direction)
    double H = srb_params.H; // body height in m (along z-direction) -- visualization only
    double CoM2H_z_dist = srb_params.CoM2H_z_dist; // CoM to hip connection z-direction distance in m
    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)
    
    // Construct torso dimensions vector
    Eigen::Vector4d p_torso(L, W, H, CoM2H_z_dist);

    // Construct leg parameter vector
    Eigen::Vector4d p_leg(thigh_length, calf_length, foot_length, heel_length);

    Vector3d pcom = x.segment<3>(0);
    Matrix3d R = Eigen::Map<Matrix3d>(x.segment(6,9).data());

    // get leg joint angles for each leg
    Eigen::VectorXd qr = q.row(0);
    Eigen::VectorXd ql = q.row(1);
    
    // torso to world
    Eigen::Matrix<double,4,4> HTMwd2com;
    //HTMwd2com << R, pcom, 0, 0, 0, 1;
    HTMwd2com.block(0,0,3,3) = R;
    HTMwd2com.block(0,3,3,1) = pcom;
    HTMwd2com.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();
                      
    // SRB vertices to torso
    torso_vertices = compute_torso_vertices_locations(HTMwd2com, p_torso);
    
    // right hip to world
    Eigen::Matrix4d HTMcom2hr;
    // HTMcom2hr.setIdentity();
    // HTMcom2hr(1,3) = -W/2.0;
    // HTMcom2hr(2,3) = -CoM2H_z_dist;
    HTMcom2hr.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    HTMcom2hr.block(0,3,3,1) = Eigen::Vector3d(CoM_X_Offset, -W/2.0, -CoM2H_z_dist);
    HTMcom2hr.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();

    Eigen::Matrix4d HTMwd2hr = HTMwd2com * HTMcom2hr;
    
    // left hip to world
    Eigen::Matrix4d HTMcom2hl;
    // HTMcom2hl.setIdentity();
    // HTMcom2hl(1,3) = W/2.0;
    // HTMcom2hl(2,3) = -CoM2H_z_dist;
    HTMcom2hl.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    HTMcom2hl.block(0,3,3,1) = Eigen::Vector3d(CoM_X_Offset, W/2.0, -CoM2H_z_dist);
    HTMcom2hl.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();

    Eigen::Matrix4d HTMwd2hl = HTMwd2com * HTMcom2hl;
    
    // legs FK
    right_leg = leg_FK(HTMwd2hr, qr, p_leg);
    left_leg = leg_FK(HTMwd2hl, ql, p_leg);

    lfv.row(0) = right_leg.col(3).transpose();
    lfv.row(1) = right_leg.col(4).transpose();
    lfv.row(2) = left_leg.col(3).transpose();
    lfv.row(3) = left_leg.col(4).transpose();
}

void dash_kin::Paden_Kahan_subproblem2(Vector2d& sol1_PK_subp2,Vector2d& sol2_PK_subp2, Vector3d p_PK_subp2, Vector3d q_PK_subp2, Vector3d r_PK_subp2, Vector3d omega1_PK_subp2, Vector3d omega2_PK_subp2) {
    // This sub-function solves the Paden-Kahan subproblem 2
    // Rotation about two subsequent axes
    // https://en.wikipedia.org/wiki/Paden%E2%80%93Kahan_subproblems#Subproblem_2:_Rotation_about_two_subsequent_axes
    
    // vector definition
    Vector3d u = p_PK_subp2 - r_PK_subp2;
    Vector3d v = q_PK_subp2 - r_PK_subp2;
    
    // calculations - z = alpha*omega1 + beta*omega2 + gamma*(omega1 x omega2)
    double alpha = (((omega1_PK_subp2.transpose() * omega2_PK_subp2) * omega2_PK_subp2.transpose() * u - omega1_PK_subp2.transpose() * v) / ((omega1_PK_subp2.transpose() * omega2_PK_subp2).squaredNorm() - 1.0)).value();
    double beta = (((omega1_PK_subp2.transpose() * omega2_PK_subp2) * omega1_PK_subp2.transpose() * v - omega2_PK_subp2.transpose() * u) / ((omega1_PK_subp2.transpose() * omega2_PK_subp2).squaredNorm() - 1.0)).value();
    double gamma_squared = (u.squaredNorm() - alpha*alpha - beta*beta - 2*alpha*beta*omega1_PK_subp2.transpose() * omega2_PK_subp2) / (omega1_PK_subp2.cross(omega2_PK_subp2).squaredNorm());
    Vector3d z1 = alpha*omega1_PK_subp2 + beta*omega2_PK_subp2 + sqrt(gamma_squared) * omega1_PK_subp2.cross(omega2_PK_subp2);
    Vector3d z2 = alpha*omega1_PK_subp2 + beta*omega2_PK_subp2 - sqrt(gamma_squared) * omega1_PK_subp2.cross(omega2_PK_subp2);
    
    // solve subproblem 1 for c1 and c2 (each corresponds to sol1 and sol2)
    Vector3d c1 = z1 + r_PK_subp2;
    Vector3d c2 = z2 + r_PK_subp2;
    double theta1_1 = -1.0 * Paden_Kahan_subproblem1(q_PK_subp2, c1, r_PK_subp2, omega1_PK_subp2);
    double theta2_1 = Paden_Kahan_subproblem1(p_PK_subp2, c1, r_PK_subp2, omega2_PK_subp2);
    double theta1_2 = -1.0 * Paden_Kahan_subproblem1(q_PK_subp2, c2, r_PK_subp2, omega1_PK_subp2);
    double theta2_2 = Paden_Kahan_subproblem1(p_PK_subp2, c2, r_PK_subp2, omega2_PK_subp2);
     sol1_PK_subp2 = Vector2d(theta1_1, theta2_1);
     sol2_PK_subp2 = Vector2d(theta1_2, theta2_2);

}

double dash_kin::Paden_Kahan_subproblem1(const Vector3d& p_PK_subp1, const Vector3d& q_PK_subp1, const Vector3d& r_PK_subp1, const Vector3d& omega_PK_subp1)
{
    // vector definition
    Vector3d u = p_PK_subp1 - r_PK_subp1;
    Vector3d v = q_PK_subp1 - r_PK_subp1;

    // calculations
    Vector3d u_prime = u - omega_PK_subp1 * (omega_PK_subp1.transpose() * u);
    Vector3d v_prime = v - omega_PK_subp1 * (omega_PK_subp1.transpose() * v);

    // compute theta (i.e., solution)
    double theta_PK_subp1 = atan2_safe(omega_PK_subp1.transpose() * u_prime.cross(v_prime), u_prime.dot(v_prime));
    return theta_PK_subp1;
}

VectorXd dash_kin::SRB_Leg_IK(SRB_Params& srb_params, VectorXd& lf1, VectorXd& lf2)
{
    // This function computes the joint positions for a 5 DoF (hip = 3, knee = 1, ankle = 1) serial revolute-joint bipedal robot leg.
    // The inputs are the end-effector (line foot front and back) positions in the body frame.
    
    // Parameters
    Vector2d q1_lim = srb_params.q1_lim; // ab joint limit
    Vector2d q2_lim = srb_params.q2_lim; // hip joint limit
    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double heel_length = srb_params.heel_length; // heel length in m (L4)

    // ab (q1) and hip (q2) joint angles

    // unit vectors perpendicular to foot plane in world frame and body frame
    Vector3d n1(0, 1, 0);
    Vector3d n2 = (dash_utils::hatMap(lf1)*lf2)/(dash_utils::hatMap(lf1)*lf2).norm();
    
    // Solve Paden_Kahan_subproblem2 to get q1 and q2 (we get two solutions --
    // only one is correct)
    // n2 = Rbh*n1
    // Rbh = rz(q1)*rx(q2)
    Vector2d sol1, sol2;
    Paden_Kahan_subproblem2(sol1, sol2, n1, n2, Vector3d::Zero(), Vector3d(0, 0, 1), Vector3d(1, 0, 0));
    
    // Asses two solutions
    VectorXd sol = Assess_PKsubp2_sols(sol1, sol2, n1, n2, q1_lim, q2_lim);
    double q1 = sol(0);
    double q2 = sol(1);
    
    // convert to new hip frame before thigh angle (q3)
    Matrix3d Rbh = dash_utils::rz(q1)*dash_utils::rx(q2);
    Vector3d h2lff_h_p = Rbh.transpose() * lf1;
    Vector3d h2lfb_h_p = Rbh.transpose() * lf2;
    
    // thigh (q3), calf (q4), and ankle (q5) joint angles

    // calculations
    // define point mf as middle of the foot
    Vector3d lfb2lff_p = h2lff_h_p - h2lfb_h_p;
    Vector3d lfb2mf_p = (1.0 / 2.0) * lfb2lff_p;
    Vector3d mf2lfb_p = -lfb2mf_p;
    Vector3d mf2lfb_p_hat = mf2lfb_p/mf2lfb_p.norm();
    Vector3d mf2a_p_hat = dash_utils::ry(M_PI / 2.0) * mf2lfb_p_hat;
    Vector3d mf2a_p = heel_length * mf2a_p_hat;
    Vector3d h2mf_p = h2lfb_h_p + lfb2mf_p;
    Vector3d h2a_p = h2mf_p + mf2a_p;
    double h2a_l = h2a_p.norm();
    
    // HKA triangle angles
    double theta = acos((pow(h2a_l, 2) - pow(thigh_length, 2) - pow(calf_length, 2)) / (-2.0 * thigh_length * calf_length));
    double alpha = acos((pow(calf_length, 2) - pow(thigh_length, 2) - pow(h2a_l, 2)) / (-2.0 * thigh_length * h2a_l));

    // HBA triangle angle
    double beta = atan(h2a_p(0) / h2a_p(2));
    
    // calf (knee) joint angle (q4) -- always greater than 0!!
    double q4 = M_PI - theta;

    // thigh joint angle (q3)
    double q3 = beta - alpha;
    if (h2a_p(2) > 0) {
        q3 += M_PI;
    }

    // get knee position
    Eigen::Matrix3d Rht = dash_utils::ry(q3);
    Eigen::Vector3d dht = Rht * Eigen::Vector3d(0, 0, -thigh_length);
    Eigen::Matrix4d HTMht;
    HTMht << Rht, dht, 0, 0, 0, 1;
    Eigen::Vector3d h2k_p = HTMht.block<3, 1>(0, 3);
    Eigen::Vector3d a2k_p = h2a_p - h2k_p;

    // KAS triangle angle
    double psi = acos(a2k_p.dot(mf2a_p) / (heel_length * calf_length));
    
    // ankle joint angle magnitude
    double q5_magnitude = M_PI - psi;

    // figure out polarity of q5
    Eigen::Vector3d sin_theta_nhat = dash_utils::hatMap(mf2a_p) * a2k_p / (calf_length * heel_length);
    double sin_theta = sin_theta_nhat(1);
    double sin_theta_tol = 1e-10;
    int q5_polarity;
    if (fabs(sin_theta - sin(q5_magnitude)) < sin_theta_tol) {
        q5_polarity = 1;
    } else {
        q5_polarity = -1;
    }

    // ankle joint angle (q5)
    double q5 = q5_magnitude * q5_polarity;

    // Inverse kinematics results
    Eigen::VectorXd q_leg(5);
    q_leg << q1, q2, q3, q4, q5;
    return q_leg;
}

VectorXd dash_kin::Assess_PKsubp2_sols(Vector2d sol1,Vector2d sol2,Vector3d n1,Vector3d n2,Vector2d q1_lim,Vector2d q2_lim)
{
    double q1_sol1 = sol1(0);
    double q2_sol1 = sol1(1);
    Matrix3d rz_q1_sol1;
    rz_q1_sol1 << cos(q1_sol1), -sin(q1_sol1), 0,
                sin(q1_sol1), cos(q1_sol1), 0,
                0, 0, 1;

    Matrix3d rx_q2_sol1;
    rx_q2_sol1 << 1, 0, 0,
                0, cos(q2_sol1), -sin(q2_sol1),
                0, sin(q2_sol1), cos(q2_sol1);

    Matrix3d Rbh_sol1 = rz_q1_sol1 * rx_q2_sol1;

    double q1_sol2 = sol2(0);
    double q2_sol2 = sol2(1);
    Matrix3d rz_q1_sol2;
    rz_q1_sol2 << cos(q1_sol2), -sin(q1_sol2), 0,
                sin(q1_sol2), cos(q1_sol2), 0,
                0, 0, 1;

    Matrix3d rx_q2_sol2;
    rx_q2_sol2 << 1, 0, 0,
                0, cos(q2_sol2), -sin(q2_sol2),
                0, sin(q2_sol2), cos(q2_sol2);

    Matrix3d Rbh_sol2 = rz_q1_sol2 * rx_q2_sol2;

    // check joint limits condition
    bool sol1_joint_limits_flg = ((q1_sol1 >= q1_lim(0)) && (q1_sol1 <= q1_lim(1))) && ((q2_sol1 >= q2_lim(0)) && (q2_sol1 <= q2_lim(1)));
    bool sol2_joint_limits_flg = ((q1_sol2 >= q1_lim(0)) && (q1_sol2 <= q1_lim(1))) && ((q2_sol2 >= q2_lim(0)) && (q2_sol2 <= q2_lim(1)));

    // check satisfaction of n2 = Rbh*n1
    double diff_tol = 1e-10;
    bool sol1_coordinate_transformation_flg = (n2 - Rbh_sol1*n1).norm() < diff_tol;
    bool sol2_coordinate_transformation_flg = (n2 - Rbh_sol2*n1).norm() < diff_tol;

    // select "correct" solution
    Vector2d sol;
    if(sol1_joint_limits_flg && sol1_coordinate_transformation_flg)
    {
        sol = sol1;
    }
    else if(sol2_joint_limits_flg && sol2_coordinate_transformation_flg)
    {
        sol = sol2;
    }
    else
    {
        sol = VectorXd::Zero(2);
    }
    return sol;

}

MatrixXd dash_kin::SRB_IK(SRB_Params srb_params, VectorXd CoM, MatrixXd R, MatrixXd lfv)
{
    // This function computes the leg joint positions from task-space humanoid
    // robot states. We assume that each humanoid robot leg has 5 DoF (hip = 3,
    // knee = 1, ankle = 1). We also assume two known end-effector points for
    // each line foot. 
    
    // Parameters
    double W = srb_params.W;
    double CoM2H_z_dist = srb_params.CoM2H_z_dist;
    Eigen::Matrix4d HTMwd2com, HTMcom2hr, HTMcom2hl, HTMwd2hr, HTMwd2hl;
    Eigen::VectorXd pc_curr, lf1R, lf2R, lf1L, lf2L, hr_wd_p, hl_wd_p, h2lf1R_wd_p, h2lf2R_wd_p, h2lf1L_wd_p, h2lf2L_wd_p;
    Eigen::VectorXd q_leg_R, q_leg_L;
    Eigen::MatrixXd q_leg(2,5);
    Eigen::Matrix3d R_curr;
    Eigen::VectorXd h2lf1R_b_p, h2lf2R_b_p, h2lf1L_b_p, h2lf2L_b_p;
    
    // task-space bipedal robot states
    pc_curr = CoM;
    R_curr = R;

    lf1R = lfv.row(0);
    lf2R = lfv.row(1);
    lf1L = lfv.row(2);
    lf2L = lfv.row(3);
    
    // HTMs
    HTMwd2com.block(0,0,3,3) = R_curr;
    HTMwd2com.block(0,3,3,1) = pc_curr;
    HTMwd2com.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();
    // HTMwd2com << R_curr, pc_curr, 0, 0, 0, 1;

    HTMcom2hr.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    HTMcom2hr.block(0,3,3,1) = Eigen::Vector3d(CoM_X_Offset, -W/2.0, -CoM2H_z_dist);
    HTMcom2hr.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();
    // HTMcom2hr << Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, -W/2, -CoM2H_z_dist), 0, 0, 0, 1;

    HTMcom2hl.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    HTMcom2hl.block(0,3,3,1) = Eigen::Vector3d(CoM_X_Offset, W/2.0, -CoM2H_z_dist);
    HTMcom2hl.block(3,0,1,4) = Vector4d(0,0,0,1).transpose();
    // HTMcom2hl << Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, W/2, -CoM2H_z_dist), 0, 0, 0, 1;

    HTMwd2hr = HTMwd2com * HTMcom2hr;
    HTMwd2hl = HTMwd2com * HTMcom2hl;
    
    // hip positions in world frame
    hr_wd_p = HTMwd2hr.col(3).head(3);
    hl_wd_p = HTMwd2hl.col(3).head(3);

    // hip to line foot positions in world frame
    h2lf1R_wd_p = lf1R - hr_wd_p;
    h2lf2R_wd_p = lf2R - hr_wd_p;
    h2lf1L_wd_p = lf1L - hl_wd_p;
    h2lf2L_wd_p = lf2L - hl_wd_p;
    
    // convert line foot positions to body frame
    h2lf1R_b_p = R_curr.transpose() * h2lf1R_wd_p;
    h2lf2R_b_p = R_curr.transpose() * h2lf2R_wd_p;
    h2lf1L_b_p = R_curr.transpose() * h2lf1L_wd_p;
    h2lf2L_b_p = R_curr.transpose() * h2lf2L_wd_p;
    
    // compute joint positions using leg inverse kinematics function
    q_leg_R = SRB_Leg_IK(srb_params, h2lf1R_b_p, h2lf2R_b_p);
    q_leg_L = SRB_Leg_IK(srb_params, h2lf1L_b_p, h2lf2L_b_p);
    
    q_leg.row(0) = q_leg_R;
    q_leg.row(1) = q_leg_L;
    return q_leg;
}

void dash_kin::SRB_Kin(MatrixXd& q, MatrixXd& qd, MatrixXd* Jv_mat, SRB_Params srb_params, VectorXd x, MatrixXd lfv, MatrixXd lfdv)
{
    // This function converts SRB states and end-effector positions to
    // joint-space positions and velocities (optional). It also computes
    // end-effector (points along line foot) jacobians.  
    // Accounts for lack of leg-dynamics in model.

    // Parameters
    double thigh_length = srb_params.thigh_length; // thigh length in m (L1)
    double calf_length = srb_params.calf_length; // calf length in m (L2)
    double foot_length = srb_params.foot_length; // toe length in m (L3)
    double heel_length = srb_params.heel_length; // heel length in m (L4)
    int num_end_effector_pts = NUM_EE_POINTS; // two per foot
    int position_vec_size = POSITION_VEC_SIZE; // x, y, z components
    int right_end_effector_idx = RIGHT_EE_IDX; // idx of front right line foot pt
    int left_end_effector_idx = LEFT_EE_IDX; // idx of front left line foot pt
    int leg_DoF = LEG_DOF; // leg number of degrees of freedom

    // construct leg parameter vector
    VectorXd p_leg(4);
    p_leg << thigh_length, calf_length, foot_length, heel_length; 

    // get SRB states
    Vector3d pc_curr = x.segment<3>(0);
    Vector3d dpc_curr = x.segment<3>(3);
    Matrix3d R_curr = Eigen::Map<Matrix3d>(x.segment(6,9).data());
    Vector3d wb_curr = x.segment<3>(15);

    // SRB IK
    q = SRB_IK(srb_params,pc_curr,R_curr,lfv);
    
    VectorXd qr = q.row(0);
    VectorXd ql = q.row(1);

    MatrixXd J_mat[4];
    J_mat[0] = MatrixXd::Zero(6,5); // initialize to zero
    J_mat[1] = MatrixXd::Zero(6,5); // initialize to zero
    J_mat[2] = MatrixXd::Zero(6,5); // initialize to zero
    J_mat[3] = MatrixXd::Zero(6,5); // initialize to zero

    
    J_mat[0] = fcn_lf1_J(qr, p_leg); // line foot pt (front right)
    J_mat[1] = fcn_lf2_J(qr, p_leg); // line foot pt (back right)
    J_mat[2] = fcn_lf1_J(ql, p_leg); // line foot pt (front left)
    J_mat[3] = fcn_lf2_J(ql, p_leg); // line foot pt (back left)
    
    MatrixXd r_mat = lfv.transpose() - pc_curr.replicate(1, num_end_effector_pts);
    // calculate joint velocities
    Vector3d ww_curr = R_curr*wb_curr; // angular velocity in world frame
    VectorXd qd_b(6);
    qd_b << dpc_curr, ww_curr; // floating base velocities
    MatrixXd lfd_mat = lfdv.transpose(); // matrix form of end-effector velocities
    
    VectorXd r_mat_right = r_mat.col(right_end_effector_idx); // right end-effector positions
    VectorXd lfd_mat_right = lfd_mat.col(right_end_effector_idx); // right end-effector velocities
    VectorXd qdr = calc_joint_vel(qd_b, R_curr, r_mat_right, lfd_mat_right, J_mat[right_end_effector_idx]); // right leg joint velocities
    
    VectorXd r_mat_left = r_mat.col(left_end_effector_idx); // left end-effector positions
    VectorXd lfd_mat_left = lfd_mat.col(left_end_effector_idx); // left end-effector velocities
    VectorXd qdl = calc_joint_vel(qd_b, R_curr, r_mat_left, lfd_mat_left, J_mat[left_end_effector_idx]); // left leg joint velocities

    qd.row(0) = qdr;
    qd.row(1) = qdl; // joint velocities

    for(int i=0;i<4;i++)
    {
        Jv_mat[i] = J_mat[i].topRows(3);
    }

}

VectorXd dash_kin::calc_joint_vel(VectorXd qd_b, Matrix3d Rwb,Vector3d x_torso, VectorXd x_dot_trans, MatrixXd J_leg)
{
    // Calculate joint velocities for one leg
    // For each leg the calculation is as following:
    // qd = A^-1*(x_dot - J_b*qd_b)
    // A = [Rwb 0;0 Rwb]*J_leg

    // append zero rotational end effector velocity (for now)
    VectorXd x_dot_rot = VectorXd::Zero(3);
    VectorXd x_dot(6);
    x_dot << x_dot_trans, x_dot_rot;

    // Calculate Jacobian from floating base to end effector
    MatrixXd J_b(6, 6);
    J_b.block(0, 0, 3, 3) = Matrix3d::Identity();
    J_b.block(0, 3, 3, 3) = -dash_utils::hatMap(x_torso);
    J_b.block(3, 0, 3, 3) = Matrix3d::Zero();
    J_b.block(3, 3, 3, 3) = Matrix3d::Identity();

    // Calculate A
    MatrixXd transform(6, 6);
    transform.block(0, 0, 3, 3) = Rwb;
    transform.block(0, 3, 3, 3) = Matrix3d::Zero();
    transform.block(3, 0, 3, 3) = Matrix3d::Zero();
    transform.block(3, 3, 3, 3) = Rwb;

    MatrixXd A = transform*J_leg;
    
    // Calculate joint velocities
    
    MatrixXd A_pinv = A.completeOrthogonalDecomposition().pseudoInverse();
    
    VectorXd qd = A_pinv*(x_dot - J_b*qd_b);
    return qd;
}