#include <Eigen/Dense>
#include <math.h>
#include "srbm_structs.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>

using namespace std;

namespace  dash_utils 
{
    Vector3d calc_EA(Matrix3d R);
    Vector3d calc_dEA(Matrix3d R, Vector3d wb);
    void gen_trapz_traj(double max_speed, vector<double> &t, vector<double> &x);
    Matrix3d hatMap(Vector3d a);

    void Human_teleop_ref_get_step_placement_data(Teleop_Ref teleop_ref);
    void LIP_dyn_ref(double t_step, double omega, double x0, double xd0, double& x, double& xd);
    Teleop_2DLIP_Traj_Data read_teleop_2DLIP_traj(const char * data_txt_file); // TODO, function not finished

    Matrix3d rx(double phi);
    Matrix3d ry(double theta);
    Matrix3d rz(double psi);

    Vector2d sinu_ref_traj(double t, Vector3d sinu_traj_params);
    Vector2d sw_leg_ref_xy(double s, Vector2d init, Vector2d final);
    Vector2d sw_leg_ref_z(double s, double zcl, double H);

}