#include "srbm_utilities.h"

#define M_PI 3.14159265358979323846

using namespace Eigen;
/*
This function calculates euler angles from a rotation matrix
*/
Vector3d dash_utils::calc_EA(Matrix3d R) {
  double R11 = R(0, 0);
  double R21 = R(1, 0);
  double R31 = R(2, 0);
  double R12 = R(0, 1);
  double R32 = R(2, 1);
  double R13 = R(0, 2);
  double R33 = R(2, 2); 

  Vector3d EA1;

  if (R31 != 1 && R31 != -1) {
    double theta1 = -asin(R31);
    double theta2 = M_PI - theta1;
    double phi1 = atan2(R32 / cos(theta1), R33 / cos(theta1));
    double phi2 = atan2(R32 / cos(theta2), R33 / cos(theta2));
    double psi1 = atan2(R21 / cos(theta1), R11 / cos(theta1));
    double psi2 = atan2(R21 / cos(theta2), R11 / cos(theta2));

    EA1 << phi1, theta1, psi1;
  } else {
    double psi1 = 0;
    if (R31 == -1) {
      double theta1 = M_PI / 2;
      double phi1 = psi1 + atan2(R12, R13);
      EA1 << phi1, theta1, psi1;
    } else {
      double theta1 = -M_PI / 2;
      double phi1 = -psi1 + atan2(-R12, -R13);
      EA1 << phi1, theta1, psi1;
    }
  }

  return EA1;
}

/*
This function calculates euler angle derivatives from a rotation matrix
and the angular velocity vector
*/
Vector3d dash_utils::calc_dEA(Matrix3d R, Vector3d wb) {
  // Calculate Euler angles
  Vector3d EA = calc_EA(R);

  // Get Euler angles
  double thetaR = EA(1);
  double psiR = EA(2);

  // Calculate dEA
  Vector3d w = R * wb;
  Matrix3d T_EA;
  T_EA << cos(psiR)/cos(thetaR), sin(psiR)/cos(thetaR), 0,
      -sin(psiR), cos(psiR), 0,
      cos(psiR)*tan(thetaR), sin(psiR)*tan(thetaR), 1;
  Vector3d dEA = T_EA * w;

  return dEA;
}

void dash_utils::gen_trapz_traj(double max_speed, vector<double> &t, vector<double> &x) 
{
    int err = 0;

    // Parameters
    double a = max_speed;  // Amplitude
    double m = 4; // Time Period
    double l = 4; // Horizontal Spread
    double c = max_speed; // Vertical Spread
    double dt = 0.05; // Time increment

    // Sample points
    for (double i = 0; i <= 10.0; i += dt) {
    t.push_back(i);
    }

    // Trapezoidal trajectory
    for (int i = 0; i < t.size(); i++) {
        x.push_back(a / M_PI * (asin(sin((M_PI / m) * t[i] + l)) + acos(cos((M_PI / m) * t[i] + l))) - a / 2 + c);
    }

    double min_num = *min_element(x.begin(), x.end());

    for (double& num : x) {
        num -= min_num;
    }

    // Modify trajectory
    int first_zero_idx = 0;
    for (int i = 0; i < x.size(); i++) {
        if (x[i] <= 0.0) {
            first_zero_idx = i;
            break;
        }
    }

    vector<double> t_mod, x_mod;
    t_mod.push_back(0.0);
    x_mod.push_back(0.0);
    for (int i = first_zero_idx; i < t.size(); i++) {
        t_mod.push_back(t[i]);
        x_mod.push_back(x[i]);
    }

    t = t_mod;
    x = x_mod;

}

Matrix3d dash_utils::hatMap(Vector3d a) {
    Matrix3d H;
    H << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;
    return H;
}

void dash_utils::Human_teleop_ref_get_step_placement_data(Teleop_Ref teleop_ref)
{
    // Get signals
    VectorXd time = teleop_ref.time; // time
    VectorXd FSM = teleop_ref.FSM_R;
    VectorXd xCoM = teleop_ref.xR_curr; // x-CoM
    VectorXd yCoM = teleop_ref.yR_curr; // y-CoM
    VectorXd xlf_R = teleop_ref.fxR_R; // foot-position of right foot
    VectorXd xlf_L = teleop_ref.fxR_L; // foot-position of left foot
    VectorXd ylf_R = teleop_ref.fyR_R; // foot-position of right foot
    VectorXd ylf_L = teleop_ref.fyR_L; // foot-position of left foot

    // compute swing-leg to CoM position vectors
    VectorXd xsw2CoM_R = xCoM - xlf_R;
    VectorXd xsw2CoM_L = xCoM - xlf_L;
    VectorXd ysw2CoM_R = yCoM - ylf_R;
    VectorXd ysw2CoM_L = yCoM - ylf_L;

    // number of data points
    int num_data_pts = time.size();

    // initialize loop variables
    int FSM_prev = 0; // always start at DSP
    int k = 0; // step count
    MatrixXd step_placement_data(100, 6);

    double t_step_beg, xsw2CoM, ysw2CoM;

    for (int sim_idx = 0; sim_idx < num_data_pts; ++sim_idx)
    {

        // get current time
        double time_curr = time(sim_idx);

        // get current FSM
        int FSM_curr = FSM(sim_idx);

        // check for DSP to SSP transition
        if ((FSM_curr == -1 || FSM_curr == 1) && (FSM_prev == 0))
        {
            // update step counter
            k = k + 1;

            // update beginning of step time
            t_step_beg = time_curr;

            // update step placement data
            step_placement_data(k, 0) = k;
            step_placement_data(k, 1) = t_step_beg;

        }

        // check for SSP to DSP transition
        if (FSM_curr == 0 && (FSM_prev == -1 || FSM_prev == 1))
        {

            // update end of step time
            double t_step_ends = time_curr;

            // get desired step placement
            if (FSM_prev == -1) 
            {
                // SSP_R
                xsw2CoM = xsw2CoM_L(sim_idx);
                ysw2CoM = ysw2CoM_L(sim_idx);

            } else if (FSM_prev == 1) 
            {
                // SSP_L
                xsw2CoM = xsw2CoM_R(sim_idx);
                ysw2CoM = ysw2CoM_R(sim_idx);            
            }

            // update step placement data
            step_placement_data(k, 2) = t_step_ends;    
            step_placement_data(k, 3) = t_step_ends - t_step_beg;
            step_placement_data(k, 4) = xsw2CoM;
            step_placement_data(k, 5) = ysw2CoM;

        }

        // update FSM 
        FSM_prev = FSM_curr;

        // TODO_DASH: what does the last line in the matlab function do

    }
}

void dash_utils::LIP_dyn_ref(double t_step, double omega, double x0, double xd0, double& x, double& xd) {
    double c1 = (0.5) * (x0 + (1 / omega) * xd0);
    double c2 = (0.5) * (x0 - (1 / omega) * xd0);
    x = c1 * exp(omega * t_step) + c2 * exp(-omega * t_step);
    xd = omega * (c1 * exp(omega * t_step) - c2 * exp(-omega * t_step));
}

/*
This function reads raw data of whole-body human dynamic telelocomotion experiments. 
*/
Teleop_2DLIP_Traj_Data dash_utils::read_teleop_2DLIP_traj(const char * data_txt_file)
{
    vector<string> data_vars = {"time", "Ts_prev",
                                "FSM_R", "FSM_H",
                                "xR_curr", "dxR_curr", "pxR_curr",
                                "yR_curr", "dyR_curr", "pyR_curr",
                                "xDCMH_ref_curr", "pxR_beg_step",
                                "fxR_R", "fyR_R", "fzR_R", "fxR_L", "fyR_L", "fzR_L",
                                "yH_curr", "dyH_curr", "pyH_curr",
                                "xH_curr", "pxH_curr",
                                "fxH_R", "fyH_R", "fzH_R", "fxH_L", "fyH_L", "fzH_L",
                                "FxH_spring", "FyH_hmi_comm", "FxH_hmi_comm", "FxR_ext",
                                "phiR_curr", "dphiR_curr", "MxR_curr"};

    Teleop_2DLIP_Traj_Data teleop_2DLIP_traj_data;

    // time data index
    int time_data_idx = 1;

    // number of data variables recorded
    int num_data_vars = data_vars.size();

    // open the file
    ifstream file(data_txt_file);

    // check if file is open
    if (!file.is_open()) {
        cout << "Failed to open file " << data_txt_file << endl;
    }

    // vector to store the text from the file
    vector<string> txt;
    string line;

    // read the file line by line
    while (getline(file, line))
    {
        txt.push_back(line);
    }

    // close the file
    file.close();

    // convert text to double
    vector<double> data;
    stringstream ss;
    for (string line : txt) 
    {
        ss.clear();
        ss.str(line);
        double value;
        while (ss >> value) {
            data.push_back(value);
        }
    }

    // time variable
    Eigen::VectorXd time(data.size() / num_data_vars);
    for (int i = 0; i < time.size(); i++) {
    time(i) = data[i * num_data_vars + time_data_idx - 1];
    }
    
    // TODO_DASH : function not finished?
    return teleop_2DLIP_traj_data;
}

/*
This function returns the rotation matrix for a rotation around the x-axis
phi is the angle of rotation
*/
Matrix3d dash_utils::rx(double phi)
{
    // Compute sin() and cos()
    double c = cos(phi);
    double s = sin(phi);

    // Rotation matrix
    Matrix3d Rx;
    Rx << 1, 0, 0,
          0, c, -s,
          0, s, c;

    return Rx;
}

/*
This function returns the rotation matrix for a rotation around the y-axis
theta is the angle of rotation
*/
Matrix3d dash_utils::ry(double theta) 
{
    // Compute sin() and cos()
    double c = cos(theta);
    double s = sin(theta);

    // Rotation matrix
    Matrix3d Ry;
    Ry << c, 0, s,
          0, 1, 0,
         -s, 0, c;

    return Ry;
}

/*
This function returns the rotation matrix for a rotation around the z-axis
psi is the angle of rotation
*/
Matrix3d dash_utils::rz(double psi) {
  // compute sin() and cos()
  double c = cos(psi);
  double s = sin(psi);

  // rotation matrix
  Matrix3d Rz;
  Rz << c, -s, 0,
        s, c, 0,
        0, 0, 1;
  return Rz;
}

/*
Sinusoidal trajectory to track by SRB PD controller
*/
Vector2d dash_utils::sinu_ref_traj(double t, Vector3d sinu_traj_params) {
    // Parameters
    double w = sinu_traj_params(0);
    double A = sinu_traj_params(1);
    double theta = sinu_traj_params(2);

    // position reference
    double p_ref = A * sin(w * t + theta);

    // velocity reference
    double v_ref = A * w * cos(w * t + theta);

    // create vector
    Vector2d ref;
    ref << p_ref, v_ref;
    return ref;
}

/*
Swing-leg trajectory that maps the swing-leg to CoM vector from its
initial to its desired in the x-y plane
*/
Vector2d dash_utils::sw_leg_ref_xy(double s, Vector2d init, Vector2d final) {
    // position and velocity reference trajectories
    Vector2d p_ref = (0.5) * ((1 + cos(M_PI * s)) * init + (1 - cos(M_PI * s)) * final);
    Vector2d v_ref = (0.5) * M_PI * sin(M_PI * s) * (final - init);

    // create vector
    Vector2d ref;
    ref << p_ref, v_ref;
    return ref;
}

/* 
Swing-leg trajectory that maps the swing-leg to CoM vector from its
initial to its desired in the z (vertical) direction
*/
Vector2d sw_leg_ref_z(double s, double zcl, double H) {
    // position and velocity reference trajectories
    double p_ref = 4 * zcl * pow(s - (0.5), 2) + H - zcl;
    double v_ref = 4 * zcl * (2 * s - 1);

    // create vector
    Vector2d ref;
    ref << p_ref, v_ref;
    return ref;
}
