#include "utilities.h"

#define M_PI 3.14159265358979323846

std::string _foldername = "";

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
      double theta1 = M_PI / 2.0;
      double phi1 = psi1 + atan2(R12, R13);
      EA1 << phi1, theta1, psi1;
    } else {
      double theta1 = -M_PI / 2.0;
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


VectorXd dash_utils::calc_wb(Vector3d dEA, VectorXd EA) {
  // Extract Euler angles from input vector
  double phi = EA(0);
  double theta = EA(1);
  double psi = EA(2);

  // Calculate rotation matrix from Euler angles
  Matrix3d Rmat;
  Rmat = AngleAxisd(phi, Vector3d::UnitX())
        * AngleAxisd(theta, Vector3d::UnitY())
        * AngleAxisd(psi, Vector3d::UnitZ());

  // Calculate inverse of T_EA
  double cos_theta = cos(theta);
  Matrix3d T_EA;
//   inv_T_EA << cos(psi)/cos_theta, -sin(psi), cos(psi)*tan(theta)/cos_theta,
//               sin(psi)/cos(theta), cos(psi), sin(psi)*tan(theta)/cos(theta),
//               0, 0, 1;
T_EA << cos(psi)/cos(theta), sin(psi)/cos(theta), 0,
      -sin(psi), cos(psi), 0,
      cos(psi)*tan(theta), sin(psi)*tan(theta), 1;

  // Calculate w from dEA using the inverse of T_EA
  Vector3d w = T_EA.inverse() * dEA;

  // Calculate wb from w and R
  VectorXd wb(3);
  wb = Rmat.transpose() * w;

  return wb;
}

void dash_utils::gen_trapz_traj(double max_speed, Eigen::VectorXd &t, Eigen::VectorXd &x) 
{
    int err = 0;

    // Parameters
    double a = max_speed;  // Amplitude
    double m = 4; // Time Period
    double l = 4; // Horizontal Spread
    double c = max_speed; // Vertical Spread
    double dt = 0.05; // Time increment

    // Sample points
    int num_samples = static_cast<int>(std::round(10.0 / dt));
    t.resize(num_samples);
    for (int i = 0; i < num_samples; i++) {
        t(i) = i * dt;
    }

    // Trapezoidal trajectory
    x.resize(num_samples);
    for (int i = 0; i < num_samples; i++) {
        x(i) = a / M_PI * (asin(sin((M_PI / m) * t(i) + l)) + acos(cos((M_PI / m) * t(i) + l))) - a / 2 + c;
    }

    double min_num = x.minCoeff();

    x.array() -= min_num;

    // Modify trajectory
    int first_zero_idx = 0;
    for (int i = 0; i < num_samples; i++) {
        if (x(i) <= 0.0) {
            first_zero_idx = i;
            break;
        }
    }

    VectorXd new_t(num_samples - first_zero_idx + 1);
    VectorXd new_x(num_samples - first_zero_idx + 1);
    new_t(0) = 0.0;
    new_x(0) = 0.0;
    for (int i = first_zero_idx; i < x.size(); i++)
    {
        new_t(i - first_zero_idx + 1) = t(i);
        new_x(i - first_zero_idx + 1) = x(i);
    }
    t = new_t;
    x = new_x;
}

Matrix3d dash_utils::hatMap(VectorXd a) {
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
    double c1 = (0.5) * (x0 + (1.0 / omega) * xd0);
    double c2 = (0.5) * (x0 - (1.0 / omega) * xd0);
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
Vector2d dash_utils::sw_leg_ref_xy(double s, double init, double final) {
    // position and velocity reference trajectories
    double p_ref = (0.5) * ((1 + cos(M_PI * s)) * init + (1 - cos(M_PI * s)) * final);
    double v_ref = (0.5) * M_PI * sin(M_PI * s) * (final - init);

    // create vector
    Vector2d ref;
    ref << p_ref, v_ref;
    return ref;
}

/* 
Swing-leg trajectory that maps the swing-leg to CoM vector from its
initial to its desired in the z (vertical) direction
*/
Vector2d dash_utils::sw_leg_ref_z(double s, double zcl, double H) {
    // position and velocity reference trajectories
    double p_ref = 4 * zcl * pow(s - (0.5), 2) + H - zcl;
    double v_ref = 4 * zcl * (2 * s - 1);

    // create vector
    Vector2d ref;
    ref << p_ref, v_ref;
    return ref;
}


VectorXd dash_utils::flatten(MatrixXd matrix) 
{
    int rows = matrix.rows();
    int cols = matrix.cols();

    VectorXd flattened(rows * cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            flattened(i * cols + j) = matrix(i, j);
        }
    }
    return flattened;
}

void dash_utils::setOutputFolder(const std::string& foldername)
{
    _foldername = foldername;
}

void dash_utils::writeVectorToCsv(const VectorXd& vec, const std::string& filename)
{
    std::ofstream file(_foldername + filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < vec.size(); i++)
    {
        file << vec(i);
        if (i < vec.size() - 1)
            file << ","; // Add comma delimiter for all but the last element
    }
    file << "\n";

    file.close();
}

void dash_utils::writeMatrixToCsv(const MatrixXd& mat, const std::string& filename)
{
    std::ofstream file(_foldername + filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    for (int i = 0; i < mat.rows(); i++)
    {
        for (int j = 0; j < mat.cols(); j++)
        {
            file << mat(i, j);
            if (j < mat.cols() - 1)
                file << ","; // Add comma delimiter for all but the last element in a row
        }
        if (i < mat.rows() - 1)
            file << std::endl; // Add newline delimiter for all but the last row
    }

    file.close();
}

void dash_utils::writeHumanDynDataToCsv(const Human_dyn_data& data, const std::string& filename) {
    std::ofstream file(_foldername + filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    const char* delimiter = ",";
    const char* newline = "\n";

    file << data.xH << delimiter
         << data.dxH << delimiter
         << data.pxH << delimiter
         << data.yH << delimiter
         << data.dyH << delimiter
         << data.pyH << delimiter
         << data.fxH_R << delimiter
         << data.fyH_R << delimiter
         << data.fzH_R << delimiter
         << data.fxH_L << delimiter
         << data.fyH_L << delimiter
         << data.fzH_L << delimiter
         << data.fdxH_R << delimiter
         << data.fdyH_R << delimiter
         << data.fdzH_R << delimiter
         << data.fdxH_L << delimiter
         << data.fdyH_L << delimiter
         << data.fdzH_L << delimiter
         << data.FxH_hmi << delimiter
         << data.FyH_hmi << delimiter
         << data.FxH_spring << newline;

    file.close();
}

void dash_utils::writeTrajPlannerDataToCsv(const Traj_planner_dyn_data& data, const std::string& filename) {
    std::ofstream file(_foldername + filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    const char* delimiter = ",";
    const char* newline = "\n";

    file << (data.stepping_flg ? 1 : 0) << delimiter
         << data.T_step << delimiter
         << data.t_sw_start << delimiter
         << data.t_dsp_start << delimiter
         << data.next_SSP << delimiter
         << data.step_width << delimiter
         << data.st2CoM_beg_step.x() << delimiter
         << data.st2CoM_beg_step.y() << delimiter
         << data.st2CoM_beg_step.z() << delimiter
         << data.sw2CoM_beg_step.x() << delimiter
         << data.sw2CoM_beg_step.y() << delimiter
         << data.sw2CoM_beg_step.z() << delimiter
         << data.xLIP_init.x() << delimiter
         << data.xLIP_init.y() << delimiter
         << data.sw_beg_step.x() << delimiter
         << data.sw_beg_step.y() << delimiter
         << data.sw_beg_step.z() << delimiter
         << data.human_leg_joystick_pos_beg_step.x() << delimiter
         << data.human_leg_joystick_pos_beg_step.y() << delimiter
         << data.human_leg_joystick_pos_beg_step.z() << delimiter
         << data.sigma1H << delimiter
         << (data.left_in_contact ? 1 : 0) << delimiter
         << (data.right_in_contact ? 1 : 0) << delimiter
         << data.left_off_gnd_cnt << delimiter
         << data.right_off_gnd_cnt << delimiter
         << data.x_HWRM << delimiter
         << data.dx_HWRM << delimiter
         << data.x_plus_HWRM.x() << delimiter
         << data.x_plus_HWRM.y() << delimiter
         << data.uk_HWRM << delimiter
         << data.st_beg_step.x() << delimiter
         << data.st_beg_step.y() << delimiter
         << data.st_beg_step.z() << newline;

    file.close();
}

void dash_utils::writeSRBParamsToTxt(const SRB_Params& params, const std::string& filename)
{
    std::ofstream file(_foldername + filename);
    if (file.is_open())
    {
        file << params.dt << std::endl;
        file << params.init_type << std::endl;
        file << params.g << std::endl;
        file << params.mu << std::endl;
        file << params.m << std::endl;
        file << params.hLIP << std::endl;
        file << params.Ib(0, 0) << "," << params.Ib(0, 1) << "," << params.Ib(0, 2) << ",";
        file << params.Ib(1, 0) << "," << params.Ib(1, 1) << "," << params.Ib(1, 2) << ",";
        file << params.Ib(2, 0) << "," << params.Ib(2, 1) << "," << params.Ib(2, 2) << std::endl;
        file << params.W << std::endl;
        file << params.L << std::endl;
        file << params.H << std::endl;
        file << params.thigh_length << std::endl;
        file << params.calf_length << std::endl;
        file << params.foot_length << std::endl;
        file << params.heel_length << std::endl;
        file << params.CoM2H_z_dist << std::endl;
        file << params.planner_type << std::endl;
        file << params.T << std::endl;
        file << params.x_sinu_traj_params(0) << "," << params.x_sinu_traj_params(1) << "," << params.x_sinu_traj_params(2) << std::endl;
        file << params.y_sinu_traj_params(0) << "," << params.y_sinu_traj_params(1) << "," << params.y_sinu_traj_params(2) << std::endl;
        file << params.z_sinu_traj_params(0) << "," << params.z_sinu_traj_params(1) << "," << params.z_sinu_traj_params(2) << std::endl;
        file << params.roll_sinu_traj_params(0) << "," << params.roll_sinu_traj_params(1) << "," << params.roll_sinu_traj_params(2) << std::endl;
        file << params.pitch_sinu_traj_params(0) << "," << params.pitch_sinu_traj_params(1) << "," << params.pitch_sinu_traj_params(2) << std::endl;
        file << params.yaw_sinu_traj_params(0) << "," << params.yaw_sinu_traj_params(1) << "," << params.yaw_sinu_traj_params(2) << std::endl;
        for (int i = 0; i < params.vx_des_t.size(); i++) {
            file << params.vx_des_t(i) << ",";
        }
        file << std::endl;
        for (int i = 0; i < params.vx_des_vx.size(); i++) {
            file << params.vx_des_vx(i) << ",";
        }
        file << std::endl;
        file << params.t_beg_stepping << std::endl;
        file << params.t_end_stepping << std::endl;
        file << params.zcl << std::endl;
        file << params.xDCMH_deadband << std::endl;
        file << params.KxDCMH << std::endl;
        file << params.Kx_DCM_mult << std::endl;
        file << params.Ky_DCM_mult << std::endl;
        file << params.T_DSP << std::endl;
        file << params.lmaxR << std::endl;
        file << params.Kp_xR << std::endl;
        file << params.Kd_xR << std::endl;
        file << params.Kp_yR << std::endl;
        file << params.Kd_yR << std::endl;
        file << params.Kp_zR << std::endl;
        file << params.Kd_zR << std::endl;
        file << params.Kp_phiR << std::endl;
        file << params.Kd_phiR << std::endl;
        file << params.Kp_thetaR << std::endl;
        file << params.Kd_thetaR << std::endl;
        file << params.Kp_psiR << std::endl;
        file << params.Kd_psiR << std::endl;
        file << params.QP_opt_sol_type << std::endl;
        file << params.W_wrench << std::endl;
        file << params.W_u_minus_u0_norm << std::endl;
        file << params.Act_const_type << std::endl;
        file << params.tau_m_max << std::endl;
        file << params.tau_m_stall << std::endl;
        file << params.alpha_m << std::endl;
        file << params.beta_trans << std::endl;
        file << params.gamma_trans << std::endl;
        file << params.Fz_min_QP << std::endl;
        file << params.Fz_min_FSM << std::endl;
        file << params.q1_lim(0) << "," << params.q1_lim(1) << std::endl;
        file << params.q2_lim(0) << "," << params.q2_lim(1) << std::endl;
        file.close();

    }
    else{
        std::cerr << "Error writing SRB_Params" << std::endl;
    }
}

std::vector<Human_dyn_data> dash_utils::readHumanDynDataFromFile(const std::string& filename) 
{
    std::ifstream file(filename);
    std::vector<Human_dyn_data> dataVector;

    if (!file.is_open()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return dataVector; // Return empty vector on failure
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        Human_dyn_data data;

        std::string value;
        std::getline(ss, value, ',');
        data.xH = std::stof(value);

        std::getline(ss, value, ',');
        data.dxH = std::stof(value);

        std::getline(ss, value, ',');
        data.pxH = std::stof(value);

        std::getline(ss, value, ',');
        data.yH = std::stof(value);

        std::getline(ss, value, ',');
        data.dyH = std::stof(value);

        std::getline(ss, value, ',');
        data.pyH = std::stof(value);

        std::getline(ss, value, ',');
        data.fxH_R = std::stof(value);

        std::getline(ss, value, ',');
        data.fyH_R = std::stof(value);

        std::getline(ss, value, ',');
        data.fzH_R = std::stof(value);

        std::getline(ss, value, ',');
        data.fxH_L = std::stof(value);

        std::getline(ss, value, ',');
        data.fyH_L = std::stof(value);

        std::getline(ss, value, ',');
        data.fzH_L = std::stof(value);

        std::getline(ss, value, ',');
        data.fdxH_R = std::stof(value);

        std::getline(ss, value, ',');
        data.fdyH_R = std::stof(value);

        std::getline(ss, value, ',');
        data.fdzH_R = std::stof(value);

        std::getline(ss, value, ',');
        data.fdxH_L = std::stof(value);

        std::getline(ss, value, ',');
        data.fdyH_L = std::stof(value);

        std::getline(ss, value, ',');
        data.fdzH_L = std::stof(value);

        std::getline(ss, value, ',');
        data.FxH_hmi = std::stof(value);

        std::getline(ss, value, ',');
        data.FyH_hmi = std::stof(value);

        std::getline(ss, value, ',');
        data.FxH_spring = std::stof(value);

        dataVector.push_back(data);
        }

        file.close();
        return dataVector;

}

// Function to convert foot position in world frame to hip frame
Vector3d dash_utils::worldToHip(Vector3d foot_pos_world, Vector3d hip_pos_world, Vector3d hip_orient_world)
{
    // Calculate transformation matrix from world frame to torso frame
    double tx = hip_pos_world(0);
    double ty = hip_pos_world(1);
    double tz = hip_pos_world(2);
    double rx = hip_orient_world(0);
    double ry = hip_orient_world(1);
    double rz = hip_orient_world(2);

    Matrix4d T_world_to_hip;
    T_world_to_hip << cos(ry)*cos(rz), -cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz), sin(rx)*sin(rz) + cos(rx)*sin(ry)*cos(rz), tx,
                        cos(ry)*sin(rz), cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz), -sin(rx)*cos(rz) + cos(rx)*sin(ry)*sin(rz), ty,
                        -sin(ry), sin(rx)*cos(ry), cos(rx)*cos(ry), tz,
                        0, 0, 0, 1;

    // Convert foot position to hip frame
    Vector4d foot_pos_world_homogeneous;
    foot_pos_world_homogeneous << foot_pos_world, 1.0;
    Vector4d foot_pos_hip_homogeneous = T_world_to_hip.inverse() * foot_pos_world_homogeneous;
    Vector3d foot_pos_hip = foot_pos_hip_homogeneous.head(3);

    return foot_pos_hip;
}

Vector3d dash_utils::hipToWorld(Vector3d vector_hip, Vector3d hip_pos_world, Vector3d hip_orient_world)
{
    // Calculate transformation matrix from world frame to hip frame
    double tx = hip_pos_world(0);
    double ty = hip_pos_world(1);
    double tz = hip_pos_world(2);
    double rx = hip_orient_world(0);
    double ry = hip_orient_world(1);
    double rz = hip_orient_world(2);

    Matrix4d T_world_to_hip;
    T_world_to_hip << cos(ry)*cos(rz), -cos(rx)*sin(rz) + sin(rx)*sin(ry)*cos(rz), sin(rx)*sin(rz) + cos(rx)*sin(ry)*cos(rz), tx,
                        cos(ry)*sin(rz), cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz), -sin(rx)*cos(rz) + cos(rx)*sin(ry)*sin(rz), ty,
                        -sin(ry), sin(rx)*cos(ry), cos(rx)*cos(ry), tz,
                        0, 0, 0, 1;

    // Convert vector from hip frame to world frame
    Vector4d vector_hip_homogeneous;
    vector_hip_homogeneous << vector_hip, 1.0;
    Vector4d vector_world_homogeneous = T_world_to_hip * vector_hip_homogeneous;
    Vector3d vector_world = vector_world_homogeneous.head(3);

    return vector_world;
}

// Define a function to parse the JSON file and write to an srb_params struct
void dash_utils::parse_json_to_srb_params(const std::string& json_file_path, SRB_Params& params) {
  // Open the JSON file
  std::ifstream json_file(json_file_path);
  if (!json_file.is_open()) {
    std::cerr << "Error: could not open JSON file for SRB_Params\n";
    return;
  }

  // Parse the JSON file
  nlohmann::json json_data;
  try {
    json_file >> json_data;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }

  // Extract the values and write to the srb_params struct
  try {
    params.Kp_xR = json_data["srb_params"]["Kp_xR"].get<double>();
    params.Kd_xR = json_data["srb_params"]["Kd_xR"].get<double>();
    params.Kp_yR = json_data["srb_params"]["Kp_yR"].get<double>();
    params.Kd_yR = json_data["srb_params"]["Kd_yR"].get<double>();
    params.Kp_zR = json_data["srb_params"]["Kp_zR"].get<double>();
    params.Kd_zR = json_data["srb_params"]["Kd_zR"].get<double>();
    params.Kp_phiR = json_data["srb_params"]["Kp_phiR"].get<double>();
    params.Kd_phiR = json_data["srb_params"]["Kd_phiR"].get<double>();
    params.Kp_thetaR = json_data["srb_params"]["Kp_thetaR"].get<double>();
    params.Kd_thetaR = json_data["srb_params"]["Kd_thetaR"].get<double>();
    params.Kp_psiR = json_data["srb_params"]["Kp_psiR"].get<double>();
    params.Kd_psiR = json_data["srb_params"]["Kd_psiR"].get<double>();

    params.Kx_DCM_mult = json_data["srb_params"]["Kx_DCM_mult"].get<double>();
    params.Ky_DCM_mult = json_data["srb_params"]["Ky_DCM_mult"].get<double>();

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }
  try {
    params.T = json_data["srb_params"]["Step_Period"].get<double>();
    params.zcl = json_data["srb_params"]["Step_Height"].get<double>();
    params.des_walking_speed = json_data["srb_params"]["Walking_Speed"].get<double>();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }
}

void dash_utils::parse_json_to_pd_params(const std::string& json_file_path, Joint_PD_config& swing, Joint_PD_config& posture) {
  // Open the JSON file
  std::ifstream json_file(json_file_path);
  if (!json_file.is_open()) {
    std::cerr << "Error: could not open JSON file for PD params\n";
    return;
  }

  // Parse the JSON file
  nlohmann::json json_data;
  try {
    json_file >> json_data;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }

  // Extract the values and write to the srb_params struct
  try {
    posture.hip_yaw_Kp = json_data["Posture_Control"]["Hip_Yaw_Kp"].get<double>();
    posture.hip_yaw_Kd = json_data["Posture_Control"]["Hip_Yaw_Kd"].get<double>();
    posture.hip_roll_Kp = json_data["Posture_Control"]["Hip_Roll_Kp"].get<double>();
    posture.hip_roll_Kd = json_data["Posture_Control"]["Hip_Roll_Kd"].get<double>();
    posture.hip_pitch_Kp = json_data["Posture_Control"]["Hip_Pitch_Kp"].get<double>();
    posture.hip_pitch_Kd = json_data["Posture_Control"]["Hip_Pitch_Kd"].get<double>();
    posture.knee_Kp = json_data["Posture_Control"]["Knee_Kp"].get<double>();
    posture.knee_Kd = json_data["Posture_Control"]["Knee_Kd"].get<double>();
    posture.ankle_Kp = json_data["Posture_Control"]["Ankle_Kp"].get<double>();
    posture.ankle_Kd = json_data["Posture_Control"]["Ankle_Kd"].get<double>();

    printJointPDConfig(posture);
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }
  try {
    swing.hip_yaw_Kp = json_data["Swing_Control"]["Hip_Yaw_Kp"].get<double>();
    swing.hip_yaw_Kd = json_data["Swing_Control"]["Hip_Yaw_Kd"].get<double>();
    swing.hip_roll_Kp = json_data["Swing_Control"]["Hip_Roll_Kp"].get<double>();
    swing.hip_roll_Kd = json_data["Swing_Control"]["Hip_Roll_Kd"].get<double>();
    swing.hip_pitch_Kp = json_data["Swing_Control"]["Hip_Pitch_Kp"].get<double>();
    swing.hip_pitch_Kd = json_data["Swing_Control"]["Hip_Pitch_Kd"].get<double>();
    swing.knee_Kp = json_data["Swing_Control"]["Knee_Kp"].get<double>();
    swing.knee_Kd = json_data["Swing_Control"]["Knee_Kd"].get<double>();
    swing.ankle_Kp = json_data["Swing_Control"]["Ankle_Kp"].get<double>();
    swing.ankle_Kd = json_data["Swing_Control"]["Ankle_Kd"].get<double>();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return;
  }
}

void dash_utils::printJointPDConfig(const Joint_PD_config& config)
{
    std::cout << "hip_yaw_Kp: " << config.hip_yaw_Kp << std::endl;
    std::cout << "hip_yaw_Kd: " << config.hip_yaw_Kd << std::endl;
    std::cout << "hip_roll_Kp: " << config.hip_roll_Kp << std::endl;
    std::cout << "hip_roll_Kd: " << config.hip_roll_Kd << std::endl;
    std::cout << "hip_pitch_Kp: " << config.hip_pitch_Kp << std::endl;
    std::cout << "hip_pitch_Kd: " << config.hip_pitch_Kd << std::endl;
    std::cout << "knee_Kp: " << config.knee_Kp << std::endl;
    std::cout << "knee_Kd: " << config.knee_Kd << std::endl;
    std::cout << "ankle_Kp: " << config.ankle_Kp << std::endl;
    std::cout << "ankle_Kd: " << config.ankle_Kd << std::endl;
}

std::chrono::high_resolution_clock::time_point start_time;
std::chrono::high_resolution_clock::time_point end_time;
std::chrono::high_resolution_clock::time_point last_print_time;

std::chrono::high_resolution_clock::time_point start_sim_time;
std::chrono::high_resolution_clock::time_point end_sim_time;
std::chrono::high_resolution_clock::time_point last_sim_print_time;

void dash_utils::start_timer(){
    start_time = std::chrono::high_resolution_clock::now();
}

void dash_utils::start_sim_timer(){
    start_sim_time = std::chrono::high_resolution_clock::now();
}

void dash_utils::print_timer(){
    end_time = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    std::chrono::nanoseconds elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time);

    // Convert the elapsed time to milliseconds with a resolution of nanoseconds
    double elapsed_time_ms = static_cast<double>(elapsed_time.count()) / 1000000.0;

    std::chrono::nanoseconds elapsed_time_since_print = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - last_print_time);
    double elapsed_time_since_print_ms = static_cast<double>(elapsed_time_since_print.count()) / 1000000.0;
    
    if(elapsed_time_since_print_ms > 500){
        std::cout << "Elapsed time: " << elapsed_time_ms << " ms" << std::endl;
        last_print_time = end_time;
    }
}

double dash_utils::measure_sim_timer(){
    end_sim_time = std::chrono::high_resolution_clock::now();
    // Calculate the elapsed time
    std::chrono::nanoseconds elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_sim_time - start_sim_time);

    // Convert the elapsed time to milliseconds with a resolution of nanoseconds
    double elapsed_time_ms = static_cast<double>(elapsed_time.count()) / 1000000.0;

    std::chrono::nanoseconds elapsed_time_since_print = std::chrono::duration_cast<std::chrono::nanoseconds>(end_sim_time - last_sim_print_time);
    double elapsed_time_since_print_ms = static_cast<double>(elapsed_time_since_print.count()) / 1000000.0;
    
    if(elapsed_time_since_print_ms > 500){
        //std::cout << "Elapsed time: " << elapsed_time_ms << " ms" << std::endl;
        last_sim_print_time = end_sim_time;
    }
    return elapsed_time_ms/1000.0;
}

void dash_utils::unpack_data_from_hmi(Human_dyn_data& data, uint8_t* buffer)
{
    size_t size = sizeof(float) * 18; // Exclude: FxH_hmi, FyH_hmi, FxH_spring
    memcpy(&data, buffer, size);
}

void dash_utils::pack_data_to_hmi(uint8_t* buffer, Human_dyn_data data)
{
    float* float_buffer = reinterpret_cast<float*>(buffer);
    float_buffer[0] = data.FxH_hmi;
    float_buffer[1] = data.FyH_hmi;
    float_buffer[2] = data.FxH_spring;
    for (int i = 0; i < 12; i++) {
        buffer[12] += buffer[i];
    }
}

void dash_utils::pack_data_to_hmi_with_ctrls(uint8_t* buffer, Human_dyn_data data,bool ff,bool tare,float gain)
{
    float* float_buffer = reinterpret_cast<float*>(buffer);
    float_buffer[0] = data.FxH_hmi;
    float_buffer[1] = data.FyH_hmi;
    float_buffer[2] = data.FxH_spring;
    float_buffer[3] = (((int)ff) << 1) | ((int) tare);
    float_buffer[4] = gain;
    for (int i = 0; i < 20; i++) {
        buffer[20] += buffer[i];
    }
}

std::chrono::high_resolution_clock::time_point last_comms_print_time;

void dash_utils::print_human_dyn_data(const Human_dyn_data& data)
{
    std::chrono::high_resolution_clock::time_point time_now = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds elapsed_time_since_print = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now - last_comms_print_time);
    double elapsed_time_since_print_ms = static_cast<double>(elapsed_time_since_print.count()) / 1000000.0;
    if(elapsed_time_since_print_ms > 20)
    {
        last_comms_print_time = std::chrono::high_resolution_clock::now();
        std::cout << "xH: " << data.xH << std::endl;
        std::cout << "dxH: " << data.dxH << std::endl;
        std::cout << "pxH: " << data.pxH << std::endl;
        std::cout << "yH: " << data.yH << std::endl;
        std::cout << "dyH: " << data.dyH << std::endl;
        std::cout << "pyH: " << data.pyH << std::endl;
        std::cout << "fxH_R: " << data.fxH_R << std::endl;
        std::cout << "fyH_R: " << data.fyH_R << std::endl;
        std::cout << "fzH_R: " << data.fzH_R << std::endl;
        std::cout << "fxH_L: " << data.fxH_L << std::endl;
        std::cout << "fyH_L: " << data.fyH_L << std::endl;
        std::cout << "fzH_L: " << data.fzH_L << std::endl;
        std::cout << "fdxH_R: " << data.fdxH_R << std::endl;
        std::cout << "fdyH_R: " << data.fdyH_R << std::endl;
        std::cout << "fdzH_R: " << data.fdzH_R << std::endl;
        std::cout << "fdxH_L: " << data.fdxH_L << std::endl;
        std::cout << "fdyH_L: " << data.fdyH_L << std::endl;
        std::cout << "fdzH_L: " << data.fdzH_L << std::endl;
        std::cout << "FxH_hmi: " << data.FxH_hmi << std::endl;
        std::cout << "FyH_hmi: " << data.FyH_hmi << std::endl;
        std::cout << "FxH_spring: " << data.FxH_spring << std::endl;
        std::cout << "================================================================" << std::endl;
        std::cout << std::endl;
    }
}

void dash_utils::rotate_foot(Eigen::Vector3d& point1, Eigen::Vector3d& point2, double theta) {
  // Compute the midpoint of the two input points
  Eigen::Vector3d midpoint = (point1 + point2) / 2.0;
  
  // Compute the rotation matrix around the z-axis
  double c = cos(theta);
  double s = sin(theta);
  Eigen::Matrix3d rotation;
  rotation << c, -s, 0,
              s,  c, 0,
              0,  0, 1;

  // Translate the points to the origin, rotate them, and translate them back
  Eigen::Vector3d translated1 = point1 - midpoint;
  Eigen::Vector3d rotated1 = rotation * translated1;
  Eigen::Vector3d translated_back1 = rotated1 + midpoint;
  
  Eigen::Vector3d translated2 = point2 - midpoint;
  Eigen::Vector3d rotated2 = rotation * translated2;
  Eigen::Vector3d translated_back2 = rotated2 + midpoint;
  
  // Pack the resulting points into a dynamically allocated VectorXd array
  Eigen::VectorXd* result = new Eigen::VectorXd[2];
  point1 = translated_back1;
  point2 = translated_back2;

}

double dash_utils::smoothData(const Eigen::VectorXd& vel, double smoothingFactor) {
    int n = vel.size();
    Eigen::VectorXd smoothedVel(n);
    smoothedVel.setZero();

    if (n > 0) {
        smoothedVel(0) = vel(0);
        int maxSamples = std::min(n, 100);
        for (int i = 1; i < maxSamples; ++i) {
            double weight = smoothingFactor / (i + 1);
            smoothedVel(i) = (1 - weight) * smoothedVel(i - 1) + weight * vel(i);
        }
        for (int i = maxSamples; i < n; ++i) {
            double weight = smoothingFactor / maxSamples;
            smoothedVel(i) = (1 - weight) * smoothedVel(i - 1) + weight * vel(i);
        }
    }

    return smoothedVel(n - 1);
}

double dash_utils::EMA(const VectorXd& circularBuffer, int latestSampleIndex, int numPreviousSamples, double smoothingParameter) {
    int bufferSize = circularBuffer.size();
    
    // Create a mask to select the previous samples
    VectorXi mask(numPreviousSamples);
    for (int i = 0; i < numPreviousSamples; ++i) {
        int index = (latestSampleIndex - i - 1 + bufferSize) % bufferSize;
        mask(i) = index;
    }
    
    // Extract the previous samples from the circular buffer using the mask
    VectorXd previousSamples = circularBuffer(mask);
    
    // Compute the weights for the previous samples
    double weight = 1.0 - smoothingParameter;
    VectorXd weights = VectorXd::Constant(numPreviousSamples, weight);
    for (int i = 1; i < numPreviousSamples; ++i)
        weights(i) = weights(i - 1) * weight;
    
    // Compute the weighted average
    double normalizationFactor = weights.sum();
    VectorXd weightedAverage = (weights.array() / normalizationFactor) * previousSamples.array();
    
    return weightedAverage.sum();
}
