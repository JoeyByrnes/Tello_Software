#include <Eigen/Dense>
#include <math.h>
#include "structs.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include "json.hpp"

using namespace std;
using namespace Eigen;

namespace  dash_utils 
{
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    Vector3d calc_EA(Matrix3d R);
    Vector3d calc_dEA(Matrix3d R, Vector3d wb);
    VectorXd calc_wb(Vector3d dEA, VectorXd EA);

    void gen_trapz_traj(double max_speed, Eigen::VectorXd &t, Eigen::VectorXd &x); 
    Matrix3d hatMap(VectorXd a);

    void Human_teleop_ref_get_step_placement_data(Teleop_Ref teleop_ref);
    void LIP_dyn_ref(double t_step, double omega, double x0, double xd0, double& x, double& xd);
    Teleop_2DLIP_Traj_Data read_teleop_2DLIP_traj(const char * data_txt_file); // TODO, function not finished

    Matrix3d rx(double phi);
    Matrix3d ry(double theta);
    Matrix3d rz(double psi);

    Vector2d sinu_ref_traj(double t, Vector3d sinu_traj_params);
    Vector2d sw_leg_ref_xy(double s, double init, double final);
    Vector2d sw_leg_ref_z(double s, double zcl, double H);

    VectorXd flatten(MatrixXd matrix); 
    void writeVectorToCsv(const VectorXd& vec, const std::string& filename);
    void writeMatrixToCsv(const MatrixXd& mat, const std::string& filename);
    void writeHumanDynDataToCsv(const Human_dyn_data& data, const std::string& filename);

    std::vector<Human_dyn_data> readHumanDynDataFromFile(const std::string& filename);

    void writeTrajPlannerDataToCsv(const Traj_planner_dyn_data& data, const std::string& filename);
    void setOutputFolder(const std::string& foldername);

    void writeSRBParamsToTxt(const SRB_Params& params, const std::string& filename);
    void writeHumanParamsToTxt(const Human_params& params, const std::string& filename);
    void writeStringToFile(const std::string& str, const std::string& filename);
    void deleteLogFile(const std::string& filename);

    Vector3d worldToHip(Vector3d foot_pos_world, Vector3d hip_pos_world, Vector3d hip_orient_world);
    Vector3d hipToWorld(Vector3d vector_hip, Vector3d hip_pos_world, Vector3d hip_orient_world);

    void parse_json_to_pd_params(const std::string& json_file_path, Joint_PD_config& swing, Joint_PD_config& posture);
    void parse_json_to_srb_params(const std::string& json_file_path, SRB_Params& params); 

    void printJointPDConfig(const Joint_PD_config& config);

    void start_timer();
    void print_timer();
    void start_sim_timer();
    double measure_sim_timer();

    void pack_data_to_hmi(uint8_t* buffer, Human_dyn_data data);
    void pack_data_to_hmi_with_ctrls(uint8_t* buffer, Human_dyn_data data,bool ff,bool tare,float gain);
    void unpack_data_from_hmi(Human_dyn_data& data, uint8_t* buffer);
    void print_human_dyn_data(const Human_dyn_data& data);

    void rotate_foot(Eigen::Vector3d& point1, Eigen::Vector3d& point2, double theta);

    double smoothData(const Eigen::VectorXd& vel, double smoothingFactor);
    double EMA(const VectorXd& circularBuffer, int latestSampleIndex, int numPreviousSamples, double smoothingParameter);

    void pseudoInverse(Matrix& matrix, double sigmaThreshold, Matrix& invMatrix, Vector* opt_sigmaOut);
}