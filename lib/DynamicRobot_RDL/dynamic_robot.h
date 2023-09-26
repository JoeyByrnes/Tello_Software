#ifndef __DYNAMIC_ROBOT_H__
#define __DYNAMIC_ROBOT_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include "PCANBasic.h"
#include <sched.h> 
#include <fstream>
#include <signal.h>
#include <vector>
#include <chrono>
#include <math.h>

#include "json.hpp"
#include "../Eigen/Dense"
#include "cheetah_motor.h"
// #include "comms.h"
// #include "timer.h"
// #include "user_config.h"
#include "../../include/utilities.h"
// #include "kinematics.h"
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"
#include "InEKF.h"
#include "SRBMController.h"

using namespace Eigen;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace vn::math;
using namespace inekf;

#define TASK_CONSTANT_PERIOD 0
#define TASK_CONSTANT_DELAY 1
#define ENCODER_TO_RADIANS ((double)(12.5/32768.0))
#define VELOCITY_TO_RADIANS_PER_SEC ((double)(65.0/4096.0))
#define DT_MIN 1e-6
#define DT_MAX 0.025

#define GEAR_RATIO 6.0
#define MOTOR_KT 0.0955
#define MAX_ACTUATOR_CURRENT 15.0
#define MAX_ACTUATOR_TORQUE_NM (double)(MAX_ACTUATOR_CURRENT*MOTOR_KT*GEAR_RATIO)
#define NM_TO_MOTOR_TORQUE_CMD (double)(2048.0/MAX_ACTUATOR_TORQUE_NM)

typedef VectorXd (*VectorXd_function)(const VectorXd&);
typedef MatrixXd (*MatrixXd_function)(const VectorXd&);

typedef MatrixXd (*MatrixXd_function_accel)(const VectorXd&, const VectorXd&);

#define BOTH_LEGS 0
#define RIGHT_LEG 1
#define LEFT_LEG  2

enum JointName {
    L_Hip_Yaw,
    L_Hip_Roll,
    L_Hip_Pitch,
    L_Knee,
    L_Ankle,
    R_Hip_Yaw,
    R_Hip_Roll,
    R_Hip_Pitch,
    R_Knee,
    R_Ankle
};

namespace RoboDesignLab {

    struct BipedActuatorTree{
        std::vector<CheetahMotor*> leftLeg;
        std::vector<CheetahMotor*> rightLeg;
    };

    struct MotorPDConfig{
        VectorXd motor_pos_desired;
        VectorXd motor_vel_desired;
        VectorXd motor_kp;
        VectorXd motor_kd;
        VectorXd motor_ff_torque;
    };

    struct JointPDConfig{
        VectorXd joint_pos_desired;
        VectorXd joint_vel_desired;
        MatrixXd joint_kp;
        MatrixXd joint_kd;
        VectorXd motor_kp;
        VectorXd motor_kd;
        VectorXd joint_ff_torque;
    };

    struct TaskPDConfig{
        VectorXd task_pos_desired = VectorXd(12);
        VectorXd task_vel_desired = VectorXd(12);
        MatrixXd task_kp;
        MatrixXd task_kd;
        MatrixXd task_ka;
        MatrixXd joint_kp;
        MatrixXd joint_kd;
        MatrixXd joint_ka;
        VectorXd motor_kp = VectorXd(10);
        VectorXd motor_kd = VectorXd(10);
        VectorXd task_ff_force = VectorXd::Zero(12);
        VectorXd task_ff_accel = VectorXd::Zero(12);
        VectorXd joint_ff_torque = VectorXd::Zero(10);
        int side; // both=0, right=1, left=2
        bool use_single_jacoian;
        bool ignore_joint_velocity;

        void setTaskKp(double x, double y, double z) {
            task_kp = ((Vector3d(x,y,z)).replicate(4, 1)).asDiagonal();
        }
        void setTaskKp(Vector3d kp) {
            task_kp = ((kp).replicate(4, 1)).asDiagonal();
        }
        void setTaskKd(double x, double y, double z) {
            task_kd = ((Vector3d(x,y,z)).replicate(4, 1)).asDiagonal();
        }
        void setTaskKd(Vector3d kd) {
            task_kd = ((kd).replicate(4, 1)).asDiagonal();
        }

        void setTaskKa(double x, double y, double z) {
            task_ka = ((Vector3d(x,y,z)).replicate(4, 1)).asDiagonal();
        }
        void setTaskKa(Vector3d ka) {
            task_ka = ((ka).replicate(4, 1)).asDiagonal();
        }

        void setJointKp(VectorXd kp) {
            joint_kp = kp.asDiagonal();
        }
        void setJointKd(VectorXd kd) {
            joint_kd = kd.asDiagonal();
        }
        void setJointKa(VectorXd ka) {
            if(ka.size()==5)
            {
                VectorXd ka_vec(10);
                ka_vec << ka,ka;
                joint_ka = ka_vec.asDiagonal();
            }
            else // else size must be 10
            {
                joint_ka = ka.asDiagonal();
            }
        }
        void setJointKa(double ka) {
            VectorXd ka_vec(10);
            ka_vec << ka,ka,ka,ka,ka,ka,ka,ka,ka,ka;
            joint_ka = ka_vec.asDiagonal();
        }
        void setJointKa(double ka0,double ka1,double ka2,double ka3,double ka4) {
            VectorXd ka_vec(10);
            ka_vec << ka0,ka1,ka2,ka3,ka4,ka0,ka1,ka2,ka3,ka4;
            joint_ka = ka_vec.asDiagonal();
        }

        void setMotorKp(double kp) {
            motor_kp << kp,kp,kp,kp,kp,kp,kp,kp,kp,kp;
        }
        void setMotorKd(double kd) {
            motor_kd << kd,kd,kd,kd,kd,kd,kd,kd,kd,kd;
        }

        void setTaskPosDesired(Vector3d lf, Vector3d lb, Vector3d rf, Vector3d rb) {
            task_pos_desired << lf, lb, rf, rb;
        }
        void setTaskVelDesired(Vector3d lf, Vector3d lb, Vector3d rf, Vector3d rb) {
            task_vel_desired << lf, lb, rf, rb;
        }
        void setFF(Vector3d lf, Vector3d lb, Vector3d rf, Vector3d rb) {
            task_ff_force << lf, lb, rf, rb;
        }
        void setFFAccel(Vector3d lf, Vector3d lb, Vector3d rf, Vector3d rb) {
            task_ff_accel << lf, lb, rf, rb;
        }
    };

    struct GRFs{
        double left_front = 0;
        double left_back = 0;
        double right_front = 0;
        double right_back = 0;
    };

    struct IMU_data{
        Vector3d ypr = Vector3d(0,0,0);
        Vector3d acc = Vector3d(0,0,0);
        Vector3d gyro = Vector3d(0,0,0);
        double timestamp = 0; //(seconds)
    };

    class DynamicRobot {
    public:
        // Constructor and destructor
        DynamicRobot();
        // Destructor
        ~DynamicRobot() {
            // Delete dynamically allocated CheetahMotor objects
            for (int i = 0; i < 10; i++) {
                delete motors[i];
            }
            // Delete dynamically allocated SRBMController object
            delete controller;
        }
        DynamicRobot(const DynamicRobot& other);

        void assign_jacobian_motors_to_joints(MatrixXd_function fcn){ _jaco_motor2joint = fcn; }
        void assign_jacobian_joints_to_motors(MatrixXd_function fcn){ _jaco_joint2motor = fcn; }

        // This part is specific to a line foot robot:
        void assign_jacobian_joints_to_task_lf_front(MatrixXd_function fcn)  { _jaco_joint2taskFront = fcn;  }
        void assign_jacobian_joints_to_task_lf_back(MatrixXd_function fcn)  { _jaco_joint2taskBack = fcn;  }

        void assign_jacobian_accel_task_to_joint(MatrixXd_function_accel fcn)  { _jaco_accel_task_to_joint = fcn;  }

        void assign_ik_joints_to_motors(VectorXd_function fcn){ _ik_joint2motor = fcn; }
        void assign_fk_motors_to_joints(VectorXd_function fcn){ _fk_motor2joint = fcn; }
        void assign_ik_task_to_joints(VectorXd_function fcn)  { _ik_task2joint = fcn;  }
        void assign_fk_joints_to_task(VectorXd_function fcn)  { _fk_joint2task = fcn;  }

        MatrixXd jacobian_joint(VectorXd joint_config)        { return (*_jaco_motor2joint)(joint_config); }
        MatrixXd jacobian_joint_inverse(VectorXd joint_config){ return (*_jaco_joint2motor)(joint_config); }

        // This part is specific to a line foot robot:
        MatrixXd jacobian_task_lf_front(VectorXd joint_config){ return (*_jaco_joint2taskFront)(joint_config); }
        MatrixXd jacobian_task_lf_back(VectorXd joint_config) { return (*_jaco_joint2taskBack)(joint_config);  }

        MatrixXd jacobian_task_accel_lf_front(VectorXd joint_config, VectorXd joint_velocites) { return (*_jaco_accel_task_to_joint)(joint_config,joint_velocites);  }

        VectorXd motor_vel_to_joint_vel(VectorXd motor_velocites, Eigen::VectorXd joint_positions);
        VectorXd joint_vel_to_motor_vel(VectorXd joint_velocites, Eigen::VectorXd joint_positions);
        VectorXd joint_vel_to_task_vel(VectorXd joint_velocites, Eigen::VectorXd joint_positions);
        VectorXd task_vel_to_joint_vel(VectorXd task_velocites_front, VectorXd task_velocites_back, Eigen::VectorXd joint_positions);
        VectorXd task_vel_to_joint_vel(VectorXd task_velocites, Eigen::VectorXd joint_positions);
        VectorXd task_vel_to_joint_vel(VectorXd task_velocites, TaskPDConfig task_conf, Eigen::VectorXd joint_positions);
        VectorXd task_vel_to_joint_vel_right(VectorXd task_velocites, TaskPDConfig task_conf, Eigen::VectorXd joint_positions);
        VectorXd task_vel_to_joint_vel_left(VectorXd task_velocites, TaskPDConfig task_conf, Eigen::VectorXd joint_positions);

        VectorXd motor_torque_to_joint_torque(VectorXd motor_torques, Eigen::VectorXd joint_positions);
        VectorXd joint_torque_to_motor_torque(VectorXd joint_torques, Eigen::VectorXd joint_positions);
        VectorXd joint_torque_to_task_force(VectorXd joint_torques, Eigen::VectorXd joint_positions);
        VectorXd task_force_to_joint_torque(VectorXd task_forces_front, VectorXd task_forces_back, Eigen::VectorXd joint_positions);
        VectorXd task_force_to_joint_torque(VectorXd task_forces, Eigen::VectorXd joint_positions);
        VectorXd task_accel_to_joint_accel(Eigen::VectorXd task_accel, Eigen::VectorXd joint_positions, Eigen::VectorXd joint_velocities);

        VectorXd motor_pos_to_joint_pos(VectorXd motor_positions){ return (*_fk_motor2joint)(motor_positions); }
        VectorXd joint_pos_to_motor_pos(VectorXd joint_positions){ return (*_ik_joint2motor)(joint_positions); }
        VectorXd joint_pos_to_task_pos(VectorXd joint_positions) { return (*_fk_joint2task)(joint_positions);  }
        VectorXd task_pos_to_joint_pos(VectorXd task_positions)  { return (*_ik_task2joint)(task_positions);   }

        void addPeriodicTask(void *(*start_routine)(void *), int sched_policy, int priority, int cpu_affinity, void *arg, std::string task_name,int task_type, int period);

        double sigmoid(double x) { return 1 / (1 + exp(-x)); }
        MotorPDConfig switchController(const MotorPDConfig& stanceCtrl, const MotorPDConfig& swingCtrl,
                                       int durationMs, bool isSwingToStance, int currTimeStep);
        VectorXd swing_stance_mux(VectorXd stanceTorques, VectorXd swingTorques,
                               double duration_sec, bool isSwingToStance, double currTimeStep, int side);

        VectorXd getJointVelocities();
        VectorXd getJointPositions();

        void setJointEncoderPosition(double rad, JointName joint_name);
        void setJointEncoderVelocity(double rad, JointName joint_name);

        void motorPD(MotorPDConfig motor_conf);
        void jointPD(JointPDConfig joint_conf);
        void taskPD(TaskPDConfig task_conf);
        VectorXd calc_pd(VectorXd position, VectorXd velocity, VectorXd desiredPosition, VectorXd desiredVelocity, MatrixXd Kp, MatrixXd Kd);

        // Simulation:
        VectorXd jointPD2(JointPDConfig joint_conf);
        VectorXd taskPD2(TaskPDConfig task_conf);
        VectorXd taskPD3(TaskPDConfig task_conf);
        void resetController(){delete controller; controller = new SRBMController();}

        // InEKF Functions:
        void update_filter_IMU_data(IMU_data imu_data);
        void update_filter_contact_data(VectorXd ground_contacts);
        void update_filter_kinematic_data(MatrixXd lfv_hip, Matrix3d R_right, Matrix3d R_left);
        void update_filter_landmark_data(int id, Vector3d landmark_pos);
        RobotState get_filter_state();
        void set_filter_state(RobotState state);
        void set_imu_data_for_ekf(IMU_data imu_data){_imu_data = imu_data;}
        IMU_data get_imu_data_for_ekf(){return _imu_data;}
        void set_gnd_contact_data_for_ekf(VectorXd ground_contacts){_ground_contacts = ground_contacts;}
        VectorXd get_gnd_contact_data_for_ekf(){return _ground_contacts;}
        void set_lfv_hip_data_for_ekf(MatrixXd direct_lfv_hip){_direct_lfv_hip = direct_lfv_hip;}
        MatrixXd get_lfv_hip_data_for_ekf(){return _direct_lfv_hip;}
        void set_q_data_for_ekf(MatrixXd q){_ekf_q = q;}
        MatrixXd get_q_data_for_ekf(){return _ekf_q;}

        Eigen::VectorXd get_CoP(); 

        // Quaterniond getFootOrientation(const Vector3d& lf1, const Vector3d& lf2, const Vector3d& knee);

        void addGravityCompensation();
        Vector3d transformForceToWorldFrame(const Eigen::VectorXd& force, VectorXd ypr);
        
        // Actuators
        int motor_pos_model_to_real(int id, double actuator_position_radians);
        double motor_pos_real_to_model(int id, int motor_position_enc_counts);
        void enable_all_motors();
        void disable_all_motors();
        void set_kp_kd_all_motors(uint16_t kp, uint16_t kd);
        void update_all_motors();
        void set_motor_torques(VectorXd motor_torques);
        void add_motor_torques(VectorXd motor_torques);
        CheetahMotor* motors[10]; // move this to private soon
        int motor_directions[10]; // temporary, need to change
        MatrixXd _motor_direction_matrix;
        int motor_zeros[10];
        int motor_timeouts[10] = {0,0,0,0,0,0,0,0,0,0};
        int motor_connected[10] = {0,0,0,0,0,0,0,0,0,0};


        // Sensors
        Eigen::Vector3d updatePosFromIMU(const Eigen::Vector3d& acc, const Eigen::Vector3d& ypr, double delta_t, Eigen::Vector3d& pos, Eigen::Vector3d& vel);
        VnSensor imu;
        Vector3d _ypr;
        Vector3d _rpy;
        Vector3d _acc = VectorXd::Zero(3);
        Vector3d _gyro = VectorXd::Zero(3);
        Vector3d _vel = VectorXd::Zero(3);
        Vector3d _pos = VectorXd::Zero(3);
        int _balance_adjust = 0;

        // ground contact sensors:
        GRFs _GRFs;
        GRFs _GRF_biases;
        bool _right_loadcells_calibrated = false;
        bool _left_loadcells_calibrated = false;

        bool isSimulation = false;
        VectorXd sim_joint_torques = VectorXd::Zero(10);
        VectorXd sim_joint_pos = VectorXd::Zero(10);
        VectorXd sim_joint_vel = VectorXd::Zero(10);

        // Balance controller
        SRBMController* controller;

        VectorXd plot_data = VectorXd(20);
        Matrix3d plot_mat;

    private:
        // Kinematics Functions
        MatrixXd_function _jaco_motor2joint;
        MatrixXd_function _jaco_joint2motor;
        MatrixXd_function _jaco_joint2taskFront;
        MatrixXd_function _jaco_joint2taskBack;

        MatrixXd_function_accel _jaco_accel_task_to_joint;

        VectorXd_function _ik_joint2motor;
        VectorXd_function _fk_motor2joint;
        VectorXd_function _ik_task2joint;
        VectorXd_function _fk_joint2task;

        // Actuators
        BipedActuatorTree _actuators;
        int _leg_DoF; // automatically set from actuator tree
        int _num_actuators;

        VectorXd joint_encoder_positions = VectorXd(10);
        VectorXd joint_encoder_velocities = VectorXd(10);

        // InEKF:
        RobotState initial_state; 
        NoiseParams noise_params;
        InEKF filter;
        IMU_data _imu_data;
        IMU_data _imu_data_prev;
        VectorXd _ground_contacts = VectorXd(4);
        MatrixXd _direct_lfv_hip = MatrixXd(4,3);
        MatrixXd _ekf_q = MatrixXd(2,5);

    };
}

#endif
