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
// #include "utilities.h"
// #include "kinematics.h"
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"

using namespace Eigen;

#define TASK_CONSTANT_PERIOD 0
#define TASK_CONSTANT_DELAY 1
#define ENCODER_TO_RADIANS ((double)(12.5/32768.0))
#define VELOCITY_TO_RADIANS_PER_SEC ((double)(65.0/4096.0))

typedef VectorXd (*VectorXd_function)(const VectorXd&);
typedef MatrixXd (*MatrixXd_function)(const VectorXd&);

namespace RoboDesignLab {

    struct BipedActuatorTree{
        std::vector<CheetahMotor*> leftLeg;
        std::vector<CheetahMotor*> rightLeg;
    };

    class DynamicRobot {
    public:
        // Constructor and destructor
        DynamicRobot(BipedActuatorTree actuators);
        ~DynamicRobot();

        void assign_jacobian_motors_to_joints(MatrixXd_function fcn){ _jaco_motor2joint = fcn; }
        void assign_jacobian_joints_to_motors(MatrixXd_function fcn){ _jaco_joint2motor = fcn; }

        // This part is specific to a line foot robot:
        void assign_jacobian_joints_to_task_lf_front(MatrixXd_function fcn)  { _jaco_joint2taskFront = fcn;  }
        void assign_jacobian_joints_to_task_lf_back(MatrixXd_function fcn)  { _jaco_joint2taskBack = fcn;  }

        void assign_ik_joints_to_motors(VectorXd_function fcn){ _ik_joint2motor = fcn; }
        void assign_fk_motors_to_joints(VectorXd_function fcn){ _fk_motor2joint = fcn; }
        void assign_ik_task_to_joints(VectorXd_function fcn)  { _ik_task2joint = fcn;  }
        void assign_fk_joints_to_task(VectorXd_function fcn)  { _fk_joint2task = fcn;  }

        MatrixXd jacobian_joint(VectorXd joint_config)        { return (*_jaco_motor2joint)(joint_config); }
        MatrixXd jacobian_joint_inverse(VectorXd joint_config){ return (*_jaco_joint2motor)(joint_config); }

        // This part is specific to a line foot robot:
        MatrixXd jacobian_task_lf_front(VectorXd joint_config){ return (*_jaco_joint2taskFront)(joint_config); }
        MatrixXd jacobian_task_lf_back(VectorXd joint_config) { return (*_jaco_joint2taskBack)(joint_config);  }

        VectorXd motor_vel_to_joint_vel(VectorXd motor_velocites);
        VectorXd joint_vel_to_motor_vel(VectorXd joint_velocites);
        VectorXd joint_vel_to_task_vel(VectorXd joint_velocites);
        VectorXd task_vel_to_joint_vel(VectorXd task_velocites_front, VectorXd task_velocites_back );
        VectorXd task_vel_to_joint_vel(VectorXd task_velocites);

        VectorXd motor_torque_to_joint_torque(VectorXd motor_torques);
        VectorXd joint_torque_to_motor_torque(VectorXd joint_torques);
        VectorXd joint_torque_to_task_force(VectorXd joint_torques);
        VectorXd task_force_to_joint_torque(VectorXd task_forces_front, VectorXd task_forces_back);
        VectorXd task_force_to_joint_torque(VectorXd task_forces);

        VectorXd motor_pos_to_joint_pos(VectorXd motor_positions){ return (*_fk_motor2joint)(motor_positions); }
        VectorXd joint_pos_to_motor_pos(VectorXd joint_positions){ return (*_ik_joint2motor)(joint_positions); }
        VectorXd joint_pos_to_task_pos(VectorXd joint_positions) { return (*_fk_joint2task)(joint_positions);  }
        VectorXd task_pos_to_joint_pos(VectorXd task_positions)  { return (*_ik_task2joint)(task_positions);   }

        void addPeriodicTask(void *(*start_routine)(void *), int sched_policy, int priority, int cpu_affinity, void *arg, std::string task_name,int task_type, int period);

        VectorXd getJointVelocities();
        VectorXd getJointPositions();

        void motorPD(VectorXd pos_desired, VectorXd vel_desired, VectorXd kp, VectorXd kd);
        void jointPD(VectorXd pos_desired, VectorXd vel_desired, MatrixXd j_kp, MatrixXd j_kd, VectorXd m_kp, VectorXd m_kd);
        void taskPD(VectorXd pos_desired, VectorXd vel_desired, MatrixXd t_kp, MatrixXd t_kd, MatrixXd j_kp, MatrixXd j_kd, VectorXd m_kp, VectorXd m_kd);
        VectorXd calc_pd_effort(VectorXd position, VectorXd velocity, VectorXd desiredPosition, VectorXd desiredVelocity, MatrixXd Kp, MatrixXd Kd);

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
    private:
        // Kinematics Functions
        MatrixXd_function _jaco_motor2joint;
        MatrixXd_function _jaco_joint2motor;
        MatrixXd_function _jaco_joint2taskFront;
        MatrixXd_function _jaco_joint2taskBack;

        VectorXd_function _ik_joint2motor;
        VectorXd_function _fk_motor2joint;
        VectorXd_function _ik_task2joint;
        VectorXd_function _fk_joint2task;

        // Sensors
        

        // Actuators
        BipedActuatorTree _actuators;
        int _leg_DoF; // automatically set from actuator tree
        int _num_actuators;
    };
}

#endif
