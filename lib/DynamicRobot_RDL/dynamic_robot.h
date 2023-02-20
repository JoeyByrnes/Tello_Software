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

#include "../Eigen/Dense"
#include "cheetah_motor.h"
// #include "comms.h"
// #include "timer.h"
// #include "user_config.h"
// #include "utilities.h"
// #include "kinematics.h"
#include "vn/sensors.h"

#define TASK_CONSTANT_PERIOD 0
#define TASK_CONSTANT_DELAY 1

typedef Eigen::VectorXd (*VectorXd_function)(const Eigen::VectorXd&);
typedef Eigen::MatrixXd (*MatrixXd_function)(const Eigen::VectorXd&);

namespace RoboDesignLab {

    struct BipedActuatorTree{
        std::vector<CheetahMotor*> leftLeg;
        std::vector<CheetahMotor*> rightLeg;
    };

    class DynamicRobot {
    public:
        // Constructor and destructor
        DynamicRobot(BipedActuatorTree actuators){ this->_actuators = actuators; }
        ~DynamicRobot();

        void assign_jacobian_motors_to_joints(MatrixXd_function fcn){ _jaco_motor2joint = fcn; }
        void assign_jacobian_joints_to_motors(MatrixXd_function fcn){ _jaco_joint2motor = fcn; }
        void assign_jacobian_joints_to_task(MatrixXd_function fcn)  { _jaco_joint2task = fcn;  }
        void assign_jacobian_task_to_joints(MatrixXd_function fcn)  { _jaco_task2joint = fcn;  }

        void assign_ik_joints_to_motors(VectorXd_function fcn){ _ik_joint2motor = fcn; }
        void assign_fk_motors_to_joints(VectorXd_function fcn){ _fk_motor2joint = fcn; }
        void assign_ik_task_to_joints(VectorXd_function fcn)  { _ik_task2joint = fcn;  }
        void assign_fk_joints_to_task(VectorXd_function fcn)  { _fk_joint2task = fcn;  }

        Eigen::MatrixXd jacobian_joint(Eigen::VectorXd joint_config)        { return (*_jaco_motor2joint)(joint_config); }
        Eigen::MatrixXd jacobian_joint_inverse(Eigen::VectorXd joint_config){ return (*_jaco_joint2motor)(joint_config); }
        Eigen::MatrixXd jacobian_task(Eigen::VectorXd joint_config)         { return (*_jaco_joint2task)(joint_config);  }
        Eigen::MatrixXd jacobian_task_inverse(Eigen::VectorXd joint_config) { return (*_jaco_task2joint)(joint_config);  }

        Eigen::MatrixXd jacobian_joint_transpose(Eigen::VectorXd joint_config)        { return (*_jaco_motor2joint)(joint_config).transpose(); }
        Eigen::MatrixXd jacobian_joint_inverse_transpose(Eigen::VectorXd joint_config){ return (*_jaco_joint2motor)(joint_config).transpose(); }
        Eigen::MatrixXd jacobian_task_transpose(Eigen::VectorXd joint_config)         { return (*_jaco_joint2task)(joint_config).transpose();  }
        Eigen::MatrixXd jacobian_task_inverse_transpose(Eigen::VectorXd joint_config) { return (*_jaco_task2joint)(joint_config).transpose();  }

        Eigen::VectorXd motor_vel_to_joint_vel(Eigen::VectorXd motor_velocites);
        Eigen::VectorXd joint_vel_to_motor_vel(Eigen::VectorXd joint_velocites);
        Eigen::VectorXd joint_vel_to_task_vel(Eigen::VectorXd joint_velocites);
        Eigen::VectorXd task_vel_to_joint_vel(Eigen::VectorXd task_velocites);

        Eigen::VectorXd motor_torque_to_joint_torque(Eigen::VectorXd motor_torques);
        Eigen::VectorXd joint_torque_to_motor_torque(Eigen::VectorXd joint_torques);
        Eigen::VectorXd joint_torque_to_task_force(Eigen::VectorXd joint_torques);
        Eigen::VectorXd task_forces_to_joint_torque(Eigen::VectorXd task_forces);

        Eigen::VectorXd motor_pos_to_joint_pos(Eigen::VectorXd motor_positions){ return (*_fk_motor2joint)(motor_positions); }
        Eigen::VectorXd joint_pos_to_motor_pos(Eigen::VectorXd joint_positions){ return (*_ik_joint2motor)(joint_positions); }
        Eigen::VectorXd joint_pos_to_task_pos(Eigen::VectorXd joint_positions) { return (*_fk_joint2task)(joint_positions);  }
        Eigen::VectorXd task_pos_to_joint_pos(Eigen::VectorXd task_positions)  { return (*_ik_task2joint)(task_positions);  }

        void addPeriodicTask(void *(*start_routine)(void *), int sched_policy, int priority, int cpu_affinity, void *arg, std::string task_name,int task_type, int period);

        // Actuators
        CheetahMotor* motors[10]; // move this to private soon
    private:
        // Kinematics Functions
        MatrixXd_function _jaco_motor2joint;
        MatrixXd_function _jaco_joint2motor;
        MatrixXd_function _jaco_joint2task;
        MatrixXd_function _jaco_task2joint;

        VectorXd_function _ik_joint2motor;
        VectorXd_function _fk_motor2joint;
        VectorXd_function _ik_task2joint;
        VectorXd_function _fk_joint2task;

        Eigen::VectorXd _joint_config;

        // Sensors

        // Actuators
        BipedActuatorTree _actuators;
        int _leg_DoF; // automatically set from actuator tree

    };
}

#endif
