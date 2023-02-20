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
// #include "cheetah_motor.h"
// #include "comms.h"
// #include "timer.h"
// #include "user_config.h"
// #include "utilities.h"
// #include "kinematics.h"
#include "vn/sensors.h"

#define TASK_CONSTANT_PERIOD 0
#define TASK_CONSTANT_DELAY 1

namespace RoboDesignLab {
    class DynamicRobot {
    public:
        // Constructor and destructor
        DynamicRobot(){}
        ~DynamicRobot();

        void assign_jacobian_motors_to_joints(std::function<Eigen::MatrixXd()>* fcn){ _jaco_motor2joint = fcn; }
        void assign_jacobian_joints_to_motors(std::function<Eigen::MatrixXd()>* fcn){ _jaco_joint2motor = fcn; }
        void assign_jacobian_joints_to_task(std::function<Eigen::MatrixXd()>* fcn)  { _jaco_joint2task = fcn;  }
        void assign_jacobian_task_to_joints(std::function<Eigen::MatrixXd()>* fcn)  { _jaco_task2joint = fcn;  }

        void assign_ik_joints_to_motors(std::function<Eigen::VectorXd()>* fcn){ _ik_joint2motor = fcn; }
        void assign_fk_motors_to_joints(std::function<Eigen::VectorXd()>* fcn){ _fk_motor2joint = fcn; }
        void assign_ik_task_to_joints(std::function<Eigen::VectorXd()>* fcn)  { _ik_task2joint = fcn;  }
        void assign_fk_joints_to_task(std::function<Eigen::VectorXd()>* fcn)  { _fk_joint2task = fcn;  }

        Eigen::MatrixXd jacobian_joint()        { return (*_jaco_motor2joint)(); }
        Eigen::MatrixXd jacobian_joint_inverse(){ return (*_jaco_joint2motor)(); }
        Eigen::MatrixXd jacobian_task()         { return (*_jaco_joint2task)();  }
        Eigen::MatrixXd jacobian_task_inverse() { return (*_jaco_task2joint)();  }

        Eigen::MatrixXd jacobian_joint_transpose()        { return (*_jaco_motor2joint)().transpose(); }
        Eigen::MatrixXd jacobian_joint_inverse_transpose(){ return (*_jaco_joint2motor)().transpose(); }
        Eigen::MatrixXd jacobian_task_transpose()         { return (*_jaco_joint2task)().transpose();  }
        Eigen::MatrixXd jacobian_task_inverse_transpose() { return (*_jaco_task2joint)().transpose();  }

        Eigen::VectorXd motor_vel_to_joint_vel(Eigen::VectorXd motor_velocites);
        Eigen::VectorXd joint_vel_to_motor_vel(Eigen::VectorXd joint_velocites);
        Eigen::VectorXd joint_vel_to_task_vel(Eigen::VectorXd joint_velocites);
        Eigen::VectorXd task_vel_to_joint_vel(Eigen::VectorXd task_velocites);

        Eigen::VectorXd motor_torque_to_joint_torque(Eigen::VectorXd motor_torques);
        Eigen::VectorXd joint_torque_to_motor_torque(Eigen::VectorXd joint_torques);
        Eigen::VectorXd joint_torque_to_task_force(Eigen::VectorXd joint_torques);
        Eigen::VectorXd task_forces_to_joint_torque(Eigen::VectorXd task_forces);

        Eigen::VectorXd motor_pos_to_joint_pos(Eigen::VectorXd motor_positions){ return (*_fk_motor2joint)(); }
        Eigen::VectorXd joint_pos_to_motor_pos(Eigen::VectorXd joint_positions){ return (*_ik_joint2motor)(); }
        Eigen::VectorXd joint_pos_to_task_pos(Eigen::VectorXd joint_positions) { return (*_fk_joint2task)();  }
        Eigen::VectorXd task_pos_to_joint_pos(Eigen::VectorXd task_positions)  { return (*_ik_task2joint)();  }

        void addPeriodicTask(void *(*start_routine)(void *), int sched_policy, int priority, int cpu_affinity, void *arg, std::string task_name,int task_type, int period);


    private:
        // Kinematics Functions
        std::function<Eigen::MatrixXd()>* _jaco_motor2joint;
        std::function<Eigen::MatrixXd()>* _jaco_joint2motor;
        std::function<Eigen::MatrixXd()>* _jaco_joint2task;
        std::function<Eigen::MatrixXd()>* _jaco_task2joint;

        std::function<Eigen::VectorXd()>* _ik_joint2motor;
        std::function<Eigen::VectorXd()>* _fk_motor2joint;
        std::function<Eigen::VectorXd()>* _ik_task2joint;
        std::function<Eigen::VectorXd()>* _fk_joint2task;

        // Sensors
    };
}
