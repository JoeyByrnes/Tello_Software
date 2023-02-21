#include "dynamic_robot.h"

using namespace RoboDesignLab;

Eigen::VectorXd DynamicRobot::motor_vel_to_joint_vel(Eigen::VectorXd motor_velocites)
{
    return this->jacobian_joint(this->_joint_config)*motor_velocites;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_motor_vel(Eigen::VectorXd joint_velocites)
{
    return this->jacobian_joint_inverse(this->_joint_config)*joint_velocites;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_task_vel(Eigen::VectorXd joint_velocites)
{
    //return this->jacobian_task(this->_joint_config)*joint_velocites;
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel(Eigen::VectorXd task_velocites)
{
   // return this->jacobian_task_inverse(this->_joint_config)*task_velocites;
}

Eigen::VectorXd DynamicRobot::motor_torque_to_joint_torque(Eigen::VectorXd motor_torques)
{
    return this->jacobian_joint_inverse(this->_joint_config).transpose()*motor_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_motor_torque(Eigen::VectorXd joint_torques)
{
    return this->jacobian_joint(this->_joint_config).transpose()*joint_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_task_force(Eigen::VectorXd joint_torques)
{
   // return this->jacobian_task_inverse(this->_joint_config).transpose()*joint_torques;
}

Eigen::VectorXd DynamicRobot::task_force_to_joint_torque(Eigen::VectorXd task_forces)
{
   // return this->jacobian_task(this->_joint_config).transpose()*task_forces;
}

void DynamicRobot::addPeriodicTask(void *(*start_routine)(void *), int sched_policy, int priority, int cpu_affinity, void *arg, std::string task_name,int task_type, int period){
    pthread_t thread;
    pthread_attr_t tattr;
    sched_param param;
    pthread_attr_init(&tattr);
    pthread_attr_setschedpolicy(&tattr, sched_policy);
    pthread_attr_getschedparam (&tattr, &param);
    param.sched_priority = priority;
    pthread_attr_setschedparam(&tattr, &param);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_affinity, &cpuset);
    pthread_attr_setaffinity_np(&tattr, sizeof(cpu_set_t), &cpuset);

    std::tuple<void*, void*, int, int>* arg_tuple;
    arg_tuple = new std::tuple<void*, void*, int, int>(this,arg,period,task_type);

    int th = pthread_create( &thread, &tattr, start_routine, arg_tuple);
}