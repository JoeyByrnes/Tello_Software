#include "dynamic_robot.h"

using namespace RoboDesignLab;

Eigen::VectorXd DynamicRobot::motor_vel_to_joint_vel(Eigen::VectorXd motor_velocites)
{
    return this->jacobian_joint(this->getJointConfig())*motor_velocites;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_motor_vel(Eigen::VectorXd joint_velocites)
{
    return this->jacobian_joint_inverse(this->getJointConfig())*joint_velocites;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_task_vel(Eigen::VectorXd joint_velocites)
{
    //return this->jacobian_task(this->getJointConfig())*joint_velocites;
    return this->jacobian_task_lf_front(this->getJointConfig())*joint_velocites;
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel(Eigen::VectorXd task_velocites_front, Eigen::VectorXd task_velocites_back )
{
   // return this->jacobian_task_inverse(this->getJointConfig())*task_velocites;
   Eigen::MatrixXd J_front = this->jacobian_task_lf_front(this->getJointConfig());
   Eigen::MatrixXd J_back = this->jacobian_task_lf_back(this->getJointConfig());
   Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_front(J_front);
   Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_back(J_back);

   Eigen::MatrixXd J_front_inverse = cod_front.pseudoInverse();
   Eigen::MatrixXd J_back_inverse = cod_back.pseudoInverse();

   Eigen::VectorXd joint_vels_from_front = J_front_inverse*task_velocites_front;
   Eigen::VectorXd joint_vels_from_back = J_back_inverse*task_velocites_back;

   return joint_vels_from_front + joint_vels_from_back; // TODO: ask Guillermo, this seems wrong to add them this way
}

Eigen::VectorXd DynamicRobot::motor_torque_to_joint_torque(Eigen::VectorXd motor_torques)
{
    return this->jacobian_joint_inverse(this->getJointConfig()).transpose()*motor_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_motor_torque(Eigen::VectorXd joint_torques)
{
    Eigen::VectorXd joint_torques_left = joint_torques.segment(0,5);
	Eigen::VectorXd joint_torques_right = joint_torques.segment(5,5);

    Eigen::VectorXd joint_positions = this->getJointConfig();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd motor_torques_left  = this->jacobian_joint(joint_positions_left).transpose()*joint_torques_left;
    Eigen::VectorXd motor_torques_right = this->jacobian_joint(joint_positions_right).transpose()*joint_torques_right;

    Eigen::VectorXd motor_torques(10);
    motor_torques << motor_torques_left, motor_torques_right;
    return motor_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_task_force(Eigen::VectorXd joint_torques)
{
   // return this->jacobian_task_inverse(this->getJointConfig()).transpose()*joint_torques;
   Eigen::Matrix<double,5,1> f;
   return f;
}

Eigen::VectorXd DynamicRobot::task_force_to_joint_torque(Eigen::VectorXd task_forces_front, Eigen::VectorXd task_forces_back)
{
   // return this->jacobian_task(this->getJointConfig()).transpose()*task_forces;
   Eigen::VectorXd torques_for_front_force = this->jacobian_task_lf_front(this->getJointConfig()).transpose()*task_forces_front;
   Eigen::VectorXd torques_for_back_force = this->jacobian_task_lf_back(this->getJointConfig()).transpose()*task_forces_back;

   return torques_for_front_force + torques_for_back_force;
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


int DynamicRobot::motor_pos_model_to_real(int id, double joint_position_radians)
{
    return (int)((float)( joint_position_radians)*((float)(this->motor_directions[id])/ENCODER_TO_RADIANS))+this->motor_zeros[id];
}

double DynamicRobot::motor_pos_real_to_model(int id, int motor_position_units)
{
    return ((double)(motor_position_units - this->motor_zeros[id]))*((double)(this->motor_directions[id]))*ENCODER_TO_RADIANS;
}

Eigen::VectorXd DynamicRobot::getJointConfig(){
    Eigen::Matrix<double,10,1> joint_config;
    Eigen::Matrix<double,5,1> motor_positions_left; 
    Eigen::Matrix<double,5,1> motor_positions_right;
    for(int i=0;i<5;i++){
        motor_positions_left[i] = motor_pos_real_to_model(i, this->motors[i]->getMotorState().pos);
        motor_positions_right[i] = motor_pos_real_to_model(i+5, this->motors[i+5]->getMotorState().pos);
    }
    Eigen::VectorXd joint_pos_left = this->motor_pos_to_joint_pos(motor_positions_left);
    Eigen::VectorXd joint_pos_right = this->motor_pos_to_joint_pos(motor_positions_right);
    joint_config << joint_pos_left, joint_pos_right;
    return joint_config;
}

