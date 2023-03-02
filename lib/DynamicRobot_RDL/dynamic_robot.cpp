#include "dynamic_robot.h"

using namespace RoboDesignLab;
using namespace Eigen;

extern pthread_mutex_t mutex_CAN_recv;

uint64_t debug_print_idx = 0;

DynamicRobot::DynamicRobot(BipedActuatorTree actuators)
{ 
    this->_actuators = actuators; 
    _leg_DoF = 5;
    _num_actuators = 10;
}

Eigen::VectorXd DynamicRobot::motor_vel_to_joint_vel(Eigen::VectorXd motor_velocites)
{
    Eigen::VectorXd motor_velocites_left = motor_velocites.segment(0,5);
	Eigen::VectorXd motor_velocites_right = motor_velocites.segment(5,5);

    Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd joint_velocities_left  = this->jacobian_joint(joint_positions_left)*motor_velocites_left;
    Eigen::VectorXd joint_velocities_right = this->jacobian_joint(joint_positions_right)*motor_velocites_right;

    Eigen::VectorXd joint_velocities(10);
    joint_velocities << joint_velocities_left, joint_velocities_right;
    joint_velocities(4) = -joint_velocities(4);
	joint_velocities(9) = -joint_velocities(9);
    return joint_velocities;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_motor_vel(Eigen::VectorXd joint_velocites)
{
    Eigen::VectorXd joint_velocites_left = joint_velocites.segment(0,5);
	Eigen::VectorXd joint_velocites_right = joint_velocites.segment(5,5);

    Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd motor_velocities_left = this->jacobian_joint_inverse(joint_positions_left)*joint_velocites_left;
    Eigen::VectorXd motor_velocities_right = this->jacobian_joint_inverse(joint_positions_right)*joint_velocites_right;

    Eigen::VectorXd motor_velocities(10);
    motor_velocities << motor_velocities_left, motor_velocities_right;
    return motor_velocities;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_task_vel(Eigen::VectorXd joint_velocites)
{
    Eigen::VectorXd joint_velocites_left = joint_velocites.segment(0,5);
	Eigen::VectorXd joint_velocites_right = joint_velocites.segment(5,5);

    Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd task_velocities_left_front  = (this->jacobian_task_lf_front(joint_positions_left)*joint_velocites_left).segment(0,3);
    Eigen::VectorXd task_velocities_right_front = (this->jacobian_task_lf_front(joint_positions_right)*joint_velocites_right).segment(0,3);

    Eigen::VectorXd task_velocities_left_back  = (this->jacobian_task_lf_back(joint_positions_left)*joint_velocites_left).segment(0,3);
    Eigen::VectorXd task_velocities_right_back = (this->jacobian_task_lf_back(joint_positions_right)*joint_velocites_right).segment(0,3);

    Eigen::VectorXd task_velocities(12);
    task_velocities << task_velocities_left_front, task_velocities_left_back, task_velocities_right_front, task_velocities_right_back;
    return task_velocities;
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel(Eigen::VectorXd task_velocities)
{
    Eigen::VectorXd task_velocites_front_left(6);
    task_velocites_front_left << task_velocities.segment(0,3), 0, 0, 0;
    Eigen::VectorXd task_velocites_back_left(6);
    task_velocites_back_left << task_velocities.segment(3,3), 0, 0, 0;
    Eigen::VectorXd task_velocites_front_right(6);
    task_velocites_front_right << task_velocities.segment(6,3), 0, 0, 0;
    Eigen::VectorXd task_velocites_back_right(6);
    task_velocites_back_right << task_velocities.segment(9,3), 0, 0, 0;

    // return this->jacobian_task_inverse(this->getJointPositions())*task_velocites;
    Eigen::MatrixXd J_front_left = this->jacobian_task_lf_front(this->getJointPositions().segment(0,5));
    Eigen::MatrixXd J_back_left = this->jacobian_task_lf_back(this->getJointPositions().segment(0,5));

    Eigen::MatrixXd J_front_right = this->jacobian_task_lf_front(this->getJointPositions().segment(5,5));
    Eigen::MatrixXd J_back_right = this->jacobian_task_lf_back(this->getJointPositions().segment(5,5));

    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_front_left(J_front_left);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_back_left(J_back_left);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_front_right(J_front_right);
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod_back_right(J_back_right);

    Eigen::MatrixXd J_front_left_inverse = cod_front_left.pseudoInverse();
    Eigen::MatrixXd J_back_left_inverse = cod_back_left.pseudoInverse();

    Eigen::MatrixXd J_front_right_inverse = cod_front_right.pseudoInverse();
    Eigen::MatrixXd J_back_right_inverse = cod_back_right.pseudoInverse();

    Eigen::VectorXd joint_vels_from_front_left = J_front_left_inverse*task_velocites_front_left;
    Eigen::VectorXd joint_vels_from_back_left = J_back_left_inverse*task_velocites_back_left;
    Eigen::VectorXd joint_vels_from_front_right = J_front_right_inverse*task_velocites_front_right;
    Eigen::VectorXd joint_vels_from_back_right = J_back_right_inverse*task_velocites_back_right;

    Eigen::VectorXd joint_velocities(10);
    joint_velocities << joint_vels_from_front_left + joint_vels_from_back_left, joint_vels_from_front_right + joint_vels_from_back_right;
    return joint_velocities;
}

Eigen::VectorXd DynamicRobot::motor_torque_to_joint_torque(Eigen::VectorXd motor_torques)
{
    return this->jacobian_joint_inverse(this->getJointPositions()).transpose()*motor_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_motor_torque(Eigen::VectorXd joint_torques)
{
    // Account for incorrect jacobian convention
    joint_torques[4] = -joint_torques[4];
	joint_torques[9] = -joint_torques[9];

    Eigen::VectorXd joint_torques_left = joint_torques.segment(0,5);
	Eigen::VectorXd joint_torques_right = joint_torques.segment(5,5);

    Eigen::VectorXd joint_positions = this->getJointPositions();
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
    // return this->jacobian_task_inverse(this->getJointPositions()).transpose()*joint_torques;
    Eigen::Matrix<double,5,1> f;
    return f;
}

Eigen::VectorXd DynamicRobot::task_force_to_joint_torque(Eigen::VectorXd task_forces_front, Eigen::VectorXd task_forces_back)
{
    Eigen::VectorXd task_forces_front_left(6), task_forces_front_right(6), task_forces_back_left(6), task_forces_back_right(6);
    task_forces_front_left << task_forces_front.segment(0,3), 0, 0, 0;
    task_forces_front_right << task_forces_front.segment(3,3), 0, 0, 0;
    task_forces_back_left << task_forces_front.segment(0,3), 0, 0, 0;
    task_forces_back_right << task_forces_front.segment(3,3), 0, 0, 0;

    Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd torques_for_front_force_left = this->jacobian_task_lf_front(joint_positions_left).transpose()*task_forces_front_left;
    Eigen::VectorXd torques_for_back_force_left = this->jacobian_task_lf_back(joint_positions_left).transpose()*task_forces_back_left;

    Eigen::VectorXd torques_for_front_force_right = this->jacobian_task_lf_front(joint_positions_right).transpose()*task_forces_front_right;
    Eigen::VectorXd torques_for_back_force_right = this->jacobian_task_lf_back(joint_positions_right).transpose()*task_forces_back_right;

    Eigen::VectorXd joint_torques_left = torques_for_front_force_left + torques_for_back_force_left;
    Eigen::VectorXd joint_torques_right = torques_for_front_force_right + torques_for_back_force_right;
    Eigen::VectorXd joint_torques(10);
    joint_torques << joint_torques_left, joint_torques_right;
    return joint_torques;
}
Eigen::VectorXd DynamicRobot::task_force_to_joint_torque(Eigen::VectorXd task_forces)
{
    Eigen::VectorXd task_forces_front_left(6), task_forces_front_right(6), task_forces_back_left(6), task_forces_back_right(6);
    task_forces_front_left << task_forces.segment(0,3), 0, 0, 0;
    task_forces_back_left << task_forces.segment(3,3), 0, 0, 0;
    task_forces_front_right << task_forces.segment(6,3), 0, 0, 0;
    task_forces_back_right << task_forces.segment(9,3), 0, 0, 0;

    Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd torques_for_front_force_left = this->jacobian_task_lf_front(joint_positions_left).transpose()*task_forces_front_left;
    Eigen::VectorXd torques_for_back_force_left = this->jacobian_task_lf_back(joint_positions_left).transpose()*task_forces_back_left;

    Eigen::VectorXd torques_for_front_force_right = this->jacobian_task_lf_front(joint_positions_right).transpose()*task_forces_front_right;
    Eigen::VectorXd torques_for_back_force_right = this->jacobian_task_lf_back(joint_positions_right).transpose()*task_forces_back_right;

    Eigen::VectorXd joint_torques_left = torques_for_front_force_left + torques_for_back_force_left;
    Eigen::VectorXd joint_torques_right = torques_for_front_force_right + torques_for_back_force_right;
    Eigen::VectorXd joint_torques(10);
    joint_torques << joint_torques_left, joint_torques_right;
    return joint_torques;
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


int DynamicRobot::motor_pos_model_to_real(int id, double actuator_position_radians)
{
    return (int)((float)( actuator_position_radians)*((float)(this->motor_directions[id])/ENCODER_TO_RADIANS))+this->motor_zeros[id];
}

double DynamicRobot::motor_pos_real_to_model(int id, int motor_position_enc_counts)
{
    return ((double)(motor_position_enc_counts - this->motor_zeros[id]))*((double)(this->motor_directions[id]))*ENCODER_TO_RADIANS;
}

Eigen::VectorXd DynamicRobot::getJointPositions()
{
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
Eigen::VectorXd DynamicRobot::getJointVelocities()
{
    VectorXd motor_velocities(10);
	for(int i=0; i<10; i++)
    {
        motor_velocities(i) = (this->motors[i]->getMotorState().vel*VELOCITY_TO_RADIANS_PER_SEC-32.484131)*this->motor_directions[i];
    }
	VectorXd joint_velocities = this->motor_vel_to_joint_vel(motor_velocities);
    return joint_velocities;
}

void DynamicRobot::motorPD(VectorXd pos_desired, VectorXd vel_desired, VectorXd kp, VectorXd kd)
{
    for(int i=0; i<this->_num_actuators; i++)
    {
        this->motors[i]->setKp(kp[i]);
        this->motors[i]->setKd(kd[i]);
        this->motors[i]->setPos(pos_desired[i]);
        this->motors[i]->setVel(vel_desired[i]);
        this->motors[i]->setff(0);
    }
}

void DynamicRobot::jointPD(VectorXd pos_desired, VectorXd vel_desired, MatrixXd j_kp, MatrixXd j_kd, VectorXd m_kp, VectorXd m_kd)
{
	// Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

	// Calculate Joint PD
	VectorXd joint_torques = calc_pd_effort(joint_positions,joint_velocities,pos_desired,vel_desired,j_kp,j_kd);

	// Convert joint PD torques to motor torques
	VectorXd motor_torques_from_joint_pd = this->joint_torque_to_motor_torque(joint_torques);

    // Use inverse kinematics to calculate motor PD
    VectorXd motor_pos_desired = this->joint_pos_to_motor_pos(pos_desired);
    VectorXd motor_vel_desired = VectorXd::Zero(10);//this->joint_vel_to_motor_vel(vel_desired);

    VectorXd motor_pos_desired_real(10);
    for(int i=0; i<10; i++){
        motor_pos_desired_real[i] = motor_pos_model_to_real(i, motor_pos_desired[i]);
        motor_vel_desired[i] = (int)((double)(motor_vel_desired[i]*motor_directions[i])/VELOCITY_TO_RADIANS_PER_SEC);
    }
    this->motorPD(motor_pos_desired_real, motor_vel_desired,m_kp,m_kd);
	
	// Add the motor torques from the Joint PD as feedforward commands
    motor_torques_from_joint_pd = _motor_direction_matrix*motor_torques_from_joint_pd;

	set_motor_torques(motor_torques_from_joint_pd);
}

// pos/vel is 12x1 vector: (3x1) left front, (3x1) left back, (3x1) right front, (3x1) right back
void DynamicRobot::taskPD(VectorXd pos_desired, VectorXd vel_desired, MatrixXd t_kp, MatrixXd t_kd, MatrixXd j_kp, MatrixXd j_kd, VectorXd m_kp, VectorXd m_kd)
{
    // Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

    // Get task space position and velocities
    VectorXd task_positions = joint_pos_to_task_pos(joint_positions);
    VectorXd task_velocities = joint_vel_to_task_vel(joint_velocities);

    // Calculate Task PD
    VectorXd task_forces = calc_pd_effort(task_positions,task_velocities,pos_desired,vel_desired,t_kp,t_kd);

    // // Get joint torques from task forces
    VectorXd joint_torques = this->task_force_to_joint_torque(task_forces);

    // // Use inverse kinematics to calculate joint pd
    VectorXd joint_pos_desired = this->task_pos_to_joint_pos(pos_desired);
    VectorXd joint_vel_desired = VectorXd::Zero(10); //this->task_vel_to_joint_vel(vel_desired);

    jointPD(joint_pos_desired,joint_vel_desired,j_kp,j_kd,m_kp,m_kd);

    // Convert joint torques to motor torques
	VectorXd motor_torques = this->joint_torque_to_motor_torque(joint_torques);

    // Add the motor torques from the Task PD as feedforward commands
    motor_torques = _motor_direction_matrix*motor_torques;
    add_motor_torques(motor_torques);

    // if(debug_print_idx%200==0)
    // {
    //     printf("Joint des: %f,\t %f,\t %f,\t %f,\t %f       \r",    joint_pos_desired[0],
    //                                                                 joint_pos_desired[1],
    //                                                                 joint_pos_desired[2],
    //                                                                 joint_pos_desired[3],
    //                                                                 joint_pos_desired[4]);
    //     std::cout.flush();
    // }
    // debug_print_idx++;

}

Eigen::VectorXd DynamicRobot::calc_pd_effort(VectorXd position, VectorXd velocity, VectorXd desiredPosition, VectorXd desiredVelocity, MatrixXd Kp, MatrixXd Kd) 
{
  // Compute position error
  Eigen::VectorXd positionError = desiredPosition - position;
  // Compute velocity error
  Eigen::VectorXd velocityError = desiredVelocity - velocity;
  // Compute control output
  return Kp*positionError + Kd*velocityError;
}

void DynamicRobot::enable_all_motors()
{
	for(int i=0;i<10;i++)
	{
		this->motors[i]->enableMotor();
	}
}
void DynamicRobot::disable_all_motors()
{
	for(int i=0;i<10;i++)
	{
		this->motors[i]->disableMotor();
	}
}
void DynamicRobot::set_kp_kd_all_motors(uint16_t kp, uint16_t kd)
{
	for(int i=0;i<10;i++)
	{
			this->motors[i]->setKp(kp);
			this->motors[i]->setKd(kd);
	}
}
void DynamicRobot::update_all_motors()
{
	for(int i=0;i<10;i++)
	{
		this->motors[i]->updateMotor();
	}
}

void DynamicRobot::set_motor_torques(Eigen::VectorXd motor_torques)
{
    for(int i=0; i<_num_actuators; i++)
    {
		this->motors[i]->setff(motor_torques[i]);
	}
}
void DynamicRobot::add_motor_torques(Eigen::VectorXd motor_torques)
{
    for(int i=0; i<_num_actuators; i++)
    {
		this->motors[i]->addff(motor_torques[i]);
	}
}
