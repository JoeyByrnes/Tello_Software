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
    motor_velocities = _motor_direction_matrix*motor_velocities;
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

MotorPDConfig DynamicRobot::switchController(const MotorPDConfig& stanceCtrl, const MotorPDConfig& swingCtrl,
                               int durationMs, bool isSwingToStance, int currTimeStep) {
    MotorPDConfig output;

    int numTimeSteps = durationMs;  // Calculate the number of time steps to use for the transition

    if (!isSwingToStance) {
        currTimeStep = numTimeSteps - currTimeStep;  // If transitioning from stance to swing, invert the current time step
    }

    for (int i = 0; i < 12; i++) {
        double stancePos = stanceCtrl.motor_pos_desired[i];
        double swingPos = swingCtrl.motor_pos_desired[i];
        double stanceVel = stanceCtrl.motor_vel_desired[i];
        double swingVel = swingCtrl.motor_vel_desired[i];
        double stanceKp = stanceCtrl.motor_kp[i];
        double swingKp = swingCtrl.motor_kp[i];
        double stanceKd = stanceCtrl.motor_kd[i];
        double swingKd = swingCtrl.motor_kd[i];
        double stanceFF = stanceCtrl.motor_ff_torque[i];
        double swingFF = swingCtrl.motor_ff_torque[i];

        double switchFactor = currTimeStep / double(numTimeSteps);  // Calculate the switch factor based on the current time step and direction flag
        if (!isSwingToStance) {
            switchFactor = 1 - switchFactor;  // If transitioning from stance to swing, invert the switch factor
        }
        double smoothVal = sigmoid((switchFactor - 0.5) * 10);  // Scale the switch factor to be between -5 and 5, and apply sigmoid function
        double posDesired = (1 - smoothVal) * stancePos + smoothVal * swingPos;  // Smoothly transition between the position desired values
        double velDesired = (1 - smoothVal) * stanceVel + smoothVal * swingVel;  // Smoothly transition between the velocity desired values
        double kp = (1 - smoothVal) * stanceKp + smoothVal * swingKp;  // Smoothly transition between the proportional gain values
        double kd = (1 - smoothVal) * stanceKd + smoothVal * swingKd;  // Smoothly transition between the derivative gain values
        double ffTorque = (1 - smoothVal) * stanceFF + smoothVal * swingFF;  // Smoothly transition between the feedforward torque values

        output.motor_pos_desired[i] = posDesired;
        output.motor_vel_desired[i] = velDesired;
        output.motor_kp[i] = kp;
        output.motor_kd[i] = kd;
        output.motor_ff_torque[i] = ffTorque;
    }

    return output;
}

void DynamicRobot::motorPD(MotorPDConfig motor_conf)
{
    for(int i=0; i<this->_num_actuators; i++)
    {
        this->motors[i]->setKp(motor_conf.motor_kp[i]);
        this->motors[i]->setKd(motor_conf.motor_kd[i]);
        this->motors[i]->setPos(motor_conf.motor_pos_desired[i]);
        this->motors[i]->setVel(motor_conf.motor_vel_desired[i]);
        this->motors[i]->setff(motor_conf.motor_ff_torque[i]);
    }
}

void DynamicRobot::jointPD(JointPDConfig joint_conf)
{
	// Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

	// Calculate Joint PD
	VectorXd joint_torques = calc_pd(joint_positions,joint_velocities,joint_conf.joint_pos_desired,
                                     joint_conf.joint_vel_desired,joint_conf.joint_kp,joint_conf.joint_kd);

	// Convert joint PD torques to motor torques
	VectorXd motor_torques_from_joint_pd = this->joint_torque_to_motor_torque(joint_torques + joint_conf.joint_ff_torque);

    // Use inverse kinematics to calculate motor PD
    VectorXd motor_pos_desired = this->joint_pos_to_motor_pos(joint_conf.joint_pos_desired);
    VectorXd motor_vel_desired = VectorXd::Zero(10);//this->joint_vel_to_motor_vel(vel_desired);

    VectorXd motor_pos_desired_real(10);
    for(int i=0; i<10; i++){
        motor_pos_desired_real[i] = motor_pos_model_to_real(i, motor_pos_desired[i]);
        motor_vel_desired[i] = (int)((double)(motor_vel_desired[i]*motor_directions[i])/VELOCITY_TO_RADIANS_PER_SEC);
    }

    // Add the motor torques from the Joint PD as feedforward commands
    motor_torques_from_joint_pd = _motor_direction_matrix*motor_torques_from_joint_pd;

    MotorPDConfig motor_conf;
    motor_conf.motor_ff_torque = motor_torques_from_joint_pd;
    motor_conf.motor_kp = joint_conf.motor_kp;
    motor_conf.motor_kd = joint_conf.motor_kd;
    motor_conf.motor_pos_desired = motor_pos_desired_real;
    motor_conf.motor_vel_desired = motor_vel_desired;

    this->motorPD(motor_conf);
}

// pos/vel is 12x1 vector: (3x1) left front, (3x1) left back, (3x1) right front, (3x1) right back
void DynamicRobot::taskPD(TaskPDConfig task_conf)
{
    // Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

    // Get task space position and velocities
    VectorXd task_positions = joint_pos_to_task_pos(joint_positions);
    VectorXd task_velocities = joint_vel_to_task_vel(joint_velocities);

    // Calculate Task PD
    VectorXd task_forces = calc_pd(task_positions,task_velocities,task_conf.task_pos_desired,
                                   task_conf.task_vel_desired,task_conf.task_kp,task_conf.task_kd);

    // Get joint torques from task forces
    VectorXd joint_torques = this->task_force_to_joint_torque(task_forces+ task_conf.task_ff_force);

    // Use inverse kinematics to calculate joint pd
    VectorXd joint_pos_desired = this->task_pos_to_joint_pos(task_conf.task_pos_desired);
    VectorXd joint_vel_desired = VectorXd::Zero(10); //this->task_vel_to_joint_vel(vel_desired);

    JointPDConfig joint_conf;
    joint_conf.joint_ff_torque = joint_torques;
    joint_conf.joint_pos_desired = joint_pos_desired;
    joint_conf.joint_vel_desired = joint_vel_desired;
    joint_conf.joint_kp = task_conf.joint_kp;
    joint_conf.joint_kd = task_conf.joint_kd;
    joint_conf.motor_kp = task_conf.motor_kp;
    joint_conf.motor_kd = task_conf.motor_kd;

    jointPD(joint_conf);

}

void DynamicRobot::addGravityCompensation()
{
    // Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

    VectorXd gcf_fl = Vector3d(0, 0, -_balance_adjust);
    VectorXd gcf_bl = Vector3d(0, 0, -_balance_adjust);
    VectorXd gcf_fr = Vector3d(0, 0, -_balance_adjust);
    VectorXd gcf_br = Vector3d(0, 0, -_balance_adjust);

    VectorXd gcf_fl_world = transformForceToWorldFrame(gcf_fl, this->_ypr);
    VectorXd gcf_bl_world = transformForceToWorldFrame(gcf_bl, this->_ypr);
    VectorXd gcf_fr_world = transformForceToWorldFrame(gcf_fr, this->_ypr);
    VectorXd gcf_br_world = transformForceToWorldFrame(gcf_br, this->_ypr);

    VectorXd gravity_comp_world(12);
    gravity_comp_world << gcf_fl_world,gcf_bl_world,gcf_fr_world,gcf_br_world;

    // Get joint torques from task forces
    VectorXd joint_torques = this->task_force_to_joint_torque(gravity_comp_world);

    // Convert joint torques to motor torques
	VectorXd motor_torques = this->joint_torque_to_motor_torque(joint_torques);

    // Add the motor torques from the Task PD as feedforward commands
    motor_torques = _motor_direction_matrix*motor_torques;
    // printf("Motor T: %f, %f, %f, %f, %f      \r",motor_torques[0],motor_torques[1],motor_torques[2],motor_torques[3],motor_torques[4]);
    add_motor_torques(motor_torques);
}

Eigen::Vector3d DynamicRobot::transformForceToWorldFrame(const Eigen::VectorXd& force, VectorXd ypr) {
    Eigen::Matrix3d rotMat;
    rotMat = //Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(-ypr[1]*DEGREES_TO_RADIANS, Eigen::Vector3d::UnitY());// *
             //Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

    Eigen::VectorXd worldForce = rotMat * force;
    return worldForce;
}

Eigen::VectorXd DynamicRobot::calc_pd(VectorXd position, VectorXd velocity, VectorXd desiredPosition, VectorXd desiredVelocity, MatrixXd Kp, MatrixXd Kd) 
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

Eigen::Vector3d DynamicRobot::updatePosFromIMU(const Eigen::Vector3d& acc, const Eigen::Vector3d& ypr, double delta_t, Eigen::Vector3d& pos, Eigen::Vector3d& vel) {
  // Compute the change in velocity due to acceleration
  Eigen::Vector3d delta_v = acc * delta_t;

  // Update the velocity due to acceleration
  vel += delta_v;

  // Compute the change in position due to velocity and time
  Eigen::Vector3d delta_p = vel * delta_t;

  // Compute the rotation matrix based on roll, pitch, and yaw
  Eigen::Matrix3d rot_mat;
  rot_mat = (  Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX()));

  // Transform the change in position from the IMU frame to the world frame
  delta_p = rot_mat * delta_p;

  // Update the position of the IMU relative to the start position in the world frame
  pos += delta_p;

  // Return the updated position
  return pos;
}