#include "dynamic_robot.h"

using namespace RoboDesignLab;
using namespace Eigen;
using namespace std;

extern pthread_mutex_t mutex_CAN_recv;

uint64_t debug_print_idx = 0;

DynamicRobot::DynamicRobot()
{ 
    _leg_DoF = 5;
    _num_actuators = 10;

    // Initialize state mean
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    R0 << 1, 0, 0, // initial orientation
          0, 1, 0, // IMU frame is aligned with world frame
          0, 0, 1;
    v0 << 0,0,0; // initial velocity
    p0 << 0,0,0; // initial position
    bg0 << 0,0,0; // initial gyroscope bias
    ba0 << 0,0,0; // initial accelerometer bias
    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initialize state covariance
    NoiseParams noise_params;
    noise_params.setGyroscopeNoise(0.9);
    noise_params.setAccelerometerNoise(0.9);
    noise_params.setGyroscopeBiasNoise(0.0001);
    noise_params.setAccelerometerBiasNoise(0.0001);
    noise_params.setContactNoise(0.025);

    // Initialize filter
    InEKF filter(initial_state, noise_params);
    std::cout << "Noise parameters are initialized to: \n";
    std::cout << filter.getNoiseParams() << std::endl;
    std::cout << "Robot's state is initialized to: \n";
    std::cout << filter.getState() << std::endl;

    mapIntVector3d prior_landmarks;
    Eigen::Vector3d p_wl;
    int id;

    // Landmark at origin
    id = 1;
    p_wl << 0,0,0;
    prior_landmarks.insert(pair<int,Eigen::Vector3d> (id, p_wl)); 

    filter.setPriorLandmarks(prior_landmarks); 

    controller = new SRBMController(); // set to empty or 1 for mujoco sim initialization


}

DynamicRobot::DynamicRobot(const DynamicRobot& other) {
        // Copy CheetahMotor pointers
        // for (int i = 0; i < 10; i++) {
        //     motors[i] = new CheetahMotor(*other.motors[i]);
        // }

        // Copy other member variables
        _motor_direction_matrix = other._motor_direction_matrix;
        _balance_adjust = other._balance_adjust;
        _GRFs = other._GRFs;
        _GRF_biases = other._GRF_biases;
        _right_loadcells_calibrated = other._right_loadcells_calibrated;
        _left_loadcells_calibrated = other._left_loadcells_calibrated;
        isSimulation = other.isSimulation;
        sim_joint_torques = other.sim_joint_torques;
        sim_joint_pos = other.sim_joint_pos;
        sim_joint_vel = other.sim_joint_vel;
        controller = new SRBMController(*other.controller);
        // plot_data = other.plot_data;
        // plot_mat = other.plot_mat;

        // Copy Kinematics Functions
        _jaco_motor2joint = other._jaco_motor2joint;
        _jaco_joint2motor = other._jaco_joint2motor;
        _jaco_joint2taskFront = other._jaco_joint2taskFront;
        _jaco_joint2taskBack = other._jaco_joint2taskBack;
        _jaco_accel_task_to_joint = other._jaco_accel_task_to_joint;
        _ik_joint2motor = other._ik_joint2motor;
        _fk_motor2joint = other._fk_motor2joint;
        _ik_task2joint = other._ik_task2joint;
        _fk_joint2task = other._fk_joint2task;

        // Copy Actuators
        _actuators = other._actuators;
        _leg_DoF = other._leg_DoF;
        _num_actuators = other._num_actuators;

        // Copy InEKF
        // initial_state = other.initial_state;
        // noise_params = other.noise_params;
        // filter = other.filter;
        _imu_data = other._imu_data;
        _imu_data_prev = other._imu_data_prev;
        _ground_contacts = other._ground_contacts;
        _direct_lfv_hip = other._direct_lfv_hip;
        // _ekf_q = other._ekf_q;
    }

Eigen::VectorXd DynamicRobot::motor_vel_to_joint_vel(Eigen::VectorXd motor_velocites, Eigen::VectorXd joint_positions)
{
    Eigen::VectorXd motor_velocites_left = motor_velocites.segment(0,5);
	Eigen::VectorXd motor_velocites_right = motor_velocites.segment(5,5);

    // Eigen::VectorXd joint_positions = this->getJointPositions();
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

Eigen::VectorXd DynamicRobot::joint_vel_to_motor_vel(Eigen::VectorXd joint_velocites, Eigen::VectorXd joint_positions)
{
    Eigen::VectorXd joint_velocites_left = joint_velocites.segment(0,5);
	Eigen::VectorXd joint_velocites_right = joint_velocites.segment(5,5);

    // Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd motor_velocities_left = this->jacobian_joint_inverse(joint_positions_left)*joint_velocites_left;
    Eigen::VectorXd motor_velocities_right = this->jacobian_joint_inverse(joint_positions_right)*joint_velocites_right;

    Eigen::VectorXd motor_velocities(10);
    motor_velocities << motor_velocities_left, motor_velocities_right;
    motor_velocities = _motor_direction_matrix*motor_velocities;
    return motor_velocities;
}

Eigen::VectorXd DynamicRobot::joint_vel_to_task_vel(Eigen::VectorXd joint_velocites, Eigen::VectorXd joint_positions)
{
    Eigen::VectorXd joint_velocites_left = joint_velocites.segment(0,5);
	Eigen::VectorXd joint_velocites_right = joint_velocites.segment(5,5);

    // Eigen::VectorXd joint_positions = this->getJointPositions();
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

struct ThreadData {
    Eigen::MatrixXd* matrix;
    Eigen::MatrixXd* result;
    int coreId;
};

// Function to be executed by each thread
void* calculatePseudoInverse(void* threadData) {
    ThreadData* data = static_cast<ThreadData*>(threadData);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(data->coreId, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    *(data->result) = data->matrix->completeOrthogonalDecomposition().pseudoInverse();
    pthread_exit(NULL);
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel_right(Eigen::VectorXd task_velocities, TaskPDConfig task_conf, Eigen::VectorXd joint_positions)
{
    if(task_conf.use_single_jacoian)
    {
        Eigen::VectorXd task_velocites_front_right(3);
        task_velocites_front_right << task_velocities.segment(6,3);
        
        Eigen::MatrixXd J_front_right = this->jacobian_task_lf_front(joint_positions.segment(5,5)).topRows(3);

        Eigen::MatrixXd J_front_right_inverse = J_front_right.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::VectorXd joint_vels = (J_front_right_inverse*task_velocites_front_right)*2;

        return joint_vels;
    }
    else // use separate jacobians only if task velocities are different
    {
        Eigen::VectorXd task_velocites_front_right(3);
        task_velocites_front_right << task_velocities.segment(6,3);
        Eigen::VectorXd task_velocites_back_right(3);
        task_velocites_back_right << task_velocities.segment(9,3);

        Eigen::MatrixXd J_front_right = this->jacobian_task_lf_front(joint_positions.segment(5,5)).topRows(3);
        Eigen::MatrixXd J_back_right = this->jacobian_task_lf_back(joint_positions.segment(5,5)).topRows(3);

        Eigen::MatrixXd J_front_right_inverse = J_front_right.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd J_back_right_inverse = J_back_right.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::VectorXd joint_vels_from_front_right = J_front_right_inverse*task_velocites_front_right;
        Eigen::VectorXd joint_vels_from_back_right = J_back_right_inverse*task_velocites_back_right;

        Eigen::VectorXd joint_velocities(5);
        joint_velocities << joint_vels_from_front_right + joint_vels_from_back_right;
        return joint_velocities;
    }
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel_left(Eigen::VectorXd task_velocities, TaskPDConfig task_conf, Eigen::VectorXd joint_positions)
{
    if(task_conf.use_single_jacoian)
    {
        Eigen::VectorXd task_velocites_front_left(3);
        task_velocites_front_left << task_velocities.segment(0,3);

        Eigen::MatrixXd J_front_left = this->jacobian_task_lf_front(joint_positions.segment(0,5)).topRows(3);

        Eigen::MatrixXd J_front_left_inverse = J_front_left.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::VectorXd joint_vels = (J_front_left_inverse*task_velocites_front_left)*2;

        return joint_vels;
    }
    else // use separate jacobians only if task velocities are different
    {
        Eigen::VectorXd task_velocites_front_left(3);
        task_velocites_front_left << task_velocities.segment(0,3);
        Eigen::VectorXd task_velocites_back_left(3);
        task_velocites_back_left << task_velocities.segment(3,3);

        Eigen::MatrixXd J_front_left = this->jacobian_task_lf_front(joint_positions.segment(0,5)).topRows(3);
        Eigen::MatrixXd J_back_left = this->jacobian_task_lf_back(joint_positions.segment(0,5)).topRows(3);

        Eigen::MatrixXd J_front_left_inverse = J_front_left.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd J_back_left_inverse = J_back_left.completeOrthogonalDecomposition().pseudoInverse();

        Eigen::VectorXd joint_vels_from_front_left = J_front_left_inverse*task_velocites_front_left;
        Eigen::VectorXd joint_vels_from_back_left = J_back_left_inverse*task_velocites_back_left;

        Eigen::VectorXd joint_velocities(5);
        joint_velocities << joint_vels_from_front_left + joint_vels_from_back_left;
        return joint_velocities;
    }
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel(Eigen::VectorXd task_velocities,TaskPDConfig task_conf, Eigen::VectorXd joint_positions)
{
    if(task_conf.side == BOTH_LEGS)
    {
        return task_vel_to_joint_vel(task_velocities, joint_positions);
    }
    else if(task_conf.side == RIGHT_LEG)
    {
        Eigen::VectorXd joint_velocities = VectorXd::Zero(10);
        joint_velocities.head(5) = task_vel_to_joint_vel_right(task_velocities,task_conf, joint_positions);
        return joint_velocities;
    }
    else//if(task_conf.side == LEFT_LEG)
    { 
        Eigen::VectorXd joint_velocities = VectorXd::Zero(10);
        joint_velocities.tail(5) = task_vel_to_joint_vel_left(task_velocities,task_conf, joint_positions);
        return joint_velocities;
    }
}

Eigen::VectorXd DynamicRobot::task_vel_to_joint_vel(Eigen::VectorXd task_velocities, Eigen::VectorXd joint_positions)
{
    Eigen::VectorXd task_velocites_front_left(3);
    task_velocites_front_left << task_velocities.segment(0,3);
    Eigen::VectorXd task_velocites_back_left(3);
    task_velocites_back_left << task_velocities.segment(3,3);
    Eigen::VectorXd task_velocites_front_right(3);
    task_velocites_front_right << task_velocities.segment(6,3);
    Eigen::VectorXd task_velocites_back_right(3);
    task_velocites_back_right << task_velocities.segment(9,3);

    Eigen::MatrixXd J_front_left = this->jacobian_task_lf_front(joint_positions.segment(0,5)).topRows(3);
    Eigen::MatrixXd J_back_left = this->jacobian_task_lf_back(joint_positions.segment(0,5)).topRows(3);

    Eigen::MatrixXd J_front_right = this->jacobian_task_lf_front(joint_positions.segment(5,5)).topRows(3);
    Eigen::MatrixXd J_back_right = this->jacobian_task_lf_back(joint_positions.segment(5,5)).topRows(3);

    // pthread_t threads[4];
    // ThreadData threadData[4];

    // Eigen::MatrixXd J_front_left_inverse(J_front_left.cols(),J_front_left.rows());
    // Eigen::MatrixXd J_back_left_inverse(J_back_left.cols(),J_back_left.rows());
    // Eigen::MatrixXd J_front_right_inverse(J_front_right.cols(),J_front_right.rows());
    // Eigen::MatrixXd J_back_right_inverse(J_back_right.cols(),J_back_right.rows());

    // // Set up thread data
    // threadData[0].matrix = &J_front_left;
    // threadData[0].result = &J_front_left_inverse;
    // threadData[0].coreId = 12;

    // threadData[1].matrix = &J_back_left;
    // threadData[1].result = &J_back_left_inverse;
    // threadData[1].coreId = 13;

    // threadData[2].matrix = &J_front_right;
    // threadData[2].result = &J_front_right_inverse;
    // threadData[2].coreId = 14;

    // threadData[3].matrix = &J_back_right;
    // threadData[3].result = &J_back_right_inverse;
    // threadData[3].coreId = 15;

    // // Create threads and start calculations
    // for (int i = 0; i < 4; ++i) {
    //     pthread_create(&threads[i], NULL, calculatePseudoInverse, static_cast<void*>(&threadData[i]));
    // }

    // // Wait for threads to finish
    // for (int i = 0; i < 4; ++i) {
    //     pthread_join(threads[i], NULL);
    // }

    Eigen::MatrixXd J_front_left_inverse = J_front_left.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd J_back_left_inverse = J_back_left.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd J_front_right_inverse = J_front_right.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd J_back_right_inverse = J_back_right.completeOrthogonalDecomposition().pseudoInverse();

    
    Eigen::VectorXd joint_vels_from_front_left = J_front_left_inverse*task_velocites_front_left;
    Eigen::VectorXd joint_vels_from_back_left = J_back_left_inverse*task_velocites_back_left;
    Eigen::VectorXd joint_vels_from_front_right = J_front_right_inverse*task_velocites_front_right;
    Eigen::VectorXd joint_vels_from_back_right = J_back_right_inverse*task_velocites_back_right;

    Eigen::VectorXd joint_velocities(10);
    joint_velocities << joint_vels_from_front_left + joint_vels_from_back_left, joint_vels_from_front_right + joint_vels_from_back_right;
    return joint_velocities;
}

Eigen::VectorXd DynamicRobot::motor_torque_to_joint_torque(Eigen::VectorXd motor_torques, Eigen::VectorXd joint_positions)
{
    return this->jacobian_joint_inverse(joint_positions).transpose()*motor_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_motor_torque(Eigen::VectorXd joint_torques, Eigen::VectorXd joint_positions)
{
    // Account for incorrect jacobian convention
    joint_torques[4] = -joint_torques[4];
	joint_torques[9] = -joint_torques[9];

    Eigen::VectorXd joint_torques_left = joint_torques.segment(0,5);
	Eigen::VectorXd joint_torques_right = joint_torques.segment(5,5);

    // Eigen::VectorXd joint_positions = this->getJointPositions();
    Eigen::VectorXd joint_positions_left = joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd motor_torques_left  = this->jacobian_joint(joint_positions_left).transpose()*joint_torques_left;
    Eigen::VectorXd motor_torques_right = this->jacobian_joint(joint_positions_right).transpose()*joint_torques_right;

    Eigen::VectorXd motor_torques(10);
    motor_torques << motor_torques_left, motor_torques_right;
    return motor_torques;
}

Eigen::VectorXd DynamicRobot::joint_torque_to_task_force(Eigen::VectorXd joint_torques, Eigen::VectorXd joint_positions)
{
    // return this->jacobian_task_inverse(this->getJointPositions()).transpose()*joint_torques;
    Eigen::Matrix<double,5,1> f; // TODO
    return f;
}

Eigen::VectorXd DynamicRobot::task_force_to_joint_torque(Eigen::VectorXd task_forces_front, Eigen::VectorXd task_forces_back, Eigen::VectorXd joint_positions)
{
    Eigen::VectorXd task_forces_front_left(6), task_forces_front_right(6), task_forces_back_left(6), task_forces_back_right(6);
    task_forces_front_left << task_forces_front.segment(0,3), 0, 0, 0;
    task_forces_front_right << task_forces_front.segment(3,3), 0, 0, 0;
    task_forces_back_left << task_forces_front.segment(0,3), 0, 0, 0;
    task_forces_back_right << task_forces_front.segment(3,3), 0, 0, 0;

    // Eigen::VectorXd joint_positions = this->getJointPositions();
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
Eigen::VectorXd DynamicRobot::task_force_to_joint_torque(Eigen::VectorXd task_forces, Eigen::VectorXd joint_positions)
{
    Eigen::VectorXd task_forces_front_left(6), task_forces_front_right(6), task_forces_back_left(6), task_forces_back_right(6);
    task_forces_front_left << task_forces.segment(0,3), 0, 0, 0;
    task_forces_back_left << task_forces.segment(3,3), 0, 0, 0;
    task_forces_front_right << task_forces.segment(6,3), 0, 0, 0;
    task_forces_back_right << task_forces.segment(9,3), 0, 0, 0;

    // Eigen::VectorXd joint_positions = this->getJointPositions();
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

// xddot = Jdot(q, qdot)*qdot + J(q)*qddot
// qddot = J_inv(q)*( xddot - Jdot(q, qdot)*qdot )
VectorXd DynamicRobot::task_accel_to_joint_accel(Eigen::VectorXd task_accel, Eigen::VectorXd joint_positions, Eigen::VectorXd joint_velocities)
{
    // Eigen::VectorXd joint_positions = this->getJointPositions();
    // Eigen::VectorXd joint_velocities = this->getJointVelocities();

    Eigen::VectorXd joint_positions_left =  joint_positions.segment(0,5);
    Eigen::VectorXd joint_positions_right = joint_positions.segment(5,5);

    Eigen::VectorXd joint_velocities_left =  joint_velocities.segment(0,5);
    Eigen::VectorXd joint_velocities_right = joint_velocities.segment(5,5);

    Eigen::VectorXd task_accel_front_left(3), task_accel_front_right(3), task_accel_back_left(3), task_accel_back_right(3);

    task_accel_front_left << task_accel.segment(0,3);
    task_accel_back_left << task_accel.segment(3,3);
    task_accel_front_right << task_accel.segment(6,3);
    task_accel_back_right << task_accel.segment(9,3);

    Eigen::MatrixXd J_front_left = this->jacobian_task_lf_front(this->getJointPositions().segment(0,5)).topRows(3);
    Eigen::MatrixXd J_front_left_inverse = J_front_left.completeOrthogonalDecomposition().pseudoInverse();

    // cout << "q: ==============================================" << endl;
    // cout << joint_positions.transpose() << endl;
    // cout << endl;
    // cout << "q_dot: ==============================================" << endl;
    // cout << joint_velocities.transpose() << endl;
    // cout << endl;
    // cout << "Jacobian =================================" << endl;
    // cout << J_front_left << endl;
    // cout << endl;
    // cout << "Jacobian Inverse =================================" << endl;
    // cout << J_front_left_inverse << endl;
    // cout << endl;
    // cout << "Jacobian_dot =================================" << endl;
    // cout << this->jacobian_task_accel_lf_front(joint_positions_left,joint_velocities_left) << endl;
    // cout << endl;



    Eigen::VectorXd joint_accel_fl = J_front_left_inverse*(task_accel_front_left - this->jacobian_task_accel_lf_front(joint_positions_left,joint_velocities_left)*joint_velocities_left);
    Eigen::VectorXd joint_accel_bl = joint_accel_fl;
    Eigen::VectorXd joint_accel_fr = J_front_left_inverse*(task_accel_front_right - this->jacobian_task_accel_lf_front(joint_positions_right,joint_velocities_right)*joint_velocities_right);
    Eigen::VectorXd joint_accel_br = joint_accel_fr;

    VectorXd joint_accel(10);
    joint_accel << (joint_accel_fl + joint_accel_bl) , (joint_accel_fr + joint_accel_br);
    return joint_accel;
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

    // // Free the dynamically allocated memory
    // delete arg_tuple;
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
    if(this->isSimulation)
    {
        return sim_joint_pos;
    }
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

    // overwrite joint positions with joint encoder measurements:
    joint_config[1] = joint_encoder_positions[1];
    joint_config[2] = joint_encoder_positions[2];
    joint_config[3] = joint_encoder_positions[3];
    joint_config[4] = joint_encoder_positions[4];

    joint_config[6] = joint_encoder_positions[6];
    joint_config[7] = joint_encoder_positions[7];
    joint_config[8] = joint_encoder_positions[8];
    joint_config[9] = joint_encoder_positions[9];
    return joint_config;
}
Eigen::VectorXd DynamicRobot::getJointVelocities()
{
    if(this->isSimulation)
    {
        return sim_joint_vel;
    }
    VectorXd motor_velocities(10);
	for(int i=0; i<10; i++)
    {
        motor_velocities(i) = (this->motors[i]->getMotorState().vel*VELOCITY_TO_RADIANS_PER_SEC-32.484131)*this->motor_directions[i];
    }
	VectorXd joint_velocities = this->motor_vel_to_joint_vel(motor_velocities,this->getJointPositions());
    return joint_velocities;
}

void DynamicRobot::setJointEncoderPosition(double rad, JointName joint_name)
{
    joint_encoder_positions[joint_name] = rad;
}

void DynamicRobot::setJointEncoderVelocity(double radpersec, JointName joint_name)
{
    joint_encoder_velocities[joint_name] = radpersec;
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

VectorXd DynamicRobot::swing_stance_mux(VectorXd stanceTorques, VectorXd swingTorques, double duration_sec, bool isSwingToStance, double currTimeStep, int side) {

    if(currTimeStep >= duration_sec) currTimeStep = duration_sec;

    VectorXd smoothedTorques(10);

    // compute the sigmoid function input for each element in the vector
    double switchFactor;
    if(duration_sec == 0) switchFactor = 1;
    else switchFactor = currTimeStep / duration_sec;  // Calculate the switch factor based on the current time step and direction flag

    if(switchFactor > 1) switchFactor = 1;
    if (isSwingToStance) {
        // return stanceTorques;
        switchFactor = 1 - switchFactor;  // If transitioning from stance to swing, invert the switch factor
    }
    double smoothVal = sigmoid((switchFactor - 0.5) * 12);  // Scale the switch factor to be between -5 and 5, and apply sigmoid function
    if(smoothVal < 0.05) smoothVal = 0;
    if(smoothVal > 0.95) smoothVal = 1;

    smoothedTorques = ((1-smoothVal) * stanceTorques) + (smoothVal * swingTorques);

    return smoothedTorques;
}

void DynamicRobot::motorPD(MotorPDConfig motor_conf)
{
    for(int i=0; i<this->_num_actuators; i++)
    {
        this->motors[i]->setKp(motor_conf.motor_kp[i]);
        this->motors[i]->setKd(motor_conf.motor_kd[i]);
        this->motors[i]->setPos(motor_conf.motor_pos_desired[i]);
        this->motors[i]->setVel(motor_conf.motor_vel_desired[i]);
        this->motors[i]->setff((double)motor_conf.motor_ff_torque[i]*NM_TO_MOTOR_TORQUE_CMD);
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
	VectorXd motor_torques_from_joint_pd = this->joint_torque_to_motor_torque(joint_torques + joint_conf.joint_ff_torque, this->getJointPositions());

    // Use inverse kinematics to calculate motor PD
    VectorXd motor_pos_desired = this->joint_pos_to_motor_pos(joint_conf.joint_pos_desired);
    VectorXd motor_vel_desired = this->joint_vel_to_motor_vel(joint_conf.joint_vel_desired, joint_positions); // VectorXd::Zero(10);

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

    // if(this->isSimulation){
    //     this->sim_joint_torques << joint_torques + joint_conf.joint_ff_torque;
    // }
    // else{
        this->motorPD(motor_conf);
    // }
}

// pos/vel is 12x1 vector: (3x1) left front, (3x1) left back, (3x1) right front, (3x1) right back
void DynamicRobot::taskPD(TaskPDConfig task_conf)
{
    // Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

    // Get task space position and velocities
    VectorXd task_positions = joint_pos_to_task_pos(joint_positions);
    VectorXd task_velocities = joint_vel_to_task_vel(joint_velocities, this->getJointPositions());

    // Calculate Task PD
    VectorXd task_forces = calc_pd(task_positions,task_velocities,task_conf.task_pos_desired,
                                   task_conf.task_vel_desired,task_conf.task_kp,task_conf.task_kd);

    // Get joint torques from task forces
    VectorXd joint_torques = this->task_force_to_joint_torque(task_forces+ task_conf.task_ff_force, this->getJointPositions());

    // Use inverse kinematics to calculate joint pd
    VectorXd joint_pos_desired = this->task_pos_to_joint_pos(task_conf.task_pos_desired);
    VectorXd joint_vel_desired = this->task_vel_to_joint_vel(task_conf.task_vel_desired, joint_positions); //VectorXd::Zero(10);

    JointPDConfig joint_conf;
    joint_conf.joint_ff_torque = joint_torques+task_conf.joint_ff_torque;
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
    VectorXd joint_torques = this->task_force_to_joint_torque(gravity_comp_world, this->getJointPositions());

    // Convert joint torques to motor torques
	VectorXd motor_torques = this->joint_torque_to_motor_torque(joint_torques, this->getJointPositions());

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








// ========== SIMULATION ==================

VectorXd DynamicRobot::jointPD2(JointPDConfig joint_conf)
{
	// Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

	// Calculate Joint PD
    
	VectorXd joint_torques = calc_pd(joint_positions,joint_velocities,joint_conf.joint_pos_desired,
                                     joint_conf.joint_vel_desired,joint_conf.joint_kp,joint_conf.joint_kd);

	// Convert joint PD torques to motor torques
	VectorXd motor_torques_from_joint_pd = this->joint_torque_to_motor_torque(joint_torques + joint_conf.joint_ff_torque, this->getJointPositions());

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

    // cout << "Joint Torques JPD:   " << (joint_torques).transpose() << endl;

    // cout << "Joint Torques + FF:   " << (joint_torques + joint_conf.joint_ff_torque).transpose() << endl;

    return (joint_torques + joint_conf.joint_ff_torque);
}



VectorXd DynamicRobot::taskPD2(TaskPDConfig task_conf)
{
    // Get joint positions and velocities
	VectorXd joint_positions = this->getJointPositions();
	VectorXd joint_velocities = this->getJointVelocities();

    // Get task space position and velocities
    VectorXd task_positions = joint_pos_to_task_pos(joint_positions);
    VectorXd task_velocities = joint_vel_to_task_vel(joint_velocities, this->getJointPositions());

    // Calculate Task PD
    VectorXd task_forces = calc_pd(task_positions,task_velocities,task_conf.task_pos_desired,
                                   task_conf.task_vel_desired,task_conf.task_kp,task_conf.task_kd);


    // Get joint torques from task forces
    VectorXd joint_torques = this->task_force_to_joint_torque(task_forces+ task_conf.task_ff_force, this->getJointPositions());

    // Use inverse kinematics to calculate joint pd
    VectorXd joint_pos_desired = this->task_pos_to_joint_pos(task_conf.task_pos_desired);

    VectorXd joint_vel_desired;
    if(task_conf.ignore_joint_velocity)
    {
        joint_vel_desired = VectorXd::Zero(10);
    }
    else
    {
        joint_vel_desired = this->task_vel_to_joint_vel(task_conf.task_vel_desired,task_conf, joint_pos_desired);
    }

    VectorXd joint_accel_from_task_accel = this->task_accel_to_joint_accel(task_conf.task_ff_accel, joint_pos_desired, joint_vel_desired);

    VectorXd joint_forces_from_accel = task_conf.joint_ka*joint_accel_from_task_accel;

    // cout << "Torque from FF accel:" << endl;
    // cout << joint_forces_from_accel.transpose() << endl;

    JointPDConfig joint_conf;
    joint_conf.joint_ff_torque = joint_torques;// + joint_forces_from_accel;
    joint_conf.joint_pos_desired = joint_pos_desired;
    joint_conf.joint_vel_desired = joint_vel_desired;
    joint_conf.joint_kp = task_conf.joint_kp;
    joint_conf.joint_kd = task_conf.joint_kd;
    joint_conf.motor_kp = task_conf.motor_kp;
    joint_conf.motor_kd = task_conf.motor_kd;

    return jointPD2(joint_conf);

}

// InEKF Functions:

void DynamicRobot::update_filter_IMU_data(IMU_data imu_data)
{
    _imu_data = imu_data;
    Eigen::Matrix<double,6,1> imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero(); // head(3) = gyro, tail(3) = acc

    imu_measurement_prev << _imu_data_prev.gyro, _imu_data_prev.acc;

    double dt = _imu_data.timestamp - _imu_data_prev.timestamp;
    if (dt > DT_MIN && dt < DT_MAX) {
        filter.Propagate(imu_measurement_prev, dt);
    }
    _imu_data_prev = _imu_data;
}

void DynamicRobot::update_filter_contact_data(VectorXd ground_contacts)
{
    _ground_contacts = ground_contacts;

    vector<pair<int,bool> > contacts;

    for(int i=0;i<4;i++)
    {
        contacts.push_back(pair<int,bool> (i, (bool)_ground_contacts(i)));
    }
    // Set filter's contact state
    filter.setContacts(contacts);

}

Matrix3d foot_orientation(const Vector3d& a, const Vector3d& b, const Vector3d& c) { // front, back, ankle
    Vector3d x = b - a;
    x.normalize();
    Vector3d z = x.cross(c - a);
    z.normalize();
    Vector3d y = z.cross(x);
    Matrix3d rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    return rot;
}

void DynamicRobot::update_filter_kinematic_data(MatrixXd lfv_hip, Matrix3d R_right, Matrix3d R_left)
{
    Eigen::Vector3d p; // position of contact relative to the body
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    vectorKinematics measured_kinematics;
    Eigen::Matrix3d Rfoot;
    Rfoot <<    1, 0, 0, // foot to imu rotation
                0, 1, 0,
                0, 0, 1;

    covariance.setZero(); // just using ideal covariance matrix for now
    
    double W = 0.252;
    double CoM2H_z_dist = 0.18;

    Vector3d CoM2hr(0,-W/2.0,-CoM2H_z_dist);
    Vector3d CoM2hl(0, W/2.0,-CoM2H_z_dist);

    Vector3d right_front_in_hip = lfv_hip.row(0);
    Vector3d right_back_in_hip = lfv_hip.row(1);
    Vector3d left_front_in_hip = lfv_hip.row(2);
    Vector3d left_back_in_hip = lfv_hip.row(3);

    Eigen::Vector3d right_front_in_CoM = right_front_in_hip+CoM2hr;
    Eigen::Vector3d right_back_in_CoM = right_back_in_hip+CoM2hr;
    Eigen::Vector3d left_front_in_CoM = left_front_in_hip+CoM2hl;
    Eigen::Vector3d left_back_in_CoM = left_back_in_hip+CoM2hl;

    Matrix4d pose_right_front = Matrix4d::Identity();
    pose_right_front.block<3,1>(0,3) = right_front_in_CoM;
    pose_right_front.block<3,3>(0,0) = R_right;

    Matrix4d pose_right_back = Matrix4d::Identity();
    pose_right_back.block<3,1>(0,3) = right_back_in_CoM;
    pose_right_back.block<3,3>(0,0) = R_right;

    Matrix4d pose_left_front = Matrix4d::Identity();
    pose_left_front.block<3,1>(0,3) = left_front_in_CoM;
    pose_left_front.block<3,3>(0,0) = R_left;

    Matrix4d pose_left_back = Matrix4d::Identity();
    pose_left_back.block<3,1>(0,3) = left_back_in_CoM;
    pose_left_back.block<3,3>(0,0) = R_left;

    inekf::Kinematics frame_rf(0, pose_right_front, covariance);
    inekf::Kinematics frame_rb(1, pose_right_back, covariance);
    inekf::Kinematics frame_lf(2, pose_left_front, covariance);
    inekf::Kinematics frame_lb(3, pose_left_back, covariance);
    measured_kinematics.push_back(frame_rf);
    measured_kinematics.push_back(frame_rb);
    measured_kinematics.push_back(frame_lf);
    measured_kinematics.push_back(frame_lb);

    plot_data << right_front_in_CoM,right_back_in_CoM,left_front_in_CoM,left_back_in_CoM,VectorXd::Zero(8);

    plot_mat = R_left;

    filter.CorrectKinematics(measured_kinematics);
}

void DynamicRobot::update_filter_landmark_data(int id, Vector3d landmark_pos)
{
    vectorLandmarks measured_landmarks;
    Eigen::Matrix3d covariance = Matrix3d::Zero();
    Landmark landmark(id, landmark_pos, covariance);
    measured_landmarks.push_back(landmark); 
    filter.CorrectLandmarks(measured_landmarks);
}

RobotState DynamicRobot::get_filter_state()
{
    return filter.getState();
}

void DynamicRobot::set_filter_state(RobotState state)
{
    filter.setState(state);
}

Eigen::VectorXd DynamicRobot::get_CoP() 
{
    Eigen::VectorXd CoP(3);
    CoP.setZero();

    // calculation based on GRFs and moment
    Eigen::MatrixXd pi = this->controller->get_lfv_world().transpose();
    Eigen::VectorXd fiz(4);
    fiz << this->_GRFs.right_front, this->_GRFs.right_back, this->_GRFs.left_front, this->_GRFs.left_back;
    CoP = (pi.col(0) * fiz(0) + pi.col(1) * fiz(1) + pi.col(2) * fiz(2) + pi.col(3) * fiz(3)) / fiz.sum();

    return CoP;
}

