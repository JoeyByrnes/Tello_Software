#include <state_estimator.h>

bool filter_data_ready = false;
extern pthread_mutex_t EKF_mutex;

void* state_estimation( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
	// Print the core and priority of the thread
	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Mujoco thread running on core %d, with priority %d\n", core, priority);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while(1)
    {
        handle_start_of_periodic_task(next);
        

        while(!filter_data_ready){usleep(10);}
        filter_data_ready = false;
        
        pthread_mutex_lock(&EKF_mutex);
        MatrixXd q = tello->get_q_data_for_ekf();

        Matrix3d R_foot_right = tello->controller->get_foot_orientation_wrt_body(q.row(0));
        Matrix3d R_foot_left = tello->controller->get_foot_orientation_wrt_body(q.row(1));

        // EKF calls here
        tello->update_filter_IMU_data(tello->get_imu_data_for_ekf());
        tello->update_filter_contact_data(tello->get_gnd_contact_data_for_ekf());
        tello->update_filter_kinematic_data(tello->get_lfv_hip_data_for_ekf(),R_foot_right,R_foot_left);
        pthread_mutex_unlock(&EKF_mutex);
        //usleep(1000);
        handle_end_of_periodic_task(next,period);
    }

}