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
#include <filesystem>
#include "owl.hpp"

#include "math.h"

#include "cheetah_motor.h"
#include "comms.h"
#include "timer.h"
#include "user_config.h"
#include "utilities.h"
#include "kinematics.h"
#include "vn/sensors.h"
#include "dynamic_robot.h"
#include "SRBM-Ctrl/kinematics.h"
#include "compile_time_tracker.h"
#include "mujoco_main.h"
#include "state_estimator.h"
#include "mocap.h"

#include <pcanfd.h>

namespace fs = std::filesystem;

#define TWOPI 6.28318530718
#define ENCODER_TO_RADIANS ((double)(12.5/32768.0))
#define RADIANS_TO_DEGREES ((double)(180.0/M_PI))
#define KNEE_OFFSET_ENC 503 // 11 degrees

#define LeftLeg_OuterBus
#define LeftLeg_InnerBus
#define RightLeg_OuterBus
#define RightLeg_InnerBus


RoboDesignLab::DynamicRobot* tello;
bool calibrate_IMU_bias = false;
bool ekf_position_initialied = false;
bool tare_efk_pos = false;
Vector3d ekf_position_offset;
bool run_motors_for_balancing = false;
bool apply_balance_torques = false;
bool no_posture_ctrl = false;

bool start_controller_time = false;
bool tare_mocap_pos = false;
Vector3d mocap_offset = Vector3d::Zero();

int udp_data_ready = 0;
char udp_control_packet[UDP_MAXLINE];
	
unsigned int num_CAN_writes_since_reset = 0;

float averageTime = 0;
float maxTime = 0;

FILE *log_file, *f_tx, *f_motion;
std::ifstream motion_in_file;
long long print_idx= 0;

struct timeval st, et;
struct timeval update_st, update_et;

struct sched_param sp = { .sched_priority = 99 };

int update_delay = 700;
int delay_microseconds = 300;
int last_update_delay = 950;

int fsm_state = 0; //0=menu, 1=udp, 2=record, 3=playback

union candata
{
    uint8_t b[8];
    uint64_t i;
};
candata send_idx;

unsigned int pcd1 = PCAN_PCIBUS5;	// PCAN_PCIBUS5	// PCAN_PCIBUS1
unsigned int pcd2 = PCAN_PCIBUS6;	// PCAN_PCIBUS6	// PCAN_PCIBUS2
unsigned int pcd3 = PCAN_PCIBUS7;	// PCAN_PCIBUS7	// PCAN_PCIBUS3
unsigned int pcd4 = PCAN_PCIBUS8;	// PCAN_PCIBUS8	// PCAN_PCIBUS4
unsigned int pcd5 = PCAN_PCIBUS1;	// PCAN_PCIBUS1	// PCAN_PCIBUS5
unsigned int pcd6 = PCAN_PCIBUS2;	// PCAN_PCIBUS2	// PCAN_PCIBUS6

int enable_motors = 0;

uint16_t encoders[10];
int position_initialized[10] = {0,0,0,0,0,0,0,0,0,0};
int all_motors_initialized = 0;
uint16_t encoder_positions[10];
uint16_t encoder_offsets[10]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int motor_directions[10] =      { 1,-1, 1, 1,-1, 1,-1, 1, 1,-1};
								//1, 2, 3, 4, 5, 6, 7, 8, 9, 10
int motor_zeros[10] = {35516,35268,33308,34190-KNEE_OFFSET_ENC,33847+KNEE_OFFSET_ENC,32721,35254,34194,34611-KNEE_OFFSET_ENC,33681+KNEE_OFFSET_ENC}; // offsets handled
int motor_init_config[10] = {35540, 36558, 31813, 38599, 31811, 32767, 36712, 32718, 38436, 33335};
int motor_initialized[10] = {0,0,0,0,0,0,0,0,0,0};
int motor_move_complete[10] = {0,0,0,0,0,0,0,0,0,0};
int motors_in_use = 10;
int mode_selected = 0;
int stop_recording = 0;
int recording_initialized = 0;
int playback_initialized = 0;
int motor_targets[10];

vn::math::vec3f tello_ypr;

double joint_setpoints_deg[10] = {0,0,0,0,0,0,0,0,0,0};

double gain_adjustment = 0;
int task_gain_adjustment = 0;
double ANKLE_COMP = 0;
int x_offset = 0;
int z_offset = 0;
double motor_kp = 50;
double motor_kd = 50;
double playback_kp = 4.0;

pthread_mutex_t mutex_CAN_recv = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t EKF_mutex = PTHREAD_MUTEX_INITIALIZER;
int can_data_ready_to_save = 1;
int disabled = 1;

std::ofstream *motion_log;
std::ofstream *jacobian_debug;
std::ifstream *motion_log_in;
// std::ofstream output_file("tello_time_log.txt", std::ios::out | std::ios::trunc);

extern bool enableScheduled;
extern bool disableScheduled;
extern bool zeroScheduled;

int simulation_mode = 0;
bool walking = false;
extern bool filter_data_ready;
double t_program_start;

double dx_prev = 0;
double dx;
VectorXd dx_vec = VectorXd(100);
double dy_prev = 0;
double dy;
VectorXd dy_vec = VectorXd(100);
double dz_prev = 0;
double dz;
VectorXd dz_vec = VectorXd(100);
double dx_filtered;
double dy_filtered;
double dz_filtered;
Vector3d pc = Vector3d(0,0,0);
Vector3d dpc = Vector3d(0,0,0);
Vector3d EA;
Vector3d dEA;
VectorXd jointFFTorque = VectorXd::Zero(10);
bool use_filter_pc = false;

int sockfd_tx;
char hmi_tx_buffer[100];
struct sockaddr_in servaddr_tx;

struct termios originalSettings;

extern double screen_recording;
extern double usbcam_recording;

extern bool auto_mode;
extern MatrixXd lfv_dsp_start;

extern bool log_data_ready;
extern std::string log_folder;
extern VectorXd x_out, u_out, q_out, qd_out,full_tau_out, tau_out, tau_ext_out, lfv_out, lfdv_out,lfv_comm_out,lfdv_comm_out, t_n_FSM_out, impulse_out, meas_grf_out, xDCM_out;
extern double last_log_time;
extern Human_dyn_data hdd_out;
extern Traj_planner_dyn_data tpdd_out;

extern OWL::Context owl;
extern Vector3d CoM_pos;
extern Vector3d CoM_rpy;
extern Vector3d CoM_vel;
extern VectorXd CoM_quat;
extern ctrlData cd_shared;
VisualizationData vizData;

void init_6dof_test();
void signal_callback_handler(int signum);

void handle_UDP_Commands(){
	if(udp_data_ready)
	{
		udp_data_ready = 0;
		uint16_t pos_cmds[10];
		int idx = 0;
		for(int i=0;i<20;i+=2){
			pos_cmds[idx] = ( (udp_control_packet[i+1]<<8)&0xFF00 | (udp_control_packet[i+2])&0x00FF );
			idx++;
		}
		for(int i=0;i<10;i++){
			uint16_t pos_set = (uint16_t)((int)encoder_offsets[i] + (int)(24*((int)pos_cmds[i]-180))*motor_directions[i]);
			tello->motors[i]->setPos(pos_set);
		}
		enable_motors = udp_control_packet[0];
		if(enable_motors){
			for(int i=0;i<10;i++){
				tello->motors[i]->enableMotor();
			}
		}
		else{
			for(int i=0;i<10;i++){
				tello->motors[i]->disableMotor();
			}
		}
	}
}
void handle_Motion_Recording(){

	if(!recording_initialized){
		printf("Recording...\n");
		motion_log = new std::ofstream(MOTION_LOG_NAME, std::ios::trunc);
		if (!motion_log->is_open()) {
			std::cerr << "Error: unable to open file '" << MOTION_LOG_NAME << "' for writing." << std::endl;
			return;
		}
		
		recording_initialized = 1;
	}
	// record here:
	for(int i=0;i<10;i++){
		*motion_log << encoders[i] << " ";
		motion_log->flush();
	}
	*motion_log << std::endl;

}
void handle_Motion_Playback(){
	if(!playback_initialized){
		printf("Playing Motion...\n");
		
		motion_log_in = new std::ifstream(MOTION_LOG_NAME);
		if (!motion_log_in->is_open()) {
			std::cerr << "Error: unable to open file '" << MOTION_LOG_NAME << "'." << std::endl;
			return;
		}
		scheduleEnable();
		playback_initialized = 1;
	}
	std::string line;
	std::getline(*motion_log_in, line);
	std::vector<int> values;
    std::stringstream ss(line);
    int value;
    while (ss >> value) {
      values.push_back(value);
    }
    if (values.size() != 10) {
	  fsm_state = 0;
	  printf("End of recording");
	  playback_initialized = 0;
    }
	else{
		int idx = 0;
		for (int i : values) {
			//std::cout << i << " " << std::flush;
			tello->motors[idx++]->setPos((uint16_t)i);
		}
		if(motor_kp < playback_kp){
			motor_kp = motor_kp+1;
			tello->set_kp_kd_all_motors(motor_kp,motor_kd);
		}
	}
}

void handle_motor_init(){
	int init_sum = 0;
	for(int i=0;i<10;i++){
		if(fabs(encoder_positions[i] - motor_zeros[i])  < 50) 
		{
			motor_initialized[i] = 1;
		}
		else
		{
			if(encoder_positions[i] < motor_zeros[i])
			{
				tello->motors[i]->setPos(encoder_positions[i]+10);
			}
			else
			{
				tello->motors[i]->setPos(encoder_positions[i]-10);
			}

		}
		init_sum += motor_initialized[i];
	}
	if(init_sum == 10){
		fsm_state = 0;
		tello->set_kp_kd_all_motors(200,100);
	}
}

int init_sum = 0;

void moveMotors(int* positions){
	for(int i=0;i<10;i++){
		if(init_sum < 10){
			if(fabs(encoder_positions[i] - positions[i])  < 50) 
			{
				motor_move_complete[i] = 1;
			}
			else
			{
				if(encoder_positions[i] < positions[i]){
					tello->motors[i]->setPos(encoder_positions[i]+10);
				}
				else{
					tello->motors[i]->setPos(encoder_positions[i]-10);
				}
			}
			init_sum = 0;
			for(int x=0;x<10;x++){
				init_sum += motor_move_complete[x];
			}
		}
		else{
			tello->motors[i]->setPos(positions[i]);
		}
	}
	if(init_sum == 10){
		//fsm_state = 0;
		//set_kp_kd_all(1200,600);
		//for(int x=0; x<10; x++) motor_move_complete[x] = 0;
	}
}

Eigen::Matrix<double,5,1> jointsL, jointsR;
Eigen::VectorXd motorsL, motorsR;
Eigen::VectorXd joint_pos_desired(10);
void updateJointPositions()
{
	jointsL(0) = 0.0		*DEGREES_TO_RADIANS;
	jointsL(1) = 0.0		*DEGREES_TO_RADIANS;
	jointsL(2) = 0.0		*DEGREES_TO_RADIANS;
	jointsL(3) = 25.0		*DEGREES_TO_RADIANS; // must be above 11
	jointsL(4) = 0.0		*DEGREES_TO_RADIANS; 
	motorsL = fcn_ik_q_2_p(jointsL);

	jointsR(0) = 0.0		*DEGREES_TO_RADIANS;
	jointsR(1) = 0.0		*DEGREES_TO_RADIANS;
	jointsR(2) = 0.0		*DEGREES_TO_RADIANS;
	jointsR(3) = 25.0		*DEGREES_TO_RADIANS; // must be above 11
	jointsR(4) = 0.0		*DEGREES_TO_RADIANS; 
	motorsR = fcn_ik_q_2_p(jointsR);

	joint_pos_desired << jointsL, jointsR;
	VectorXd motors = ik_joints_to_motors(joint_pos_desired);

	motor_targets[0] = tello->motor_pos_model_to_real(0, motors(0));
	motor_targets[1] = tello->motor_pos_model_to_real(1, motors(1));
	motor_targets[2] = tello->motor_pos_model_to_real(2, motors(2));
	motor_targets[3] = tello->motor_pos_model_to_real(3, motors(3));
	motor_targets[4] = tello->motor_pos_model_to_real(4, motors(4));
	motor_targets[5] = tello->motor_pos_model_to_real(5, motors(5));
	motor_targets[6] = tello->motor_pos_model_to_real(6, motors(6));
	motor_targets[7] = tello->motor_pos_model_to_real(7, motors(7));
	motor_targets[8] = tello->motor_pos_model_to_real(8, motors(8));
	motor_targets[9] = tello->motor_pos_model_to_real(9, motors(9));
}
int sample_index = 0;
VectorXd pitch_deg_buffer(200);
double x_off_prev = 0;

bool move_up_down = false;
int force_ctrl = 0;
double delta_task = 0.02;
double h_offset = 0;
RoboDesignLab::TaskPDConfig task_pd_config;
void run_tello_pd()
{
	pthread_mutex_lock(&mutex_CAN_recv);
	double joint_kp = 12;
	double joint_kd = 0.6;
	VectorXd kp_vec_joint = VectorXd::Ones(10)*(joint_kp+gain_adjustment);
	VectorXd kd_vec_joint = VectorXd::Ones(10)*joint_kd;

	kp_vec_joint(0) = joint_kp + gain_adjustment/5.0;
	kp_vec_joint(1) = joint_kp + gain_adjustment/5.0;
	kp_vec_joint(2) = joint_kp + gain_adjustment;
	kp_vec_joint(3) = joint_kp + gain_adjustment;
	kp_vec_joint(4) = joint_kp + gain_adjustment;

	kp_vec_joint(5) = joint_kp + gain_adjustment/5.0;
	kp_vec_joint(6) = joint_kp + gain_adjustment/2.0;
	kp_vec_joint(7) = joint_kp + gain_adjustment;
	kp_vec_joint(8) = joint_kp + gain_adjustment;
	kp_vec_joint(9) = joint_kp + gain_adjustment;
	


	double motor_kp = 0;
	double motor_kd = 500;
	VectorXd kp_vec_motor = VectorXd::Ones(10)*motor_kp;
	VectorXd kd_vec_motor = VectorXd::Ones(10)*motor_kd;

	VectorXd vel_desired = VectorXd::Zero(12);

	if(move_up_down){
			h_offset+=delta_task;
	}
	if(h_offset > 70){
		delta_task = -0.02;
	}
	if(h_offset < 0){
		delta_task = 0.02;
	}
	Vector3d target(0, 0.01, -0.58+0.088);

	double foot_len_half = 0.060;
	double pitch_degrees = tello_ypr[1];

	double x_off = 0.000;

	
	Vector3d target_front_left(foot_len_half+target(0)+x_off, target(1), target(2)-(pitch_degrees/3500.0));
	Vector3d target_back_left(-foot_len_half+target(0)+x_off, target(1), target(2)+(pitch_degrees/3500.0));
	Vector3d target_front_right(foot_len_half+target(0)+x_off, -target(1), target(2)-(pitch_degrees/3500.0));
	Vector3d target_back_right(-foot_len_half+target(0)+x_off, -target(1), target(2)+(pitch_degrees/3500.0));

	VectorXd pos_desired(12);
	pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

	double task_kp = 0;
	double task_kd = 0;
	VectorXd kp_vec_task = VectorXd::Ones(3)*task_kp;
	VectorXd kd_vec_task = VectorXd::Ones(3)*task_kd;

	// Set up configuration struct for Task Space Controller
	task_pd_config.task_ff_force = VectorXd::Zero(12);
	task_pd_config.task_pos_desired = pos_desired;
	task_pd_config.task_vel_desired = vel_desired;
	task_pd_config.setTaskKp(kp_vec_task);
	task_pd_config.setTaskKd(kd_vec_task);
	task_pd_config.setJointKp(kp_vec_joint);
	task_pd_config.setJointKd(kd_vec_joint);
	task_pd_config.motor_kp = kp_vec_motor;
	task_pd_config.motor_kd = kd_vec_motor;

	// ff force must be negative, make it about 1000

	// if(tello->_GRFs.left_front > 0.3){
	// 	//printf("LEFT CONTACT\n");
	// 	task_pd_config.task_kp.topRows(6).setZero();
	// 	task_pd_config.task_kd.topRows(6).setZero();
	// 	task_pd_config.joint_kp.topRows(6).setZero();
	// 	task_pd_config.joint_kd.topRows(6).setZero();

	// 	task_pd_config.task_ff_force(2) = -200;
	// 	task_pd_config.task_ff_force(5) = -200;

	// }
	// if(tello->_GRFs.right_front > 0.3){
	// 	//printf("RIGHT CONTACT\n");
	// 	task_pd_config.task_kp.bottomRows(6).setZero();
	// 	task_pd_config.task_kd.bottomRows(6).setZero();
	// 	task_pd_config.joint_kp.bottomRows(6).setZero();
	// 	task_pd_config.joint_kd.bottomRows(6).setZero();

	// 	task_pd_config.task_ff_force(8) = -200;
	// 	task_pd_config.task_ff_force(11) = -200;

	// }

	
	tello->taskPD(task_pd_config);

	pthread_mutex_unlock(&mutex_CAN_recv);
}

void run_tello_pd_DEMO()
{
	pthread_mutex_lock(&mutex_CAN_recv);
	double joint_kp = 0.8;
	double joint_kd = 0.05;
	VectorXd kp_vec_joint = VectorXd::Ones(10)*(joint_kp);
	VectorXd kd_vec_joint = VectorXd::Ones(10)*joint_kd;

	MatrixXd kp_mat_joint = kp_vec_joint.asDiagonal();
	MatrixXd kd_mat_joint = kd_vec_joint.asDiagonal();

	double motor_kp = 0;
	double motor_kd = 500;
	VectorXd kp_vec_motor = VectorXd::Ones(10)*motor_kp;
	VectorXd kd_vec_motor = VectorXd::Ones(10)*motor_kd;

	VectorXd vel_desired = VectorXd::Zero(12);

	if(move_up_down){
			h_offset+=delta_task;
	}
	if(h_offset > 70){
		delta_task = -0.02;
	}
	if(h_offset < 0){
		delta_task = 0.02;
	}
	Vector3d target(0, 0, -0.460);

	double foot_len_half = 0.060;
	Vector3d target_front_left(foot_len_half+target(0), target(1), target(2));
	Vector3d target_back_left(-foot_len_half+target(0), target(1), target(2));
	Vector3d target_front_right(foot_len_half+target(0), target(1), target(2));
	Vector3d target_back_right(-foot_len_half+target(0), target(1), target(2));

	VectorXd pos_desired(12);
	pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

	double task_kp = 200;
	double task_kd = 0;
	VectorXd kp_vec_task = VectorXd::Ones(12)*(task_kp+gain_adjustment);
	VectorXd kd_vec_task = VectorXd::Ones(12)*task_kd;
	double z_gain = (200+gain_adjustment);
	double x_gain = 8;
	double y_gain = 8;
	kp_vec_task(0) = x_gain;
	kp_vec_task(1) = y_gain;
	kp_vec_task(2) = z_gain;
	kp_vec_task(3) = x_gain;
	kp_vec_task(4) = y_gain;
	kp_vec_task(5) = z_gain;
	kp_vec_task(6) = x_gain;
	kp_vec_task(7) = y_gain;
	kp_vec_task(8) = z_gain;
	kp_vec_task(9) = x_gain;
	kp_vec_task(10) = y_gain;
	kp_vec_task(11) = z_gain;
	MatrixXd kp_mat_task = kp_vec_task.asDiagonal();
	MatrixXd kd_mat_task = kd_vec_task.asDiagonal();

	// Set up configuration struct for Task Space Controller
	task_pd_config.task_ff_force = VectorXd::Zero(12);
	task_pd_config.task_pos_desired = pos_desired;
	task_pd_config.task_vel_desired = vel_desired;
	task_pd_config.task_kp = kp_mat_task;
	task_pd_config.task_kd = kd_mat_task;
	task_pd_config.joint_kp = kp_mat_joint;
	task_pd_config.joint_kd = kd_mat_joint;
	task_pd_config.motor_kp = kp_vec_motor;
	task_pd_config.motor_kd = kd_vec_motor;

	tello->taskPD(task_pd_config);

	pthread_mutex_unlock(&mutex_CAN_recv);
}

void* hw_logging( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	printf('g',"Logging task running.\n");

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    usleep(1000000);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(true)
    {
        handle_start_of_periodic_task(next);
        while(!log_data_ready) usleep(50);
        log_data_ready = false;

        if(t_n_FSM_out(0) != last_log_time){
            last_log_time = t_n_FSM_out(0);
            // logging:
            dash_utils::writeVectorToCsv(x_out,"x.csv");
            dash_utils::writeVectorToCsv(u_out,"u.csv");
            dash_utils::writeVectorToCsv(tau_out,"tau.csv");
            // dash_utils::writeVectorToCsv(full_tau_out,"full_sw_st_tau.csv");
            dash_utils::writeVectorToCsv(tau_ext_out,"tau_ext.csv");

            dash_utils::writeVectorToCsv(q_out,"q.csv");
            dash_utils::writeVectorToCsv(qd_out,"qd.csv");

            dash_utils::writeVectorToCsv(lfv_out,"lfv.csv");
            dash_utils::writeVectorToCsv(lfdv_out,"lfdv.csv");

            dash_utils::writeVectorToCsv(lfv_comm_out,"lfv_comm.csv");
            dash_utils::writeVectorToCsv(lfdv_comm_out,"lfdv_comm.csv");

            dash_utils::writeVectorToCsv(t_n_FSM_out,"t_and_FSM.csv");

            dash_utils::writeHumanDynDataToCsv(hdd_out,"human_dyn_data.csv");
            dash_utils::writeTrajPlannerDataToCsv(tpdd_out,"traj_planner_dyn_data.csv");

            // dash_utils::writeVectorToCsv(impulse_out,"external_forces.csv");

            // dash_utils::writeVectorToCsv(meas_grf_out,"meas_grf_out.csv");

            // dash_utils::writeVectorToCsv(xDCM_out,"xDCM.csv");
            
        }
        // end logging
        usleep(50);
		// handle_end_of_periodic_task(next,period);
	}
    return  0;
}

void balance_pd(MatrixXd lfv_hip)
{
	RoboDesignLab::TaskPDConfig swing_pd_config, posture_pd_config;

	Vector3d target_front_left = tello->controller->get_lfv_comm_hip().row(2);
	Vector3d target_back_left = tello->controller->get_lfv_comm_hip().row(3);
	Vector3d target_front_right = tello->controller->get_lfv_comm_hip().row(0);
	Vector3d target_back_right = tello->controller->get_lfv_comm_hip().row(1);

	Vector3d target_front_left_vel = tello->controller->get_lfdv_comm_hip().row(2);
	Vector3d target_back_left_vel = tello->controller->get_lfdv_comm_hip().row(3);
	Vector3d target_front_right_vel = tello->controller->get_lfdv_comm_hip().row(0);
	Vector3d target_back_right_vel = tello->controller->get_lfdv_comm_hip().row(1);

	// Vector3d target_front_left_accel = tello->controller->get_lfddv_comm_hip().row(2);
	// Vector3d target_back_left_accel = tello->controller->get_lfddv_comm_hip().row(3);
	// Vector3d target_front_right_accel = tello->controller->get_lfddv_comm_hip().row(0);
	// Vector3d target_back_right_accel = tello->controller->get_lfddv_comm_hip().row(1);

	VectorXd vel_desired(12);
	vel_desired  << 0,0,0,0,0,0,0,0,0,0,0,0;//target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel;

	VectorXd pos_desired(12);
	pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

	VectorXd leg_inertia(5);
	leg_inertia <<  0.0111, // straight leg (hip + thigh + shin + foot) inertia seen by hip yaw
					0.0926, // straight leg (thigh + shin + foot) inertia seen by hip roll
					0.0913, // straight leg (thigh + shin + foot) inertia seen by hip pitch
					0.0174, // straight leg (shin + foot) inertia seen by knee
					0.0030; // foot inertia seen by ankle
	double inertia_accel_gain = 0.0;

	VectorXd swing_leg_torques = VectorXd::Zero(10);

	swing_pd_config.use_single_jacoian = false;
	swing_pd_config.side = BOTH_LEGS;

	swing_pd_config.ignore_joint_velocity = true;
	swing_pd_config.task_ff_force = VectorXd::Zero(12);
	swing_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
	swing_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
	swing_pd_config.setTaskKp(0,0,0);
	swing_pd_config.setTaskKd(0,0,0);
	swing_pd_config.setJointKa(0);
	swing_pd_config.setFFAccel(Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero(),Vector3d::Zero());

	VectorXd kp_vec_joint_swing(10);
	VectorXd kd_vec_joint_swing(10);
	kp_vec_joint_swing << 64, 500, 1000,1000,500, 64, 500, 1000,1000,500;
	kd_vec_joint_swing <<  2, 2, 2,2,2,  2, 2, 2,2,2;

	swing_pd_config.setJointKp(kp_vec_joint_swing);
	swing_pd_config.setJointKd(kd_vec_joint_swing);
	swing_pd_config.motor_kp = VectorXd::Zero(10);
	swing_pd_config.motor_kd = VectorXd::Zero(10);
	swing_leg_torques = tello->taskPD2(swing_pd_config);

	posture_pd_config = swing_pd_config;
	posture_pd_config.ignore_joint_velocity = true;
	posture_pd_config.side = BOTH_LEGS;
 
	posture_pd_config.task_ff_force = VectorXd::Zero(12);
	posture_pd_config.setTaskPosDesired(target_front_left, target_back_left, target_front_right, target_back_right);
	posture_pd_config.setTaskVelDesired(target_front_left_vel, target_back_left_vel, target_front_right_vel, target_back_right_vel);
	posture_pd_config.setTaskKp(0,0,0);
	posture_pd_config.setTaskKd(0,0,0);
	posture_pd_config.setTaskKa(0,0,0);
	VectorXd kp_vec_joint_posture(10);
	VectorXd kd_vec_joint_posture(10);
	kp_vec_joint_posture << 32, 64, 0,0,0, 32, 64, 0,0,0;
	kd_vec_joint_posture <<  0, 0, 0,0,0,  0, 0, 0,0,0;
	posture_pd_config.setJointKp(kp_vec_joint_posture);
	posture_pd_config.setJointKd(kd_vec_joint_posture);

	VectorXd posture_ctrl_torques = tello->taskPD2(posture_pd_config);
	
	// END TASK PD CODE ======================================+++++++++++++++++
	VectorXd tau_LR(10);
	tau_LR = jointFFTorque + posture_ctrl_torques;

	// cout << tau_LR.transpose() << endl;
	VectorXd torques_left  = tello->swing_stance_mux(tau_LR.head(5), swing_leg_torques.head(5),
														0.000,tello->controller->get_isSwingToStanceRight(), 
														tello->controller->get_time()-tello->controller->get_transitionStartRight(), 
														0);
	VectorXd torques_right = tello->swing_stance_mux(tau_LR.tail(5), swing_leg_torques.tail(5),
														0.000,tello->controller->get_isSwingToStanceLeft(),
														tello->controller->get_time()-tello->controller->get_transitionStartLeft(), 
														1);
	VectorXd tau_LR_muxed(10);
	tau_LR_muxed << torques_left,torques_right;


	// cout << tau_LR_muxed.transpose() << endl;


	double joint_kp = 8.0;
	double joint_kd = 0.00;
	VectorXd kp_vec_joint = VectorXd::Ones(10)*(joint_kp+gain_adjustment);
	VectorXd kd_vec_joint = VectorXd::Ones(10)*joint_kd;

	kp_vec_joint(0) = joint_kp + gain_adjustment/5.0;
	kp_vec_joint(1) = joint_kp + gain_adjustment/5.0;
	kp_vec_joint(2) = joint_kp + gain_adjustment;
	kp_vec_joint(3) = joint_kp + gain_adjustment;
	kp_vec_joint(4) = joint_kp + gain_adjustment;

	kp_vec_joint(5) = joint_kp + gain_adjustment/5.0;
	kp_vec_joint(6) = joint_kp + gain_adjustment/2.0;
	kp_vec_joint(7) = joint_kp + gain_adjustment;
	kp_vec_joint(8) = joint_kp + gain_adjustment;
	kp_vec_joint(9) = joint_kp + gain_adjustment;
	
	double motor_kp = 0;
	double motor_kd = 400;
	VectorXd kp_vec_motor(10);// = VectorXd::Ones(10)*motor_kp;
	kp_vec_motor << 0,0,0,0,0,0,0,0,0,0;

	VectorXd kd_vec_motor = VectorXd::Ones(10)*motor_kd;
	if(tello->controller->get_FSM()==1)
	{
		kd_vec_motor << 400,400,400,400,400,300,300,300,300,300;
	}
	if(tello->controller->get_FSM()==-1)
	{
		kd_vec_motor << 300,300,300,300,300,400,400,400,400,400;
	}

	vel_desired = VectorXd::Zero(12);

	// Vector3d target_front_left = lfv_hip.row(2);
	// Vector3d target_back_left = lfv_hip.row(3);
	// Vector3d target_front_right = lfv_hip.row(0);
	// Vector3d target_back_right = lfv_hip.row(1);

	// if(!no_posture_ctrl)
	// {
	// 	target_front_left(2) = -0.58+0.088;
	// 	target_back_left(2) = -0.58+0.088;
	// 	target_front_right(2) = -0.58+0.088;
	// 	target_back_right(2) = -0.58+0.088;
	// }
	// double foot_len_half = 0.060;
	// Vector3d target_front_left(foot_len_half+target(0), target(1), target(2));
	// Vector3d target_back_left(-foot_len_half+target(0), target(1), target(2));
	// Vector3d target_front_right(foot_len_half+target(0), target(1), target(2));
	// Vector3d target_back_right(-foot_len_half+target(0), target(1), target(2));

	// VectorXd pos_desired(12);
	// pos_desired << target_front_left, target_back_left, target_front_right, target_back_right;

	double task_kp = 0;
	double task_kd = 0;
	VectorXd kp_vec_task = VectorXd::Ones(3)*task_kp;
	VectorXd kd_vec_task = VectorXd::Ones(3)*task_kd;

	kp_vec_joint << 0, 0, 0,0,0, 0, 0, 0,0,0;
	kd_vec_joint.setZero();

	// Set up configuration struct for Task Space Controller
	task_pd_config.task_ff_force = VectorXd::Zero(12);
	task_pd_config.task_pos_desired = pos_desired;
	task_pd_config.task_vel_desired = vel_desired;
	task_pd_config.setTaskKp(kp_vec_task);
	task_pd_config.setTaskKd(kd_vec_task);
	task_pd_config.setJointKp(kp_vec_joint);
	task_pd_config.setJointKd(kd_vec_joint);
	task_pd_config.motor_kp = kp_vec_motor;
	task_pd_config.motor_kd = kd_vec_motor;
	task_pd_config.joint_ff_torque = tau_LR; // was jointFFTorque
	
	tello->taskPD(task_pd_config);
}

void run_balance_controller()
{
	auto now = std::chrono::system_clock::now();  // Get the current time
    auto micros = std::chrono::time_point_cast<std::chrono::microseconds>(now);  // Round down to nearest microsecond
    auto since_epoch = micros.time_since_epoch();  // Get duration since epoch
    double t = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count() / 1000000.0 - t_program_start;  // Convert to double with resolution of microseconds
	tello->controller->set_time(t);
	

	if(tare_mocap_pos)
	{
		MatrixXd lfv_tare = tello->controller->get_lfv_world();
		VectorXd center = calculateSupportCenter(lfv_tare.row(0),lfv_tare.row(1),lfv_tare.row(2),lfv_tare.row(3));
		mocap_offset = CoM_pos;
	}
	mocap_offset(2) = 0;

	// Set up code for switching to Tello_locomotion_ctrl() here:
	// ctrlData cd_local = cd_shared;
	// TELLO_locomotion_ctrl(cd_local);
	// TODO: set the z forces from foot sensors
	// TODO: add a new config json for hardware mode
	// end code for switching to Tello_locomotion_ctrl().

	// Set pc_curr, dpc_curr, EA_curr, dEA_Curr, q, qd here:
	Vector3d pc_curr = CoM_pos-mocap_offset; // from mocap
	Vector3d dpc_curr = CoM_vel; // from mocap
	Vector3d EA_curr = CoM_rpy; // from mocap
	Vector3d dEA_curr = tello->_gyro; // from IMU

	Vector3d pc_des = tello->controller->get_SRB_state_ref().col(0).head(3);

	Vector3d EA_des = tello->controller->get_SRB_state_ref().col(0).tail(3);
	
	vizData.CoM_pos_measured[0] = pc_curr[0];
	vizData.CoM_pos_measured[1] = pc_curr[1];
	vizData.CoM_pos_measured[2] = pc_curr[2];

	vizData.CoM_pos_desired[0] = pc_des[0];
	vizData.CoM_pos_desired[1] = pc_des[1];
	vizData.CoM_pos_desired[2] = pc_des[2];

	vizData.CoM_rpy_measured[0] = EA_curr[0];
	vizData.CoM_rpy_measured[1] = EA_curr[1];
	vizData.CoM_rpy_measured[2] = EA_curr[2];

	vizData.CoM_rpy_desired[0] = EA_des[0];
	vizData.CoM_rpy_desired[1] = EA_des[1];
	vizData.CoM_rpy_desired[2] = EA_des[2];

	MatrixXd lfv_comm_hip = tello->controller->get_lfv_comm_hip();
	VectorXd task_pos_desired(12);
	task_pos_desired.segment<3>(0) = lfv_comm_hip.row(2);
	task_pos_desired.segment<3>(3) = lfv_comm_hip.row(3);
	task_pos_desired.segment<3>(6) = lfv_comm_hip.row(0);
	task_pos_desired.segment<3>(9) = lfv_comm_hip.row(1);
	
	VectorXd q_desired_eig = tello->task_pos_to_joint_pos(task_pos_desired); // in order left then right

	VectorXd q_measured_eig = tello->getJointPositions();

	for(int i=0;i<10;i++)
	{
		vizData.q_measured[i] = q_measured_eig(i);
		vizData.q_desired[i] = q_desired_eig(i);
	}
	// cout << "Hip Pitch Diff: " << q_measured_eig(2)-q_measured_eig(7) << "    Knee Pitch Diff: " << q_measured_eig(3)-q_measured_eig(8) << endl;

	// cout << "Time: " << t << "     X: " << pc_curr(0) << "     Y: " << pc_curr(1) << "     Z: " << pc_curr(2) << "                \r";
	// cout.flush();

	// cout << "LFV_Z: " << "     RF: " << tello->controller->get_lfv_world()(0,2) << "                \r";
	// cout.flush();

	// cout << "RPY Error: " << "     R: " << tello->_rpy(0)-CoM_rpy(0) << "     P: " << tello->_rpy(1)-CoM_rpy(1) << "     Y: " << 0-CoM_rpy(2) << "                \r";
	// cout.flush();

	// cout << "GRFs: " << tello->_GRFs.left_front << ",  " << tello->_GRFs.right_back << ",  " << tello->_GRFs.right_front << ",  " << tello->_GRFs.right_back << endl;
	// cout << "FSM: " << tello->controller->get_FSM() << "GRFs: " << tello->_GRFs.left_front << ",  " << tello->_GRFs.right_back << ",  " << tello->_GRFs.right_front << ",  " << tello->_GRFs.right_back << endl;

	VectorXd task_velocities = tello->joint_vel_to_task_vel(tello->getJointVelocities(),tello->getJointPositions());

    MatrixXd lfdv_hip(4,3);
    lfdv_hip.row(2) = task_velocities.segment<3>(0);
    lfdv_hip.row(3) = task_velocities.segment<3>(3);
    lfdv_hip.row(0) = task_velocities.segment<3>(6);
    lfdv_hip.row(1) = task_velocities.segment<3>(9);
    tello->controller->set_lfdv_hip(lfdv_hip);

	VectorXd joint_pos_vec = tello->getJointPositions();
	MatrixXd q = MatrixXd::Zero(2,5);
	q.row(0) = joint_pos_vec.tail(5);
	q.row(1) = joint_pos_vec.head(5);
	VectorXd joint_vel_vec = tello->getJointVelocities();
	MatrixXd qd = MatrixXd::Zero(2,5);
	// qd.row(0) = joint_vel_vec.tail(5);
	// qd.row(1) = joint_vel_vec.head(5);
	VectorXd gnd_contacts = VectorXd::Ones(4);

	VectorXd tau = tello->controller->update(pc_curr, dpc_curr, EA_curr, dEA_curr,q ,qd ,t);
	VectorXd tau_LR(10);
    tau_LR << tau.tail(5), tau.head(5);
	if(apply_balance_torques) jointFFTorque = tau_LR;

	if(tare_mocap_pos)
	{
		tello->controller->set_lfv0(tello->controller->get_lfv_world());
		lfv_dsp_start = tello->controller->get_lfv0();
		tare_mocap_pos = false;
	}
	if(start_controller_time)
	{
		init_6dof_test();
		start_controller_time = false;
	}

	// cout << " Tau: " << tau_LR.transpose() << endl;

	// cout << "FSM: " << tello->controller->get_FSM() << "         \r";
	// cout << "RF: " << tello->_GRFs.right_front << "     RB: " << tello->_GRFs.right_back << "     LF: " << tello->_GRFs.left_front << "     LB: " << tello->_GRFs.left_back << "         \r";
	// cout.flush();

	// Matrix3d R_foot_right = tello->controller->get_foot_orientation_wrt_body(q.row(0));
	// Matrix3d R_foot_left = tello->controller->get_foot_orientation_wrt_body(q.row(1));

	// EKF calls here

	// inekf::RobotState filter_state;
	// filter_state = tello->get_filter_state();
	// if(tare_efk_pos)
	// {
	// 	tare_efk_pos = false;
	// 	ekf_position_offset = filter_state.getPosition();
	// 	use_filter_pc = true;
	// }

	// Vector3d estimated_pc(filter_state.getPosition()(0),filter_state.getPosition()(1),filter_state.getPosition()(2));
    // dx = (estimated_pc(0) - dx_prev)/0.001;
    // dx_prev = estimated_pc(0);
    // dy = (estimated_pc(1) - dy_prev)/0.001;
    // dy_prev = estimated_pc(1);
    // dz = (estimated_pc(2) - dz_prev)/0.001;
    // dz_prev = estimated_pc(2);
    // dx_vec.tail(99) = dx_vec.head(99).eval();
    // dx_vec[0] = dx;
    // dy_vec.tail(99) = dy_vec.head(99).eval();
    // dy_vec[0] = dy;
    // dz_vec.tail(99) = dz_vec.head(99).eval();
    // dz_vec[0] = dz;
    // dx_filtered = dash_utils::smoothData(dx_vec,3);
    // dy_filtered = dash_utils::smoothData(dy_vec,3);
    // dz_filtered = dash_utils::smoothData(dz_vec,3);
    // Vector3d estimated_dpc(dx_filtered,dy_filtered,dz_filtered);
	// dpc = estimated_dpc;
	// pc = Vector3d(0,0,0);
	
	// RoboDesignLab::IMU_data imu_data;
    // imu_data.timestamp = t;
    // imu_data.acc = tello->_acc;		// DATA DIRECTIONS AND BIASES VERIFIED
    // imu_data.gyro = tello->_gyro; 	// DATA DIRECTIONS AND BIASES VERIFIED

	//pthread_mutex_lock(&EKF_mutex);
	MatrixXd direct_lfv_hip = MatrixXd(4,3);
	VectorXd task_pos = tello->joint_pos_to_task_pos(tello->getJointPositions());
	direct_lfv_hip.row(0) = task_pos.segment<3>(6);
	direct_lfv_hip.row(1) = task_pos.segment<3>(9);
	direct_lfv_hip.row(2) = task_pos.segment<3>(0);
	direct_lfv_hip.row(3) = task_pos.segment<3>(3); // DATA VERIFIED

	// SET LOG DATA HERE:
	x_out = tello->controller->get_x();
	u_out = tello->controller->get_GRFs();
	tau_out = tello->controller->get_joint_torques();
	tau_ext_out = tello->controller->get_tau_ext();
	q_out = dash_utils::flatten(tello->controller->get_q());
	qd_out = dash_utils::flatten(tello->controller->get_qd());

	lfv_out = dash_utils::flatten(tello->controller->get_lfv_world());
	lfdv_out = dash_utils::flatten(tello->controller->get_lfdv_world());

	lfv_comm_out = dash_utils::flatten(tello->controller->get_lfv_comm_world());
	lfdv_comm_out = dash_utils::flatten(tello->controller->get_lfdv_comm_world());

	t_n_FSM_out = Eigen::Vector2d(tello->controller->get_time(),tello->controller->get_FSM());

	hdd_out = tello->controller->get_human_dyn_data();
	tpdd_out = tello->controller->get_traj_planner_dyn_data();
	log_data_ready = true;

	// END SETTING LOG DATA HERE
	
	// tello->set_imu_data_for_ekf(imu_data);
	// tello->set_gnd_contact_data_for_ekf(gnd_contacts);
	// tello->set_lfv_hip_data_for_ekf(direct_lfv_hip);
	// tello->set_q_data_for_ekf(q);
	// double CoM_z = tello->controller->get_CoM_z(direct_lfv_hip,gnd_contacts,EA_curr); 
	// pc(2) = CoM_z;

	// filter_data_ready = true;
	// tello->update_filter_IMU_data(imu_data);
	// tello->update_filter_contact_data(gnd_contacts);
	// tello->update_filter_kinematic_data(direct_lfv_hip,R_foot_right,R_foot_left);

	// double x = filter_state.getPosition()(0)-ekf_position_offset(0);
	// double y = filter_state.getPosition()(1)-ekf_position_offset(1);
	// double z = filter_state.getPosition()(2)-ekf_position_offset(2);

	// Vector3d rf_error = tello->controller->get_lfv_comm_hip().row(2) - direct_lfv_hip.row(2);
	//cout << direct_lfv_hip << endl;
	//cout << "lfv_comm_hip: " << tello->controller->get_lfv_comm_hip().row(2) << "            \r"; // lfv_comm verified
	//cout << "==================================================================================================" << endl;
	//cout << "X: " << x << "       Y: " << y << "       Z: " << z << "       t: " << t << "             \r";
	// cout << tau_LR.transpose() << "           \r";
	//cout << x << ",\t" << y << ",\t" << z << ",\t" << t << "\n";
	//cout << "R: " << EA_curr(0) << "       P: " << EA_curr(1) << "       Y: " << EA_curr(2) << "       t: " << t << "             \n";
	//cout << "X: " << tello->_acc(0) << "       Y: " << tello->_acc(1) << "       Z: " << tello->_acc(2) << "       t: " << t << "             \n";
	// cout.flush();

	if(run_motors_for_balancing){
		balance_pd(tello->controller->get_lfv_comm_hip());
	}
	else{
		
	}
}

static void* update_1kHz( void * arg )
{
	startTimer();
	
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	int period = std::get<2>(*arg_tuple_ptr);

	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Main Update thread running on core %d, with priority %d\n", core, priority);
    
	tello->motors[0] = new CheetahMotor(0x01,PCAN_PCIBUS5);
	tello->motors[1] = new CheetahMotor(0x02,PCAN_PCIBUS5);
	tello->motors[2] = new CheetahMotor(0x03,PCAN_PCIBUS6);
	tello->motors[3] = new CheetahMotor(0x04,PCAN_PCIBUS5);
	tello->motors[4] = new CheetahMotor(0x05,PCAN_PCIBUS6);
	tello->motors[5] = new CheetahMotor(0x06,PCAN_PCIBUS7);
	tello->motors[6] = new CheetahMotor(0x07,PCAN_PCIBUS8);
	tello->motors[7] = new CheetahMotor(0x08,PCAN_PCIBUS7);
	tello->motors[8] = new CheetahMotor(0x09,PCAN_PCIBUS8);
	tello->motors[9] = new CheetahMotor(0x0A,PCAN_PCIBUS7);

	for(int i=0;i<10;i++)
	{
		tello->motors[i]->setKp(50);
		tello->motors[i]->setKd(50);
		tello->motors[i]->setVel(0);
		tello->motors[i]->setff(0);
		tello->motors[i]->setPos(32768); // (Zero)
		tello->motors[i]->disableMotor();
	}
	usleep(1000);

	int pos_initialized = 0;
	while(pos_initialized < motors_in_use)
	{
		pthread_mutex_lock(&mutex_CAN_recv);
		pos_initialized = 0;
		for(int i=0;i<10;i++)
		{
			pos_initialized+=position_initialized[i];
		}
		pthread_mutex_unlock(&mutex_CAN_recv);
		usleep(1000);
	}
	// all_motors_initialized = 1;
	printf("Motor zero Positions:\n");
	for(int i = 0;i<10;i++){
		printf("Motor %d: %u\n",i+1,encoder_offsets[i]);
	}
	printf('g',"\nAll motors Initialized, Enabling.\n");
	for(int i=0;i<10;i++)
	{
		pthread_mutex_lock(&mutex_CAN_recv);
		tello->motors[i]->setPos(encoder_offsets[i]);
		tello->motors[i]->updateMotor();
		pthread_mutex_unlock(&mutex_CAN_recv);
	}

	usleep(100000); // 100ms

	uint16_t pos_cmds[10];

	// set up UDP transmit here: =======================================================================
	int sockfd;
	char hmi_tx_buffer[100];
	struct sockaddr_in	 servaddr;
	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
	memset(&servaddr, 0, sizeof(servaddr));
	// Filling server information
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(UDP_TRANSMIT_PORT);
	servaddr.sin_addr.s_addr = inet_addr(HMI_IP_ADDRESS);
	// ens UDP setup here: =============================================================================
	// set up UDP_Viz transmit here: =======================================================================
	int sockfd_viz;
	char viz_tx_buffer[200];
	struct sockaddr_in	 servaddr_viz;
	// Creating socket file descriptor
	if ( (sockfd_viz = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
	memset(&servaddr_viz, 0, sizeof(servaddr_viz));
	// Filling server information
	servaddr_viz.sin_family = AF_INET;
	servaddr_viz.sin_port = htons(VIZ_UDP_TRANSMIT_PORT);
	servaddr_viz.sin_addr.s_addr = inet_addr(VIZ_IP_ADDRESS);
	// ens UDP_Viz setup here: =============================================================================
	
	printf('o',"\n\nEnter CTRL+C in terminal to exit.\n\n");

	auto start = std::chrono::high_resolution_clock::now();
	usleep(1000);

	struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

	while(1)
	{
		handle_start_of_periodic_task(next);
		// Write update loop code under this line ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		switch(fsm_state){
			case 0:
				// do nothing
				break;
			case 1:
				tello->set_kp_kd_all_motors(400,200);
				handle_UDP_Commands();
				break;
			case 2:
				pthread_mutex_lock(&mutex_CAN_recv);
				handle_Motion_Recording();
				pthread_mutex_unlock(&mutex_CAN_recv);
				break;
			case 3:
				handle_Motion_Playback();
				break;
			case 4:
				run_tello_pd();
				break;
			case 5:
				run_tello_pd_DEMO();
				break;
			case 6:
				run_balance_controller();
				break;
			default:
				// do nothing
				break;
		}
		//handle UDP transmit here:
		Human_dyn_data hdd = tello->controller->get_human_dyn_data();
		if(tello->controller->is_human_ctrl_enabled())
		{
			
		}
		else
		{
			hdd.FxH_hmi = 0;
			hdd.FyH_hmi = 0;
			hdd.FxH_spring = 0;
		}
		// cout << "Fx: " << hdd.FxH_hmi << "Fy: " << hdd.FyH_hmi << endl;
		// dash_utils::pack_data_to_hmi((uint8_t*)hmi_tx_buffer,hdd);
		// int n = sendto(sockfd, hmi_tx_buffer, 12,MSG_CONFIRM, 
		// 	   (const struct sockaddr *) &servaddr, sizeof(servaddr));

		memcpy(viz_tx_buffer, &vizData, sizeof(vizData));
		int n = sendto(sockfd_viz, viz_tx_buffer, 152,MSG_CONFIRM, 
			   (const struct sockaddr *) &servaddr_viz, sizeof(servaddr_viz));


		pthread_mutex_lock(&mutex_CAN_recv);
		if(disableScheduled){
			tello->disable_all_motors();
			disableScheduled = false;
		}
		else if(enableScheduled){
			tello->enable_all_motors();
			enableScheduled = false;
		}
		else{
			tello->update_all_motors();
		}
		pthread_mutex_unlock(&mutex_CAN_recv);
		// Write update loop code above this line ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		handle_end_of_periodic_task(next,period);
	}

	return NULL;
}

// This callback handles CTRL+C and Segfaults
void signal_callback_handler(int signum){
	owl.done();
    owl.close();
	screen_recording = false;
	usbcam_recording = false;
	system("killall -2 ffmpeg");
	usleep(2000000);
	tcsetattr(STDIN_FILENO, TCSANOW, &originalSettings);
	Human_dyn_data hdd;
	hdd.FxH_spring = 0;
	hdd.FxH_hmi = 0;
	hdd.FyH_hmi = 0;
	dash_utils::pack_data_to_hmi_with_ctrls((uint8_t*)hmi_tx_buffer,hdd,0,0,0);
	int n = sendto(sockfd_tx, hmi_tx_buffer, 24,MSG_CONFIRM, 
			(const struct sockaddr *) &servaddr_tx, sizeof(servaddr_tx));

	fsm_state = 0;
	if (signum == SIGINT) {
        printf('o',"\nProgram disabled by user.\n");
    }
    else if (signum == SIGSEGV) {
        printf('r',"Program crashed due to Seg Fault");
    }
	if(simulation_mode == 0)
	{
		for(int i=0;i<10;i++){
		tello->motors[i]->disableMotor(); 
		}
		printf('o',"Disable command set to all motors.\n");
	}
	// motion_log->close();
	// motion_log_in->close();
	
	if(set_cpu_governor(GOV_POWERSAVE)){
		printf('o',"CPU Governor set to Powersave\n");
	}
	else{
		printf('r',"Failed to set CPU Governor. (does user have write access to file?)\n");
	}

	printf('o',"\n==================== Exiting Tello Software ====================\n\n");
	usleep(1000);
	exit(signum);
}

void init_6dof_test()
{
	string dummy;
	double sim_time;
	char DoF = 'x';
	MatrixXd lfv0 = tello->controller->get_lfv0();
	SRB_Params srb_params = tello->controller->get_SRB_params();
	Traj_planner_dyn_data traj_planner_dyn_data = tello->controller->get_traj_planner_dyn_data();
	Human_params human_params = tello->controller->get_human_params();
	VectorXd x0 = tello->controller->get_x0();
	dash_planner::SRB_6DoF_Test(dummy,sim_time,srb_params,lfv0,DoF,1);

	// auto_mode = true;
	// printf("Walking Selected\n\n");
	// // Option 2: Walking using LIP angular momentum regulation about contact point
	// // user input (walking speed and step frequency)
	// double des_walking_speed = 0;
	// // double des_walking_step_period = 0.2;
	// // end user input
	// std::string recording_file_name = "Walking";
	// srb_params.planner_type = 1; 
	// // srb_params.T = des_walking_step_period;
	// VectorXd t_traj, v_traj;
	// double t_beg_stepping_time, t_end_stepping_time;
	// dash_planner::SRB_LIP_vel_traj(des_walking_speed,t_traj,v_traj,t_beg_stepping_time,t_end_stepping_time);
	// srb_params.vx_des_t = t_traj;
	// srb_params.vx_des_vx = v_traj;
	// srb_params.t_beg_stepping = 5;
	// srb_params.t_end_stepping = 1e10;
	// sim_time = srb_params.vx_des_t(srb_params.vx_des_t.size()-1);

	// Initialize trajectory planner data
    dash_planner::SRB_Init_Traj_Planner_Data(traj_planner_dyn_data, srb_params, human_params, x0, lfv0);

    tello->controller->set_SRB_params(srb_params);
    tello->controller->set_traj_planner_dyn_data(traj_planner_dyn_data);    
}
	

int main(int argc, char *argv[]) {

    tcgetattr(STDIN_FILENO, &originalSettings);

	rlimit stack_limit;
    stack_limit.rlim_cur = 8 * 1024 * 1024;  // 8MB
    stack_limit.rlim_max = RLIM_INFINITY;    // Set to unlimited maximum size if desired
    if (setrlimit(RLIMIT_STACK, &stack_limit) == -1) {
        std::cerr << "Failed to set stack size limit." << std::endl;
        return 1;
    }

	
    if (argc == 1) {
        simulation_mode = 0;
    } else if (argc == 2) {
        std::string arg1 = argv[1];
        if (arg1 == "--simulation" || arg1 == "-s") {
            simulation_mode = 1;
        }
		else if (arg1 == "--srbm-simulation" || arg1 == "-e") {
            simulation_mode = 2;
        } 
		else if (arg1 == "--animation" || arg1 == "-a") {
            simulation_mode = 3;
        }
		else if (arg1 == "--visualization" || arg1 == "-v") {
            simulation_mode = 4;
        }
		else {
            std::cerr << "Invalid input: " << arg1 << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Too many arguments." << std::endl;
        return 1;
    }

	setvbuf(stdout, NULL, _IONBF, 0); // no buffering on printf, change to _IOLBF for buffering until \n character
	std::cout << std::setprecision(4);
	printf('b',"\n==================== Running Tello Software ====================\n\n");
	printf("Software last compiled on: %s \n",getCompileTime().c_str());
	int minutes_since_compile = (int)(std::chrono::duration_cast<std::chrono::minutes>(minutesSinceLastCompile()).count());
	if(minutes_since_compile > 15){
		printf('o',"It has been %d minutes since this program was last compiled. \n\n", minutes_since_compile);
	}
	else if(minutes_since_compile > 2){
		printf('y',"It has been %d minutes since this program was last compiled. \n\n", minutes_since_compile);
	}
	else{
		printf('g',"It has been %d minutes since this program was last compiled. \n\n", minutes_since_compile);
	}
	assignToCore(ISOLATED_CORE_1_THREAD_1);
	setpriority(PRIO_PROCESS, 0, -20); // Set NICE Priority in case user doesn't have RT Permission

	if(set_cpu_governor(GOV_PERFORMANCE)){
		printf('g',"CPU Governor set to Performance\n");
	}
	else{
		printf('r',"Failed to set CPU Governor. (does user have write access to file?)\n");
	}
	print_cpu_speed(ISOLATED_CORE_1_THREAD_1);

	signal(SIGINT, signal_callback_handler); // Handle CTRL+C input
	signal(SIGSEGV, signal_callback_handler);

    int chrt_err = sched_setscheduler(0, SCHED_FIFO, &sp); // set scheduler to FIFO

    int policy = sched_getscheduler(0);
    if (chrt_err == -1) {
        printf('r',"Your user does not have RealTime Scheduler Permissions.\n");
		setpriority(PRIO_PROCESS, 0, -20); // Set NICE Priority in case user doesn't have RT Permission
    }
	else{
		switch(policy) {
			case SCHED_OTHER: printf('o',"Scheduler in use does not support Real-Time\n"); break;
			case SCHED_RR:   printf('g',"Using Round Robin Scheduler (Real-Time Capable)\n"); break;
			case SCHED_FIFO:  printf('g',"Using FIFO Scheduler(Real-Time Capable)\n"); break;
			default:   printf('r',"Unknown Scheduler...does not support Real-Time\n");
		}
	}

	// PRINT CORE AND PRIORITIES =========================================================
	int core = sched_getcpu();
	printf("Main process assigned to core %d, ",core);
	int prio = getpriority(0,0);
	if(!chrt_err){
		printf('g',"with RT Priority: %d\n\n",sp.sched_priority);
	}
	else{
		printf('o',"with NICE Priority: %d\n\n",prio);
	}

	SIM_START:
	tello = new RoboDesignLab::DynamicRobot();
	if(simulation_mode != 0) tello->isSimulation = true;
	for(int i = 0; i<10; i++){ // not in the constructor becuase I want to change how this works
		tello->motor_zeros[i] = motor_zeros[i];
		tello->motor_directions[i] = motor_directions[i];
	}
	VectorXd dir_vec(10);
	dir_vec <<  motor_directions[0], motor_directions[1], motor_directions[2],
				motor_directions[3], motor_directions[4], motor_directions[5],
				motor_directions[6], motor_directions[7], motor_directions[8],
				motor_directions[9];
    tello->_motor_direction_matrix = dir_vec.asDiagonal();

	// Assign existing kinematics functions for the tello DynamicRobot object to use
	tello->assign_ik_joints_to_motors(ik_joints_to_motors);
	tello->assign_jacobian_joints_to_motors(fcn_Jaco_dq_2_dp);
	tello->assign_jacobian_motors_to_joints(fcn_Jaco_dp_2_dq);
	tello->assign_ik_task_to_joints(tello_leg_IK_biped);
	tello->assign_jacobian_joints_to_task_lf_front(fcn_Jaco_dq_2_dT_front);
	tello->assign_jacobian_joints_to_task_lf_back(fcn_Jaco_dq_2_dT_back);
	tello->assign_fk_motors_to_joints(fk_motors_to_joints);
	tello->assign_fk_joints_to_task(fk_joints_to_task);
	tello->assign_jacobian_accel_task_to_joint(fcn_lf1_Jv_dot);

	// SRB_Params srb_params = tello->controller->get_SRB_params();
	// dash_utils::parse_json_to_srb_params("./tello_files/srb_pd_config_HW.json",srb_params);
	// tello->controller->set_SRB_params(srb_params);
	

	auto now = std::chrono::system_clock::now();  // Get the current time
    auto micros = std::chrono::time_point_cast<std::chrono::microseconds>(now);  // Round down to nearest microsecond
    auto since_epoch = micros.time_since_epoch();  // Get duration since epoch
    t_program_start = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch).count() / 1000000.0;  // Convert to double with resolution of microseconds
	
	if(simulation_mode == 1) // Mujoco Sim Mode
	{
		tello->controller->set_sim_mode(1);
		// SIMULATION MODE (Interfaces with Mujoco instead of real sensors)
		printf('o',"Software running in Simulation Mode.\n\n");
		//printf('o',"Software running in Simulation Mode.\n\
		\r\033[1;38;5;208mIf this is a mistake, run without the \033[1;33m-s 1;38;5;208mflag or comment the following line in platformio.ini:\n\
		\r\033[34mupload_command \033[39m= pio run -t exec -a \"-s\"\n\n");
		tello->addPeriodicTask(&sim_step_task, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_1, (void*)(NULL),"sim_step_task",TASK_CONSTANT_PERIOD, 998);
		tello->addPeriodicTask(&mujoco_Update_1KHz, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_2, (void*)(NULL),"mujoco_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&tello_controller, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"tello_ctrl",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&curve_fitting, SCHED_FIFO, 99, ISOLATED_CORE_3_THREAD_2, (void*)(NULL),"tello_ctrl",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&PS4_Controller, SCHED_FIFO, 90, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"ps4_controller_task",TASK_CONSTANT_PERIOD, 500);
		tello->addPeriodicTask(&rx_UDP, SCHED_FIFO, 99, ISOLATED_CORE_3_THREAD_1, NULL,"rx_UDP",TASK_CONSTANT_DELAY, 100);
		// tello->addPeriodicTask(&Human_Playback, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"human_playback_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&Animate_Log, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"human_playback_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&logging, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"logging_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&screenRecord, SCHED_FIFO, 1, ISOLATED_CORE_4_THREAD_1, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&usbCamRecord, SCHED_FIFO, 0, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&state_estimation, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"EKF_Task",TASK_CONSTANT_PERIOD, 3000);
		// tello->addPeriodicTask(&plotting, SCHED_FIFO, 99, ISOLATED_CORE_3_THREAD_1, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&plot_human_data, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_2, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		while(1){ usleep(1000); }
		return 0;
	}
	else if(simulation_mode == 3) // Log Animation Mode
	{
		tello->controller->set_sim_mode(3);
		// SIMULATION MODE (Interfaces with Mujoco instead of real sensors)
		printf('o',"Software running in Animation Mode.\n\n");
		//printf('o',"Software running in Simulation Mode.\n\
		\r\033[1;38;5;208mIf this is a mistake, run without the \033[1;33m-s 1;38;5;208mflag or comment the following line in platformio.ini:\n\
		\r\033[34mupload_command \033[39m= pio run -t exec -a \"-s\"\n\n");
		tello->addPeriodicTask(&sim_step_task, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_1, (void*)(NULL),"sim_step_task",TASK_CONSTANT_PERIOD, 998);
		tello->addPeriodicTask(&mujoco_Update_1KHz, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_2, (void*)(NULL),"mujoco_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&tello_controller, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"tello_ctrl",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&PS4_Controller, SCHED_FIFO, 90, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"ps4_controller_task",TASK_CONSTANT_PERIOD, 500);
		// tello->addPeriodicTask(&rx_UDP, SCHED_FIFO, 99, ISOLATED_CORE_3_THREAD_1, NULL,"rx_UDP",TASK_CONSTANT_DELAY, 100);
		// tello->addPeriodicTask(&Human_Playback, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"human_playback_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&Animate_Log, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"animate_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&logging, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"logging_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&screenRecord, SCHED_FIFO, 1, ISOLATED_CORE_4_THREAD_1, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&usbCamRecord, SCHED_FIFO, 0, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&state_estimation, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"EKF_Task",TASK_CONSTANT_PERIOD, 3000);
		tello->addPeriodicTask(&plotting, SCHED_FIFO, 99, 6, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&plot_human_data, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_2, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&motion_capture, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"Mocap_Task",TASK_CONSTANT_PERIOD, 1000);
		while(1){ usleep(1000); }
		return 0;
	}
	else if(simulation_mode == 4) // Visualization Mode
	{
		tello->controller->set_sim_mode(3);
		// SIMULATION MODE (Interfaces with Mujoco instead of real sensors)
		printf('o',"Software running in Visualization Mode.\n\n");
		tello->addPeriodicTask(&sim_step_task, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_1, (void*)(NULL),"sim_step_task",TASK_CONSTANT_PERIOD, 998);
		tello->addPeriodicTask(&mujoco_Update_1KHz, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_2, (void*)(NULL),"mujoco_task",TASK_CONSTANT_PERIOD, 10000);
		// tello->addPeriodicTask(&PS4_Controller, SCHED_FIFO, 90, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"ps4_controller_task",TASK_CONSTANT_PERIOD, 500);
		tello->addPeriodicTask(&visualize_robot, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"animate_task",TASK_CONSTANT_PERIOD, 500);
		tello->addPeriodicTask(&screenRecord, SCHED_FIFO, 1, ISOLATED_CORE_4_THREAD_1, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&usbCamRecord, SCHED_FIFO, 0, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&plotting, SCHED_FIFO, 99, 6, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&motion_capture, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"Mocap_Task",TASK_CONSTANT_PERIOD, 1000);
		while(1){ usleep(1000); }
		return 0;
	}
	else if(simulation_mode == 2) // SRB Euler Integration Sim Mode
	{
		tello->controller->set_sim_mode(2);
		// SIMULATION MODE (Interfaces with Mujoco instead of real sensors)
		printf('o',"Software running in SRBM Simulation Mode.\n\n");
		//printf('o',"Software running in Simulation Mode.\n\
		\r\033[1;38;5;208mIf this is a mistake, run without the \033[1;33m-s 1;38;5;208mflag or comment the following line in platformio.ini:\n\
		\r\033[34mupload_command \033[39m= pio run -t exec -a \"-s\"\n\n");
		tello->addPeriodicTask(&sim_step_task, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_1, (void*)(NULL),"sim_step_task",TASK_CONSTANT_PERIOD, 998);
		tello->addPeriodicTask(&mujoco_Update_1KHz, SCHED_FIFO, 98, ISOLATED_CORE_1_THREAD_2, (void*)(NULL),"mujoco_task",TASK_CONSTANT_PERIOD, 2000);
		tello->addPeriodicTask(&tello_controller, SCHED_FIFO, 98, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"tello_ctrl",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&PS4_Controller, SCHED_FIFO, 90, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"ps4_controller_task",TASK_CONSTANT_PERIOD, 500);
		tello->addPeriodicTask(&rx_UDP, SCHED_FIFO, 99, ISOLATED_CORE_3_THREAD_1, NULL,"rx_UDP",TASK_CONSTANT_DELAY, 100);
		tello->addPeriodicTask(&Human_Playback, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"human_playback_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&logging, SCHED_FIFO, 90, ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"logging_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&screenRecord, SCHED_FIFO, 1, ISOLATED_CORE_4_THREAD_1, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		tello->addPeriodicTask(&usbCamRecord, SCHED_FIFO, 0, ISOLATED_CORE_4_THREAD_2, (void*)(NULL),"screen_recording_task",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&state_estimation, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(NULL),"EKF_Task",TASK_CONSTANT_PERIOD, 3000);
		// tello->addPeriodicTask(&plotting, SCHED_FIFO, 99, 6, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		// tello->addPeriodicTask(&plot_human_data, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_2, NULL, "plotting",TASK_CONSTANT_PERIOD, 1000);
		while(1){ usleep(1000); }
		return 0;
	}
	else
	{
		// HARDWARE MODE (Running on actual robot)
	}

	// Initialize Peak Systems CAN adapters
	TPCANStatus s1,s2,s3,s4,s5,s6;
    s1 = CAN_Initialize(pcd1, PCAN_BAUD_1M, 0, 0, 0);
    s2 = CAN_Initialize(pcd2, PCAN_BAUD_1M, 0, 0, 0);
	s3 = CAN_Initialize(pcd3, PCAN_BAUD_1M, 0, 0, 0);
	s4 = CAN_Initialize(pcd4, PCAN_BAUD_1M, 0, 0, 0); 
	s5 = CAN_Initialize(pcd5, PCAN_BAUD_1M, 0, 0, 0);
	s6 = CAN_Initialize(pcd6, PCAN_BAUD_1M, 0, 0, 0);
	std::string channels = "";
	if(!s1)channels+="1, ";
	if(!s2)channels+="2, ";
	if(!s3)channels+="3, ";
	if(!s4)channels+="4, ";
	if(!s5)channels+="5, ";
	if(!s6)channels+="6";

	if(channels.empty()){
		printf('r',"No CAN channels could be opened, did you mean to run in simulation mode? \033[0;38;5;208m(y/n)\n");
		char sim_choice;
		std::cin >> sim_choice;
		if(sim_choice == 'y')
		{
			simulation_mode = 1;
			printf('o',"\n\033[0;38;5;208mTo run simulation mode directly, use the argument \033[1;33m--simulation \033[0;38;5;208mor \033[1;33m-s\n\n");
			//printf('o',"\033[38;5;208mIf running in VS Code, uncomment the following line in platformio.ini:\n\033[32m; upload_command = pio run -t exec -a \"-s\"\n\n");
			goto SIM_START;
		}
		else{
			printf('r',"Check CAN connection and driver and resolve issue before running again.\n");
			return 0;
		}
	}
	
	printf("CAN Channels opened: %s \n",(channels).c_str());

	// Path to the directory
    fs::path directoryPath = "/media/tello/logging_usb/Tello_HW_Logs/";

    // Check if the directory exists
    if (fs::exists(directoryPath)) {
        log_folder = createLogFolder("/media/tello/logging_usb/Tello_HW_Logs/");
		if(log_folder.empty())
		{
			log_folder = createLogFolder("/home/tello/tello_outputs/temp_logs");
		}
    } else {
        log_folder = createLogFolder("/home/tello/tello_outputs/temp_logs");
    }
    dash_utils::setOutputFolder(log_folder);

	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, (void*)(&pcd1),"rx_bus1",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, (void*)(&pcd2),"rx_bus2",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, (void*)(&pcd3),"rx_bus3",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, (void*)(&pcd4),"rx_bus4",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, (void*)(&pcd5),"rx_bus4",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, (void*)(&pcd6),"rx_bus5",TASK_CONSTANT_DELAY, 50);
	// tello->addPeriodicTask(&rx_UDP, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, NULL,"rx_UDP",TASK_CONSTANT_DELAY, 100);
	tello->addPeriodicTask(&IMU_Comms, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_2, NULL, "imu_task", TASK_CONSTANT_DELAY, 1000);
	tello->addPeriodicTask(&update_1kHz, SCHED_FIFO, 99, UPX_ISOLATED_CORE_1_THREAD_2, NULL, "update_task",TASK_CONSTANT_PERIOD, 1000);
	// tello->addPeriodicTask(&BNO055_Comms, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_1, NULL, "bno_imu_task", TASK_CONSTANT_DELAY, 1000);
	// tello->addPeriodicTask(&state_estimation, SCHED_FIFO, 99, UPX_ISOLATED_CORE_2_THREAD_2, (void*)(NULL),"EKF_Task",TASK_CONSTANT_PERIOD, 3000);
	tello->addPeriodicTask(&motion_capture, SCHED_FIFO, 99, UPX_ISOLATED_CORE_3_THREAD_1, (void*)(NULL),"Mocap_Task",TASK_CONSTANT_PERIOD, 1000);
	tello->addPeriodicTask(&hw_logging, SCHED_FIFO, 99, UPX_ISOLATED_CORE_3_THREAD_2, (void*)(NULL),"hw_logging_task",TASK_CONSTANT_PERIOD, 1000);

	usleep(1000);
	
	while(1){
		printf('u',"\n\nTello Software Menu:\n");
		printf("u : Enter UDP Control Mode\n");
		printf("r : Enter Motion Recording Mode\n");
		printf("e : Exit Motion Recording Mode\n");
		printf("p : Enter Motion Playback Mode\n");
		printf("i : Enter Idle Mode\n\n");
		Eigen::Matrix<double,5,1> jointsLeft, jointsRight;
		Eigen::VectorXd motorsLeft, motorsRight;
		int targets[10];

		std::chrono::_V2::system_clock::time_point now1;
		std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::microseconds> micros1;
		std::chrono::microseconds since_epoch1;

		char choice;
		std::cin >> choice;
		switch(choice){
			case 'C':
				calibrate_IMU_bias = true;
				printf("Calibrating IMU\n");
				break;
			case 'D':
				move_up_down = !move_up_down;
				printf("Running Move Up-Down\n");
				break;
			case 'w':
				gain_adjustment+=4.0;
				printf("New Adj. : %f \n", gain_adjustment);
				break;
			case 'q':
				if(gain_adjustment>=4.0){
					gain_adjustment-=4.0;
				}
				
				printf("New Adj. : %f \n", gain_adjustment);
				break;
			case 'm':
				printf("\nMotor Positions:\n");
				for(int i = 0;i<10;i++){
					printf("M(deg) %d: %d ",i+1, (int)(encoder_positions[i]));
				}
				printf("\n");
				break;
			case 'j':
				fsm_state = 4;
				printf("\nTask Space Testing Mode:\n");
				scheduleEnable();

				break;
			case 'B':
				fsm_state = 6;
				tare_mocap_pos = true;
				printf("\nBalance Controller:\n");

				break;
			case 'b':
				no_posture_ctrl = true;
				printf("\nBalance Torques only:\n");

				break;
			case 'M':
				scheduleEnable();
				run_motors_for_balancing = true;
				tare_mocap_pos = true;
				break;
			case 'a':
				fsm_state = 6;
				run_motors_for_balancing = true;
				apply_balance_torques = true;
				no_posture_ctrl = true;
				tare_mocap_pos = true;
				now1 = std::chrono::system_clock::now();  // Get the current time
				micros1 = std::chrono::time_point_cast<std::chrono::microseconds>(now1);  // Round down to nearest microsecond
				since_epoch1 = micros1.time_since_epoch();  // Get duration since epoch
				t_program_start = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch1).count() / 1000000.0;  // Convert to double with resolution of microseconds
				break;
			case 't':
				printf("\nEKF Position Tared:\n");
				tare_efk_pos = true;
				tare_mocap_pos = true;
				break;
			case '6':
				printf("\nStart 6DoF Test:\n");
				now1 = std::chrono::system_clock::now();  // Get the current time
				micros1 = std::chrono::time_point_cast<std::chrono::microseconds>(now1);  // Round down to nearest microsecond
				since_epoch1 = micros1.time_since_epoch();  // Get duration since epoch
				t_program_start = std::chrono::duration_cast<std::chrono::microseconds>(since_epoch1).count() / 1000000.0;  // Convert to double with resolution of microseconds
				start_controller_time = true;
				break;
			case 'R':
				fsm_state = 5;
				printf("\nTask Space Testing Mode:\n");
				//scheduleEnable();
				tello->motors[5]->enableMotor();
				tello->motors[6]->enableMotor();
				tello->motors[7]->enableMotor();
				tello->motors[8]->enableMotor();
				tello->motors[9]->enableMotor();

				break;
			case 'd':
				scheduleDisable();
				printf("\nDisabling\n");
				break;
			case 'u':
				fsm_state = 1;
				printf("\nEntering UDP Mode\n");
				break;
			case 'r':
				fsm_state = 2;
				printf("\nEntering Motion Recording Mode\n");
				scheduleDisable();
				break;
			case 'p':
				fsm_state = 3;
				printf("\nEntering Motion Playback Mode\n");
				scheduleEnable();
				break;
			case 'e':
				if(fsm_state == 2){
					fsm_state = 0;
					printf("\nEnding Recording.\n");
					usleep(10000);
					motion_log->close();
					printf("Recording Written to file.\n");
					recording_initialized = 0;
					printf("\nEntering Idle Mode\n");
					break;
				}
				else if(fsm_state == 3){
					fsm_state = 0;
					printf("\nEnding Playback.\n");
					usleep(10000);
					motion_log_in->close();
					playback_initialized = 0;
					printf("\nEntering Idle Mode\n");
					break;
				}
				else{
					scheduleEnable();
					printf("\nEnabling\n");
				}
				break;
			default:
				fsm_state = 0;
				printf("\nEntering Idle Mode\n");
				scheduleDisable();
				break;
		}
		usleep(1000);
	}

	CAN_Uninitialize(pcd1);
	CAN_Uninitialize(pcd2);
	CAN_Uninitialize(pcd3);
	CAN_Uninitialize(pcd4);
	
	return 0;
}
