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

#include "math.h"

#include "cheetah_motor.h"
#include "comms.h"
#include "timer.h"
#include "user_config.h"
#include "utilities.h"
#include "kinematics.h"
#include "vn/sensors.h"
#include "dynamic_robot.h"

#include <pcanfd.h>

#define TWOPI 6.28318530718
#define ENCODER_TO_RADIANS ((double)(12.5/32768.0))
#define RADIANS_TO_DEGREES ((double)(180.0/M_PI))
#define KNEE_OFFSET_ENC 503

RoboDesignLab::DynamicRobot* tello;

int udp_data_ready = 0;
char udp_control_packet[UDP_MAXLINE];
	
unsigned int num_CAN_writes_since_reset = 0;

float averageTime = 0;
float maxTime = 0;

FILE *log_file, *f_tx, *f_motion;
std::ifstream motion_in_file;

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

unsigned int pcd1 = PCAN_PCIBUS1;
unsigned int pcd2 = PCAN_PCIBUS2;
unsigned int pcd3 = PCAN_PCIBUS3;
unsigned int pcd4 = PCAN_PCIBUS4;

int enable_motors = 0;

uint16_t encoders[10];
int position_initialized[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t encoder_positions[10];
uint16_t encoder_offsets[10]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int motor_directions[10] =      { 1,-1, 1, 1,-1, 1,-1, 1, 1,-1};
								//1, 2, 3, 4, 5, 6, 7, 8, 9, 10
int motor_zeros[10] = {35534,35149,33481,33917-KNEE_OFFSET_ENC,33116+KNEE_OFFSET_ENC,32713,35168,34134,35148-KNEE_OFFSET_ENC,34552+KNEE_OFFSET_ENC}; // offsets handled
int motor_init_config[10] = {35540, 36558, 31813, 38599, 31811, 32767, 36712, 32718, 38436, 33335};
int motor_initialized[10] = {0,0,0,0,0,0,0,0,0,0};
int motor_move_complete[10] = {0,0,0,0,0,0,0,0,0,0};
int motors_in_use = 10;
int mode_selected = 0;
int stop_recording = 0;
int recording_initialized = 0;
int playback_initialized = 0;
int* motor_targets;

vn::math::vec3f tello_ypr;

double joint_setpoints_deg[10] = {0,0,0,0,0,0,0,0,0,0};

int motor_kp = 50;
int motor_kd = 600;
int playback_kp = 1000;

CheetahMotor* motors[10];

pthread_mutex_t mutex_CAN_recv = PTHREAD_MUTEX_INITIALIZER;
int can_data_ready_to_save = 1;
int disabled = 1;

std::ofstream *motion_log;
std::ifstream *motion_log_in;
std::ofstream output_file("tello_time_log.txt", std::ios::out | std::ios::trunc);

extern bool enableScheduled;
extern bool disableScheduled;
extern bool zeroScheduled;

void enable_all(){
	for(int i=0;i<10;i++)
	{
		motors[i]->enableMotor();
	}
}
void disable_all(){
	for(int i=0;i<10;i++)
	{
		motors[i]->disableMotor();
	}
}
void set_kp_kd_all(uint16_t kp, uint16_t kd){
	for(int i=0;i<10;i++)
	{
		if(i == 3 || i == 4 || i == 8 || i == 9){
			motors[i]->setKp(kp*3);
			motors[i]->setKd(kd);
		}
		else{
			motors[i]->setKp(kp);
			motors[i]->setKd(kd);
		}
	}
}
void update_all_motors(){
	for(int i=0;i<10;i++)
	{
		motors[i]->updateMotor();
	}
}

int motor_pos_model_to_real(int id, double joint_position_radians)
{
    return (int)((float)( joint_position_radians)*((float)(motor_directions[id])/ENCODER_TO_RADIANS))+motor_zeros[id];
}

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
			motors[i]->setPos(pos_set);
		}
		enable_motors = udp_control_packet[0];
		if(enable_motors){
			for(int i=0;i<10;i++){
				motors[i]->enableMotor();
			}
		}
		else{
			for(int i=0;i<10;i++){
				motors[i]->disableMotor();
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
			motors[idx++]->setPos((uint16_t)i);
		}
		if(motor_kp < playback_kp){
			motor_kp = motor_kp+1;
			set_kp_kd_all(motor_kp,motor_kd);
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
				motors[i]->setPos(encoder_positions[i]+10);
			}
			else
			{
				motors[i]->setPos(encoder_positions[i]-10);
			}

		}
		init_sum += motor_initialized[i];
	}
	if(init_sum == 10){
		fsm_state = 0;
		set_kp_kd_all(200,100);
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
				if(encoder_positions[i] < positions[i])
				{
					motors[i]->setPos(encoder_positions[i]+10);
				}
				else
				{
					motors[i]->setPos(encoder_positions[i]-10);
				}

			}
			init_sum = 0;
			for(int x=0;x<10;x++){
				init_sum += motor_move_complete[x];
			}

		}
		else
		{
			motors[i]->setPos(positions[i]);
		}
		
	}
	if(init_sum == 10){
		//fsm_state = 0;
		//set_kp_kd_all(1200,600);
		//for(int x=0; x<10; x++) motor_move_complete[x] = 0;
	}
}
double integral = 0;
double last_error = 0;
double kp = 1;
double pid_controller(double error) {
	if(error < 0){
		kp = 1;
	}
	else{
		kp = 1;
	}
    double ki = 0.0;   // Integral gain
    double kd = 0.0;   // Derivative gain
    double dt = 0.1;   // Time step
    double integral = 0;
    double derivative = 0;
    
    
    // Calculate the proportional term
    double proportional = kp * error;
    
    // Calculate the integral term
    integral += error * dt;
	if(integral > 1){
		integral = 1;
	}
	if(integral < -1){
		integral = -1;
	}
    double integral_term = ki * integral;
    
    // Calculate the derivative term
    derivative = (error - last_error) / dt;
    double derivative_term = kd * derivative;
    
    // Calculate the overall effort
    double effort = proportional + integral_term + derivative_term;
    
    // Update the last error for the next iteration
    last_error = error;
    
    return effort;
}

Eigen::Matrix<double,5,1> jointsL, jointsR;
Eigen::VectorXd motorsL, motorsR;
int targets1[10];

void updateJointPositions()
{
	double effort = pid_controller(tello_ypr[1]);
	jointsL(0) = 0.0			*DEGREES_TO_RADIANS;
	jointsL(1) = 0.0			*DEGREES_TO_RADIANS;
	jointsL(2) = -19.0			*DEGREES_TO_RADIANS;
	jointsL(3) = 25.0		*DEGREES_TO_RADIANS; // must be above 11
	jointsL(4) = (-0.55*effort-20)			*DEGREES_TO_RADIANS; 
	motorsL = fcn_ik_q_2_p(jointsL);

	jointsR(0) = 0.0		*DEGREES_TO_RADIANS;
	jointsR(1) = 0.0		*DEGREES_TO_RADIANS;
	jointsR(2) = -19.0		*DEGREES_TO_RADIANS;
	jointsR(3) = 25.0		*DEGREES_TO_RADIANS; // must be above 11
	jointsR(4) = (-0.55*effort-18)		*DEGREES_TO_RADIANS; 
	motorsR = fcn_ik_q_2_p(jointsR);


	targets1[0] = motor_pos_model_to_real(0, motorsL(0));
	targets1[1] = motor_pos_model_to_real(1, motorsL(1));
	targets1[2] = motor_pos_model_to_real(2, motorsL(2));
	targets1[3] = motor_pos_model_to_real(3, motorsL(3));
	targets1[4] = motor_pos_model_to_real(4, motorsL(4));
	targets1[5] = motor_pos_model_to_real(5, motorsR(0));
	targets1[6] = motor_pos_model_to_real(6, motorsR(1));
	targets1[7] = motor_pos_model_to_real(7, motorsR(2));
	targets1[8] = motor_pos_model_to_real(8, motorsR(3));
	targets1[9] = motor_pos_model_to_real(9, motorsR(4));
	motor_targets = targets1;
}


static void* update_1kHz( void * arg )
{
	startTimer();

	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Main Update thread running on core %d, with priority %d\n", core, priority);
    
	motors[0] = new CheetahMotor(0x01,PCAN_PCIBUS1);
	motors[1] = new CheetahMotor(0x02,PCAN_PCIBUS1);
	motors[2] = new CheetahMotor(0x03,PCAN_PCIBUS2);
	motors[3] = new CheetahMotor(0x04,PCAN_PCIBUS1);
	motors[4] = new CheetahMotor(0x05,PCAN_PCIBUS2);
	motors[5] = new CheetahMotor(0x06,PCAN_PCIBUS3);
	motors[6] = new CheetahMotor(0x07,PCAN_PCIBUS4);
	motors[7] = new CheetahMotor(0x08,PCAN_PCIBUS3);
	motors[8] = new CheetahMotor(0x09,PCAN_PCIBUS4);
	motors[9] = new CheetahMotor(0x0A,PCAN_PCIBUS3);

	for(int i=0;i<10;i++)
	{
		motors[i]->setKp(50);
		motors[i]->setKd(1);
		motors[i]->setPos(32768);
		motors[i]->disableMotor();
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
	printf("Motor zero Positions:\n");
	for(int i = 0;i<10;i++){
		printf("Motor %d: %u\n",i+1,encoder_offsets[i]);
	}
	printf("\nAll motors Initialized, Enabling.\n");
	for(int i=0;i<10;i++)
	{
		motors[i]->setPos(encoder_offsets[i]);
		motors[i]->updateMotor();
	}

	usleep(100000); // 100ms

	uint16_t pos_cmds[10];

	printf("\n\nEnter CTRL+C in terminal to exit.\n\n");

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
				set_kp_kd_all(400,200);
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
				updateJointPositions();
				set_kp_kd_all(2000,600);
				moveMotors(motor_targets);
				break;
			default:
				// do nothing
				break;
		}
		if(disableScheduled){
			disable_all();
			disableScheduled = false;
		}
		else if(enableScheduled){
			enable_all();
			enableScheduled = false;
		}
		else{
			update_all_motors();
		}
		// Write update loop code above this line ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		handle_end_of_periodic_task(next);
	}

	return NULL;
}

// This callback handles CTRL+C Input
void signal_callback_handler(int signum){
	fsm_state = 0;
	for(int i=0;i<10;i++){
		// motors[i]->disableMotor();
	}

	//fclose(log_file);
	output_file.close();
	motion_log->close();
	motion_log_in->close();
	if(recording_initialized){
		usleep(1000);
		recording_initialized = 0;
	}
	
	printf("\n\n==================== Exiting Tello Software ====================\n");
	
	if(set_cpu_governor(GOV_POWERSAVE)){
		printf("CPU Governor set to Powersave\n");
	}
	else{
		printf("Failed to set CPU Governor. (does user have write access to file?)\n");
	}
	
	usleep(1000);
	exit(signum);
}
	

int main() {
	setvbuf(stdout, NULL, _IONBF, 0); // no buffering on printf, change to _IOLBF for buffering until \n character
	std::cout << std::setprecision(2);
	printf("\n==================== Running Tello Software ====================\n\n");
	assignToCore(ISOLATED_CORE_1_THREAD_1);
	setpriority(PRIO_PROCESS, 0, 19); // Set NICE Priority in case user doesn't have RT Permission

	if(set_cpu_governor(GOV_PERFORMANCE)){
		printf("CPU Governor set to Performance\n");
	}
	else{
		printf("Failed to set CPU Governor. (does user have write access to file?)\n");
	}
	print_cpu_speed(ISOLATED_CORE_1_THREAD_1);

	signal(SIGINT, signal_callback_handler); // Handle CTRL+C input


    int chrt_err = sched_setscheduler(0, SCHED_FIFO, &sp); // set scheduler to FIFO

    int policy = sched_getscheduler(0);
    if (chrt_err == -1) {
        perror("Your user does not have RealTime Scheduler Permissions.");
		setpriority(PRIO_PROCESS, 0, 19); // Set NICE Priority in case user doesn't have RT Permission
    }
	else{
		switch(policy) {
			case SCHED_OTHER: printf("Scheduler in use does not support Real-Time\n"); break;
			case SCHED_RR:   printf("Using Round Robin Scheduler (Real-Time Capable)\n"); break;
			case SCHED_FIFO:  printf("Using FIFO Scheduler(Real-Time Capable)\n"); break;
			default:   printf("Unknown Scheduler...does not support Real-Time\n");
		}
	}

	// PRINT CORE AND PRIORITIES =========================================================
	int core = sched_getcpu();
	printf("Main process assigned to core %d, ",core);
	int prio = getpriority(0,0);
	if(!chrt_err){
		printf("with RT Priority: %d\n\n",sp.sched_priority);
	}
	else{
		printf("with NICE Priority: %d\n\n",prio);
	}

	tello = new RoboDesignLab::DynamicRobot();

	TPCANStatus s1,s2,s3,s4;
    s1 = CAN_Initialize(pcd1, PCAN_BAUD_1M, 0, 0, 0);
    s2 = CAN_Initialize(pcd2, PCAN_BAUD_1M, 0, 0, 0);
	s3 = CAN_Initialize(pcd3, PCAN_BAUD_1M, 0, 0, 0);
	s4 = CAN_Initialize(pcd4, PCAN_BAUD_1M, 0, 0, 0);
	std::string channels = "";
	if(!s1)channels+="1, ";
	if(!s2)channels+="2, ";
	if(!s3)channels+="3, ";
	if(!s4)channels+="4";
	
	printf("CAN Channels opened: %s \n",(channels).c_str());

	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(&pcd1),"rx_bus1",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(&pcd2),"rx_bus2",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(&pcd3),"rx_bus3",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&rx_CAN, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, (void*)(&pcd4),"rx_bus4",TASK_CONSTANT_DELAY, 50);
	tello->addPeriodicTask(&IMU_Comms, SCHED_FIFO, 99, ISOLATED_CORE_2_THREAD_1, NULL, "imu_task", TASK_CONSTANT_DELAY, 1000);
	tello->addPeriodicTask(&update_1kHz, SCHED_FIFO, 99, ISOLATED_CORE_1_THREAD_2, NULL, "update_task",TASK_CONSTANT_PERIOD, 1000);
	
	usleep(1000);
	
	while(1){
		printf("\n\nTello Software Menu:\n");
		printf("u : Enter UDP Control Mode\n");
		printf("r : Enter Motion Recording Mode\n");
		printf("e : Exit Motion Recording Mode\n");
		printf("p : Enter Motion Playback Mode\n");
		printf("i : Enter Idle Mode (or press any other unused key)\n\n");
		Eigen::Matrix<double,5,1> jointsLeft, jointsRight;
		Eigen::VectorXd motorsLeft, motorsRight;
		int targets[10];
		char choice;
		std::cin >> choice;
		switch(choice){
			case 'm':
				printf("\nMotor Positions:\n");
				for(int i = 0;i<10;i++){
					printf("M(deg) %d: %f ",i+1, (float)(encoder_positions[i]-motor_zeros[i])*ENCODER_TO_RADIANS*RADIANS_TO_DEGREES );
				}
				printf("\n");
				break;
			case 'J':
				printf("\nJoint Inputs:\n");
				jointsLeft(0) = 0.0			*DEGREES_TO_RADIANS;
				jointsLeft(1) = 0.0			*DEGREES_TO_RADIANS;
				jointsLeft(2) = 0.0			*DEGREES_TO_RADIANS;
				jointsLeft(3) = 15.0		*DEGREES_TO_RADIANS; // must be above 11
				jointsLeft(4) = 0.0			*DEGREES_TO_RADIANS; 
				motorsLeft = fcn_ik_q_2_p(jointsLeft);

				jointsRight(0) = 0.0		*DEGREES_TO_RADIANS;
				jointsRight(1) = 0.0		*DEGREES_TO_RADIANS;
				jointsRight(2) = 0.0		*DEGREES_TO_RADIANS;
				jointsRight(3) = 15.0		*DEGREES_TO_RADIANS; // must be above 11
				jointsRight(4) = 0.0		*DEGREES_TO_RADIANS; 
				motorsRight = fcn_ik_q_2_p(jointsRight);


				targets[0] = motor_pos_model_to_real(0, motorsLeft(0));
				targets[1] = motor_pos_model_to_real(1, motorsLeft(1));
				targets[2] = motor_pos_model_to_real(2, motorsLeft(2));
				targets[3] = motor_pos_model_to_real(3, motorsLeft(3));
				targets[4] = motor_pos_model_to_real(4, motorsLeft(4));
				targets[5] = motor_pos_model_to_real(5, motorsRight(0));
				targets[6] = motor_pos_model_to_real(6, motorsRight(1));
				targets[7] = motor_pos_model_to_real(7, motorsRight(2));
				targets[8] = motor_pos_model_to_real(8, motorsRight(3));
				targets[9] = motor_pos_model_to_real(9, motorsRight(4));
				motor_targets = targets;
				fsm_state = 4;
				
				scheduleEnable();

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
			case '1':
				motor_targets = motor_zeros;
				fsm_state = 4;
				
				scheduleEnable();
				printf("Moving Motors to zero\n");
				break;
			case '2':
				motor_targets = motor_init_config;
				fsm_state = 4;
				
				scheduleEnable();
				printf("Moving Motors to standing config\n");
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
