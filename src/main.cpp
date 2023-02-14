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

#include <pcanfd.h>

#define TWOPI 6.28318530718
#define ENCODER_TO_RADIANS ((double)(12.5/32768.0))
#define RADIANS_TO_DEGREES ((double)(180.0/M_PI))

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

CheetahMotor *m1 ;
CheetahMotor *m2 ;
CheetahMotor *m3 ;
CheetahMotor *m4 ;
CheetahMotor *m5 ;
CheetahMotor *m6 ;
CheetahMotor *m7 ;
CheetahMotor *m8 ;
CheetahMotor *m9 ;
CheetahMotor *m10;

int enable_motors = 0;

uint16_t encoders[10];
int position_initialized[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t encoder_positions[10];
uint16_t encoder_offsets[10]  = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int motor_directions[10] =      { 1,-1, 1, 1,-1, 1,-1, 1, 1,-1};
								//1, 2, 3, 4, 5, 6, 7, 8, 9, 10
int motor_zeros[10] = {35598,35023,33523,33959,33157,32744,35038,34363,33703,34596}; // verified. Only works when legs pulled straight down before power on
int motor_init_config[10] = {35540, 36558, 31813, 38599, 31811, 32767, 36712, 32718, 38436, 33335};
int motor_initialized[10] = {0,0,0,0,0,0,0,0,0,0};
int motor_move_complete[10] = {0,0,0,0,0,0,0,0,0,0};
int motors_in_use = 10;
int mode_selected = 0;
int stop_recording = 0;
int recording_initialized = 0;
int playback_initialized = 0;
int* motor_targets;

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
		//f_motion = fopen("motion_log.txt", "w+");
		motion_log = new std::ofstream(MOTION_LOG_NAME, std::ios::trunc);
		if (!motion_log->is_open()) {
			std::cerr << "Error: unable to open file '" << MOTION_LOG_NAME << "' for writing." << std::endl;
			return;
		}
		
		recording_initialized = 1;
	}
	// record here:
	for(int i=0;i<10;i++){
		//fprintf(f_motion, "%u ", encoders[i]);
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
    //std::cout << std::endl;
	// for(int i=0;i<10;i++){
	// 	int a;
	// 	motion_in_file >> a;
	// 	if(a == 123456){
	// 		printf("\nEnd of recording reached\n");
	// 		exit(0);
	// 	}
	// 	motors[i]->setPos((uint16_t)a);
	// }
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

void moveMotors(int* positions){
	int init_sum = 0;
	for(int i=0;i<10;i++){
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
		init_sum += motor_move_complete[i];
	}
	if(init_sum == 10){
		fsm_state = 0;
		set_kp_kd_all(200,100);
		for(int x=0; x<10; x++) motor_move_complete[x] = 0;
	}
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
		//motors[i]->updateMotor();
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
			// //if(!position_initialized[i]){
			// 	motors[i]->disableMotor();
			// //}
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
		// auto end = std::chrono::high_resolution_clock::now();
		// auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
		// output_file << duration.count() << std::endl;
		// output_file.flush();
		// start = std::chrono::high_resolution_clock::now();
		
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
				set_kp_kd_all(3500,1000);
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
		//fprintf(f_motion, "123456\n");
		usleep(1000);
		//fclose(f_motion);
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
	
// Driver code
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

	//log_file = fopen("tello_log.txt", "w+"); // open file for logging data

	
	char buffer[UDP_MAXLINE];
    int cnt = 0;
  	TPCANMsg Message;
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

	// Thread setup:
	pthread_t rx_bus1, rx_bus2, rx_bus3, rx_bus4, rx_udp, update_main;
	pthread_attr_t tattr;
	pthread_attr_t tattr_high_prio;
	int ret;
	sched_param param, param_high_prio;
	/* initialized with default attributes */
	ret = pthread_attr_init (&tattr);
	ret = pthread_attr_init (&tattr_high_prio);
	/* set the scheduler of the thread to use a realtime policy */
	pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
	pthread_attr_setschedpolicy(&tattr_high_prio, SCHED_FIFO);
	/* safe to get existing scheduling param */
	ret = pthread_attr_getschedparam (&tattr, &param);
	ret = pthread_attr_getschedparam (&tattr_high_prio, &param_high_prio);
	/* set the priority; others are unchanged */
	param.sched_priority = 40;
	param_high_prio.sched_priority = 99;
    pthread_attr_setschedparam(&tattr, &param);
	pthread_attr_setschedparam(&tattr_high_prio, &param_high_prio);

	// Set the CPU affinity of the thread to the specified core
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(ISOLATED_CORE_2_THREAD_1, &cpuset);
    pthread_attr_setaffinity_np(&tattr, sizeof(cpu_set_t), &cpuset); // set the following threads to core 6

	// initialize one receive thread per CAN channel
	int rx1 = pthread_create( &rx_bus1, &tattr, &rx_CAN, (void*)(&pcd1));
	int rx2 = pthread_create( &rx_bus2, &tattr, &rx_CAN, (void*)(&pcd2));
	int rx3 = pthread_create( &rx_bus3, &tattr, &rx_CAN, (void*)(&pcd3));
	int rx4 = pthread_create( &rx_bus4, &tattr, &rx_CAN, (void*)(&pcd4));
	pthread_setschedparam(rx_bus1, SCHED_RR, &param);
	pthread_setschedparam(rx_bus2, SCHED_RR, &param);
	pthread_setschedparam(rx_bus3, SCHED_RR, &param);
	pthread_setschedparam(rx_bus4, SCHED_RR, &param);

    CPU_ZERO(&cpuset);
    CPU_SET(ISOLATED_CORE_2_THREAD_2, &cpuset);
    pthread_attr_setaffinity_np(&tattr, sizeof(cpu_set_t), &cpuset); // set the following threads to core 7

	// initialize receive thread for UDP communication
	int rxUDP = pthread_create( &rx_udp, &tattr, &rx_UDP, (void*)(NULL));

    CPU_ZERO(&cpuset);
    CPU_SET(ISOLATED_CORE_1_THREAD_2, &cpuset);
    pthread_attr_setaffinity_np(&tattr, sizeof(cpu_set_t), &cpuset); // set the following threads to core 5

	// initialize main update thread that sends commands to motors
	int update_th = pthread_create( &update_main, &tattr_high_prio, &update_1kHz, (void*)(NULL));
	pthread_setschedparam(update_main, SCHED_FIFO, &param_high_prio);

	usleep(1000);
	
	while(1){
		// printf("\n\nTello Software Menu:\n");
		// printf("u : Enter UDP Control Mode\n");
		// printf("r : Enter Motion Recording Mode\n");
		// printf("e : Exit Motion Recording Mode\n");
		// printf("p : Enter Motion Playback Mode\n");
		// printf("i : Enter Idle Mode (or press any other unused key)\n\n");
		Eigen::Matrix<double,5,1> jointsLeft;
		Eigen::VectorXd motorsLeft;
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
				printf("\nJoint Debug:\n");
				jointsLeft(0) = 0.01;
				jointsLeft(1) = 0.01;
				jointsLeft(2) = 0.01;
				jointsLeft(3) = 0.01;
				jointsLeft(4) = 0.01;
				motorsLeft = fcn_ik_q_2_p(jointsLeft);
				for(int i = 0;i<5;i++){
					printf("M(deg) %d: %f ",i+1, (float)(motorsLeft(i)*RADIANS_TO_DEGREES) );
				}
				printf("\n");
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
