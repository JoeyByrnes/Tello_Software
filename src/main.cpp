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

#include "math.h"

#include "cheetah_motor.h"
#include "comms.h"
#include "timer.h"
#include "user_config.h"
#include "utilities.h"

#define TWOPI 6.28318530718

int udp_data_ready = 0;
char udp_control_packet[UDP_MAXLINE];
	
unsigned int num_CAN_writes_since_reset = 0;

float averageTime = 0;
float maxTime = 0;

FILE *log_file, *f_tx, *f_motion;

struct timeval st, et;
struct timeval update_st, update_et;

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
int motors_in_use = 5;
int mode_selected = 0;
int stop_recording = 0;

CheetahMotor* motors[10];

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
		motors[i]->setKp(kp);
		motors[i]->setKp(kd);
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

}
void handle_Motion_Playback(){

}



static void* update_1kHz( void * arg )
{
	startTimer();
    
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
		motors[i]->setKp(800);
		motors[i]->setKd(5);
		motors[i]->setPos(32768);
		motors[i]->updateMotor();
		motors[i]->disableMotor();
	}

	int pos_initialized = 0;
	while(pos_initialized < motors_in_use)
	{
		pos_initialized = 0;
		for(int i=0;i<10;i++)
		{
			pos_initialized+=position_initialized[i];
			motors[i]->disableMotor();
		}
		usleep(10000);
	}
	printf("\nAll motors Initialized, Enabling.\n");
	for(int i=0;i<10;i++)
	{
		pos_initialized+=position_initialized[i];
		motors[i]->setPos(encoder_offsets[i]);
		motors[i]->updateMotor();
		motors[i]->enableMotor();
	}

	usleep(1000);

	uint16_t pos_cmds[10];

	printf("\n\nEnter CTRL+C in terminal to exit.\n\n");

	gettimeofday(&update_st,NULL);
	usleep(990);

	while(1)
	{
		handle_start_of_periodic_task();
		// Write update loop code under this line ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		switch(fsm_state){
			case 0:
				disable_all();
				// Nothing to be done here
				break;
			case 1:
				set_kp_kd_all(400,200);
				handle_UDP_Commands();
				break;
			case 2:
				handle_Motion_Recording();
				break;
			case 3:
				handle_Motion_Playback();
				break;
			default:
				disable_all();
				break;
		}

		update_all_motors();

		// Write update loop code above this line ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		handle_end_of_periodic_task();
	}

	return NULL;
}

// This callback handles CTRL+C Input
void signal_callback_handler(int signum){

	for(int i=0;i<10;i++){
		motors[i]->disableMotor();
	}

	fclose(log_file);
	printf("\n\n==================== Exiting Tello Software ====================\n");
	set_cpu_gov(6,GOV_POWERSAVE);
	printf("CPU Governor set to Powersave (Approx. 900MHz on Up Xtreme 866)\n\n\n");
	
	usleep(1000);
	exit(signum);
}
	
// Driver code
int main() {
	setvbuf(stdout, NULL, _IONBF, 0); // no buffering on printf, change to _IOLBF for buffering until \n character
	printf("\n==================== Running Tello Software ====================\n\n");
	assignToCore(ISOLATED_CORE);

	set_cpu_gov(6,GOV_PERFORMANCE);
	printf("CPU Governor set to Performance (Max 4.4GHz on Up Xtreme 866)\n");
	
    setpriority(PRIO_PROCESS, 0, 19); // Set NICE Priority in case user doesn't have RT Permission

	signal(SIGINT, signal_callback_handler); // Handle CTRL+C input


    // int chrt_err = sched_setscheduler(0, SCHED_RR, &sp); // set scheduler to FIFO

    // int policy = sched_getscheduler(0);
    // if (chrt_err == -1) {
    //     perror("Your user does not have RealTime Scheduler Permissions.");
    //     return 1;
    // }
    // switch(policy) {
    //     case SCHED_OTHER: printf("Scheduler in use does not support Real-Time\n"); break;
    //     case SCHED_RR:   printf("Using Round Robin Scheduler (Real-Time Capable)\n"); break;
    //     case SCHED_FIFO:  printf("Using FIFO Scheduler(Real-Time Capable)\n"); break;
    //     default:   printf("Unknown Scheduler...does not support Real-Time\n");
    // }

	// // PRINT CORE AND PRIORITIES =========================================================
	// printf("Process assigned to core %d\n",ISOLATED_CORE);
	// int prio = getpriority(0,0);
	// printf("NICE Priority: %d\n",prio);
	// if(!chrt_err) printf("RT Priority: %d\n",sp.sched_priority);
	// else printf("Error setting RT Priority");

	log_file = fopen("tello_log.txt", "w+"); // open file for logging data
	
	
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
	int ret;
	int newprio = 15;
	sched_param param;
	/* initialized with default attributes */
	ret = pthread_attr_init (&tattr);
	/* safe to get existing scheduling param */
	ret = pthread_attr_getschedparam (&tattr, &param);
	/* set the priority; others are unchanged */
	param.sched_priority = newprio;
	/* setting the new scheduling param */
	ret = pthread_attr_setschedparam (&tattr, &param);

	int rx1 = pthread_create( &rx_bus1, 0, &rx_CAN, (void*)(&pcd1));
	int rx2 = pthread_create( &rx_bus2, 0, &rx_CAN, (void*)(&pcd2));
	int rx3 = pthread_create( &rx_bus3, 0, &rx_CAN, (void*)(&pcd3));
	int rx4 = pthread_create( &rx_bus4, 0, &rx_CAN, (void*)(&pcd4));

	int rxUDP = pthread_create( &rx_udp, 0, &rx_UDP, (void*)(NULL));

	newprio = 20;
	ret = pthread_attr_setschedparam (&tattr, &param);
	int update_th = pthread_create( &update_main, 0, &update_1kHz, (void*)(NULL));

	usleep(1000);
	
	while(1){
		printf("\n\nTello Software Menu:\n");
		printf("u : Enter UDP Control Mode\n");
		printf("r : Enter Motion Recording Mode\n");
		printf("p : Enter Motion Playback Mode\n");
		printf("i : Enter Idle Mode (or press any other unused key)\n\n");
		char choice;
		std::cin >> choice;
		switch(choice){
			case 'u':
				fsm_state = 1;
				printf("\nEntering UDP Mode\n");
				break;
			case 'r':
				fsm_state = 2;
				printf("\nEntering Motion Recording Mode\n");
				break;
			case 'p':
				fsm_state = 3;
				printf("\nEntering Motion Playback Mode\n");
				break;
			default:
				printf("\nEntering Idle Mode");
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
