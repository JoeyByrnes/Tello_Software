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
#include <string.h>
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
#include <user_config.h>

#define MICROSEC_PER_SEC 1000000

void startTimer();

void stopTimer();

int stopTimer(FILE * file);

int log_task_time();

void handle_periodic_task_scheduling(struct timespec &next);

void handle_start_of_periodic_task(struct timespec &next);
void handle_end_of_periodic_task(struct timespec &next);
void handle_end_of_periodic_task(struct timespec &next,int period_us);