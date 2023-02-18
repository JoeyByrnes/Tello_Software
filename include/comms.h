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
#include "user_config.h"
#include "utilities.h"
#include <mutex>
// #include "cheetah_motor.h"
#include "vn/sensors.h"
#include "vn/compositedata.h"
#include "vn/util.h"
#include "../lib/Eigen/Dense"

#define UDP_MAXLINE 1024

// ========================== CAN =============================

void* rx_CAN( void * arg );

void process_motor_data(TPCANMsg Message);
void process_foot_sensor_data(TPCANMsg Message);

// ========================== UDP =============================

void* rx_UDP( void * arg );

// ========================= SERIAL ===========================

void* IMU_Comms( void * arg );