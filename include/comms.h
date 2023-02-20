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
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <termios.h>
#include <cstdlib>
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
#include "../lib/DynamicRobot_RDL/dynamic_robot.h"

#define UDP_MAXLINE 1024

// ========================== CAN =============================

void* rx_CAN( void * arg );

void process_motor_data(TPCANMsg Message, RoboDesignLab::DynamicRobot* robot);
void process_foot_sensor_data(TPCANMsg Message, RoboDesignLab::DynamicRobot* robot);

// ========================== UDP =============================

void* rx_UDP( void * arg );

// ========================= SERIAL ===========================

void* IMU_Comms( void * arg );