// Use this file for configuring Tello software all in one place

#define UPDATE_HZ 1000

#define ISOLATED_CORE_1_THREAD_1 7
#define ISOLATED_CORE_1_THREAD_2 8
#define ISOLATED_CORE_2_THREAD_1 9
#define ISOLATED_CORE_2_THREAD_2 10
#define ISOLATED_CORE_3_THREAD_1 12
#define ISOLATED_CORE_3_THREAD_2 13
#define ISOLATED_CORE_4_THREAD_1 14
#define ISOLATED_CORE_4_THREAD_2 15

#define UDP_RECEIVE_PORT 54003
#define UDP_TRANSMIT_PORT 54004
#define HMI_IP_ADDRESS "169.254.212.117"

#define MOTION_LOG_NAME "motion_log.txt"

#define MOTOR_TIMEOUT 3 //Milliseconds until motors cut out

#define SIMULATED_COMMS_LATENCY_US 750

#define TELEOP_FILTER_GAIN 2

#define FOOT_2_JOYSTICK 0.1080
