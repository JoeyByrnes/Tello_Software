#ifndef __MUJOCO_MAIN_H__ 
#define __MUJOCO_MAIN_H__ 
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
#include "timer.h"
#include "utilities.h"
#include "json.hpp"
#include <mutex>
#include "../lib/DynamicRobot_RDL/dynamic_robot.h"
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include "SRBM-Ctrl/structs.h"
#include <matplotlibcpp.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <glm/glm.hpp>
#include <filesystem>

#include "SRBM-Ctrl/utilities.h"
#include "SRBM-Ctrl/controllers.h"
#include "SRBM-Ctrl/dynamics.h"
#include "SRBM-Ctrl/initialization.h"
#include "SRBM-Ctrl/kinematics.h"
#include "SRBM-Ctrl/planner.h"
#include "SRBMController.h"

// PS4 Controller  includes:
#include "Driver/DualShock4.h"
#include "Driver/DataPacket/DataPacket.h"
#include "Driver/DualShock4Connector.h"

void* mujoco_Update_1KHz( void * arg );
void* plotting( void * arg );
void* plot_human_data( void * arg );
void* PS4_Controller( void * arg );

#endif