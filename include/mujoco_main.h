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
#include <implot.h>
#include <glm/glm.hpp>
#include <filesystem>

#include "rt_nonfinite.h"
#include "step_z_curve_fit.h"
#include "step_z_curve_fit_terminate.h"
#include "coder_array.h"

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
void* tello_controller( void * arg );
void* plotting( void * arg );
void* plot_human_data( void * arg );
void* PS4_Controller( void * arg );
void* Human_Playback( void * arg );
void* logging( void * arg );
void* sim_step_task( void * arg );
void* screenRecord( void * arg );
void* usbCamRecord( void * arg );
void* Animate_Log( void * arg );
void* curve_fitting( void * arg );

struct userProfile {
    std::string name;
    double lip_height;
    double weight;
    std::string config_filename;
};

struct simConfig {
    bool en_data_logging;
    bool en_auto_record;
    bool en_HMI_recording;
    bool en_screen_recording;
    bool en_realtime_plot;
    bool en_playback_mode;
    bool en_v2_controller;
    bool en_autonomous_mode_on_boot;
    bool en_autonomous_mode;
    bool en_safety_monitor;
    bool en_x_haptic_force;
    bool en_force_feedback;
    bool en_full_hmi_controls;
    bool en_human_control;
    bool en_live_variable_view;
    bool en_ps4_controller;
};

struct ctrlData {
    double t;
    double xR;
    double xdR;
    double yR;
    double ydR;
    double zR;
    double zdR;
    double phiR;
    double phidR;
    double thetaR;
    double thetadR;
    double psiR;
    double psidR; 
    double q1l;
    double q2l;
    double q3l;         
    double q4l;
    double q5l;  
    double q1r;
    double q2r;
    double q3r;         
    double q4r;
    double q5r;
    double qd1l;
    double qd2l;
    double qd3l;      
    double qd4l;
    double qd5l;
    double qd1r;
    double qd2r;
    double qd3r;      
    double qd4r;
    double qd5r;
    // MatrixXd mujoco_lfv = MatrixXd(4,3);
    mjtNum acceleration[3];
    mjtNum angular_velocity[3];

    // Default constructor
    ctrlData() {
    }

    // Copy constructor
    ctrlData(const ctrlData& other) {
        *this = other;
    }

    // Assignment operator
    ctrlData& operator=(const ctrlData& other) {
        t = other.t;
        xR = other.xR;
        xdR = other.xdR;
        yR = other.yR;
        ydR = other.ydR;
        zR = other.zR;
        zdR = other.zdR;
        phiR = other.phiR;
        phidR = other.phidR;
        thetaR = other.thetaR;
        thetadR = other.thetadR;
        psiR = other.psiR;
        psidR = other.psidR;
        q1l = other.q1l;
        q2l = other.q2l;
        q3l = other.q3l;
        q4l = other.q4l;
        q5l = other.q5l;
        q1r = other.q1r;
        q2r = other.q2r;
        q3r = other.q3r;
        q4r = other.q4r;
        q5r = other.q5r;
        qd1l = other.qd1l;
        qd2l = other.qd2l;
        qd3l = other.qd3l;
        qd4l = other.qd4l;
        qd5l = other.qd5l;
        qd1r = other.qd1r;
        qd2r = other.qd2r;
        qd3r = other.qd3r;
        qd4r = other.qd4r;
        qd5r = other.qd5r;
        // mujoco_lfv = other.mujoco_lfv;
        std::memcpy(acceleration, other.acceleration, sizeof(acceleration));
        std::memcpy(angular_velocity, other.angular_velocity, sizeof(angular_velocity));
        return *this;
    }
};

#endif