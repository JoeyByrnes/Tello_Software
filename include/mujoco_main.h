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
#include "IconsFontAwesome5.h"

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
#include "Driver/DualShock4Connector.h"\

// robot states indices
#define torso_x_idx         (int)0
#define torso_y_idx         (int)1
#define torso_z_idx         (int)2
#define torso_roll_idx      (int)3
#define torso_pitch_idx     (int)4
#define torso_yaw_idx       (int)5
#define hip_yaw_r_idx       (int)6
#define hip_roll_r_idx      (int)7
#define hip_pitch_r_idx     (int)8
#define knee_pitch_r_idx    (int)9
#define ankle_pitch_r_idx   (int)10
#define hip_yaw_l_idx       (int)11
#define hip_roll_l_idx      (int)12
#define hip_pitch_l_idx     (int)13
#define knee_pitch_l_idx    (int)14
#define ankle_pitch_l_idx   (int)15

// robot actuators indices
#define hip_motor1_r_idx    (int)0
#define hip_motor1_l_idx    (int)1
#define hip_motor2_r_idx    (int)2
#define hip_motor2_l_idx    (int)3
#define hip_motor3_r_idx    (int)4
#define hip_motor3_l_idx    (int)5
#define knee_motor_r_idx    (int)6
#define knee_motor_l_idx    (int)7
#define ankle_motor_r_idx   (int)8
#define ankle_motor_l_idx   (int)9


// robot states indices
#define hip_yaw_r_idx_viz       (int)16
#define hip_roll_r_idx_viz      (int)17
#define hip_pitch_r_idx_viz     (int)18
#define knee_pitch_r_idx_viz    (int)19
#define ankle_pitch_r_idx_viz   (int)20
#define hip_yaw_l_idx_viz       (int)21
#define hip_roll_l_idx_viz      (int)22
#define hip_pitch_l_idx_viz     (int)23
#define knee_pitch_l_idx_viz    (int)24
#define ankle_pitch_l_idx_viz   (int)25

// robot actuators indices
#define hip_motor1_r_idx_viz  (int)(10+0)
#define hip_motor1_l_idx_viz  (int)(10+1)
#define hip_motor2_r_idx_viz  (int)(10+2)
#define hip_motor2_l_idx_viz  (int)(10+3)
#define hip_motor3_r_idx_viz  (int)(10+4)
#define hip_motor3_l_idx_viz  (int)(10+5)
#define knee_motor_r_idx_viz  (int)(10+6)
#define knee_motor_l_idx_viz  (int)(10+7)
#define ankle_motor_r_idx_viz (int)(10+8)
#define ankle_motor_l_idx_viz (int)(10+9)


struct VisualizationData {
    float CoM_pos_measured[3];
    float CoM_rpy_measured[3];
    float q_measured[10];
    float CoM_pos_desired[3];
    float CoM_rpy_desired[3];
    float q_desired[10];
    float CoP[3];
    float GRFs[3];
};

// Define the packed struct for button data
#pragma pack(1)
struct HW_CTRL_Data {
    bool tare_hmi;
    bool start_legs;
    bool balance;
    bool enable_teleop;
    float hmi_gain;
    bool emergency_stop;
    bool enable_datalogging;
    bool enable_v2_controller;
    bool auto_mode;
    bool enable_x_force;
    bool enable_force_feedback;
    bool enable_safety_monitor;
    float human_x_zero;
    bool set_full_joint_kp;
    bool start_dcm_tracking;
};
#pragma pack()


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
    double xddR;
    double yR;
    double ydR;
    double yddR;
    double zR;
    double zdR;
    double zddR;
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
    double grf_rf;
    double grf_rb;
    double grf_lf;
    double grf_lb;

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

    void setTime(double tsec){
        t = tsec;
    }
    void setGRFs(double rf, double rb, double lf, double lb){
        grf_rf = rf;
        grf_rb = rb;
        grf_lf = lf;
        grf_lb = lb;
    }

    void setCoMPos(Vector3d pos){
        xR = pos(0);
        yR = pos(1);
        zR = pos(2);
    }
    void setCoMVel(Vector3d vel){
        xdR = vel(0);
        ydR = vel(1);
        zdR = vel(2);
    }
    void setCoMRPY(Vector3d rpy){
        phiR = rpy(0);
        thetaR = rpy(1);
        psiR = rpy(2);
    }
    void setCoMAngVel(Vector3d drpy){
        phidR = drpy(0);
        thetadR = drpy(1);
        psidR = drpy(2);
        angular_velocity[0] = drpy(0);
        angular_velocity[1] = drpy(1);
        angular_velocity[2] = drpy(2);
    }
    void setJointPos(VectorXd pos_LR){
        q1l = pos_LR(0);
        q2l = pos_LR(1);
        q3l = pos_LR(2);
        q4l = pos_LR(3);
        q5l = pos_LR(4);
        q1r = pos_LR(5);
        q2r = pos_LR(6);
        q3r = pos_LR(7);
        q4r = pos_LR(8);
        q5r = pos_LR(9);
    }
    void setJointVel(VectorXd vel_LR){
        qd1l = vel_LR(0);
        qd2l = vel_LR(1);
        qd3l = vel_LR(2);
        qd4l = vel_LR(3);
        qd5l = vel_LR(4);
        qd1r = vel_LR(5);
        qd2r = vel_LR(6);
        qd3r = vel_LR(7);
        qd4r = vel_LR(8);
        qd5r = vel_LR(9);
    }
    void setCoMAcc(Vector3d acc){
        xddR = acc(0);
        yddR = acc(1);
        zddR = acc(2);
        acceleration[0] = acc(0);
        acceleration[1] = acc(1);
        acceleration[2] = acc(2);
    }

    // Assignment operator
    ctrlData& operator=(const ctrlData& other) {
        t = other.t;
        xR = other.xR;
        xdR = other.xdR;
        xddR = other.xddR;
        yR = other.yR;
        ydR = other.ydR;
        yddR = other.yddR;
        zR = other.zR;
        zdR = other.zdR;
        zddR = other.zddR;
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
        grf_rf = other.grf_rf;
        grf_rb = other.grf_rb;
        grf_lf = other.grf_lf;
        grf_lb = other.grf_lb;
        // mujoco_lfv = other.mujoco_lfv;
        std::memcpy(acceleration, other.acceleration, sizeof(acceleration));
        std::memcpy(angular_velocity, other.angular_velocity, sizeof(angular_velocity));
        return *this;
    }
};

void TELLO_locomotion_ctrl(ctrlData cd);
void* mujoco_Update_1KHz( void * arg );
void* tello_controller( void * arg );
void* plotting( void * arg );
void* plot_human_data( void * arg );
void* PS4_Controller( void * arg );
void* Human_Playback( void * arg );
void* sim_step_task( void * arg );
void* Animate_Log( void * arg );
void* visualize_robot( void * arg );
void* curve_fitting( void * arg );
ImVec4 hex2ImVec4(int hex);
void* visualization_render_thread( void * arg );
void initializeSRBMCtrl();

#endif