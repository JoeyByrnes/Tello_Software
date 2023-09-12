#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <sys/resource.h>
#include <iostream>
#include <iomanip>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <termios.h>
#include <cstdlib>
#include "vn/sensors.h"
#include "../lib/Eigen/Dense"
#include <ctime>
#include <chrono>
#include <sstream>
#include <sys/stat.h>

#include "mujoco_main.h"

#define GOV_PERFORMANCE "performance"
#define GOV_POWERSAVE "powersave"

void set_cpu_gov(int cpu, const char* gov);
bool set_cpu_governor(const char* gov);
void assignToCore(int core_id);
void print_cpu_speed(int core_number);

void scheduleEnable();
void scheduleDisable();
void scheduleZero();

void quaternion_to_euler(double q0, double q1, double q2, double q3, float &roll, float &pitch, float &yaw);
vn::math::mat3f eigenToVnMatrix(const Eigen::Matrix3d& eigenMat);

void printf(char color, const char* message, ...);

bool optimize_serial_communication(std::string portName);

std::string getCurrentDateTime();
std::string createLogFolder(const std::string& location);

double ema_filter(const Eigen::VectorXd& vel, double smoothingFactor);

Eigen::Vector3d calculateSupportCenter(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& v4);