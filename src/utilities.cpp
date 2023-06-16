#include "utilities.h"
#include "vn/sensors.h"
#include "../lib/Eigen/Dense"

#include <cstdio>
#include <cstdarg>
#include <cstring>

using namespace Eigen;

cpu_set_t  mask;

bool enableScheduled = 0;
bool disableScheduled = 0;
bool zeroScheduled = 0;

bool set_cpu_governor(const char* gov)
{
    std::ofstream governor_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
    if (!governor_file.is_open())
    {
        return false;
    }

    governor_file << gov;
    governor_file.close();

    return true;
}

void assignToCore(int core_id)
{
    CPU_ZERO(&mask);
    CPU_SET(core_id, &mask);
    sched_setaffinity(0, sizeof(mask), &mask);
}

void print_cpu_speed(int core_number) {
    std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(core_number) + "/cpufreq/scaling_cur_freq";

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file " << path << std::endl;
        return;
    }

    int frequency;
    file >> frequency;
    
    std::cout << "CPU Core " << core_number << " Speed: " << (float)frequency / 1000000.0 << " GHz" << std::endl;
}

void scheduleEnable()
{
    enableScheduled = true;
    disableScheduled = false;
}
void scheduleDisable()
{
    disableScheduled = true;
    enableScheduled = false;
}
void scheduleZero()
{
    zeroScheduled = true;
}

void printf(char color, const char* message, ...) {
    va_list args;
    va_start(args, message);

    switch (color) {
        case 'r':
            printf("\033[1;31m");
            break;
        case 'g':
            printf("\033[1;32m");
            break;
        case 'y':
            printf("\033[1;33m");
            break;
        case 'o':
            printf("\033[1;38;5;208m");
            break;
        case 'b':
            printf("\033[1;34m");
            break;
        case 'm':
            printf("\033[1;95m");
            break;
        case 'c':
            printf("\033[1;36m");
            break;
        case 'w':
            printf("\033[1;37m");
            break;
        case 'u':
            printf("\033[4;37m");
            break;
        default:
            break;
    }

    vprintf(message, args);
    printf("\033[0m");  // Reset text color to default
    va_end(args);
}


bool optimize_serial_communication(std::string portName)
{
  int portFd = -1;

  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    return false;
  }

  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%m-%d-%y__%H-%M-%S");
    return ss.str();
}

std::string createLogFolder(const std::string& location) {
    std::string folderName = getCurrentDateTime();
    std::string folderPath = location + "/" + folderName;

    // Create the folder
    if (mkdir(folderPath.c_str(), 0777) == -1) {
        std::cerr << "Failed to create folder." << std::endl;
        return "";
    }

    std::cout << "Log Folder created successfully: " << folderPath << std::endl;
    return folderPath + "/";
}

double ema_filter(const Eigen::VectorXd& vel, double smoothingFactor) 
{
    int n = vel.size();
    Eigen::VectorXd smoothedVel(n);
    smoothedVel.setZero();

    if (n > 0) {
        smoothedVel(0) = vel(0);
        int maxSamples = std::min(n, 100);
        for (int i = 1; i < maxSamples; ++i) {
            double weight = smoothingFactor / (i + 1);
            smoothedVel(i) = (1 - weight) * smoothedVel(i - 1) + weight * vel(i);
        }
        for (int i = maxSamples; i < n; ++i) {
            double weight = smoothingFactor / maxSamples;
            smoothedVel(i) = (1 - weight) * smoothedVel(i - 1) + weight * vel(i);
        }
    }

    return smoothedVel(n - 1);
}