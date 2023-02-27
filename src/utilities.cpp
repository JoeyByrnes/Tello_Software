#include "utilities.h"
#include "vn/sensors.h"
#include "../lib/Eigen/Dense"

#include <cstdio>
#include <cstdarg>
#include <cstring>

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