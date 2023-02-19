#include "utilities.h"
#include "vn/sensors.h"
#include "../lib/Eigen/Dense"

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

using namespace Eigen;
