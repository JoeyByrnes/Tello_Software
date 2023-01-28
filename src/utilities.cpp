#include "utilities.h"

cpu_set_t  mask;
struct sched_param sp = { .sched_priority = 99 };

// Similar to running the following but doesn't start a whole new process:
//system("echo \"performance\" | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor\n");
void set_cpu_gov(int cpu, const char* gov){
    char num = '0' + cpu;
    std::string filename = "/sys/devices/system/cpu/cpu" + std::string(&num) + "/cpufreq/scaling_governor";
    FILE* cpu_gov= fopen(filename.c_str(), "w");
	fprintf(cpu_gov, "%s", gov);
	fclose(cpu_gov);
}

void assignToCore(int core_id)
{
    CPU_ZERO(&mask);
    CPU_SET(core_id, &mask);
    sched_setaffinity(0, sizeof(mask), &mask);
}