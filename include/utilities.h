#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <sys/resource.h>
#include <iostream>
#include <iomanip>

#define GOV_PERFORMANCE "performance"
#define GOV_POWERSAVE "powersave"

void set_cpu_gov(int cpu, const char* gov);
bool set_cpu_governor(const char* gov);
void assignToCore(int core_id);
void print_cpu_speed(int core_number);

void scheduleEnable();
void scheduleDisable();
void scheduleZero();
