#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <sys/resource.h>

#define GOV_PERFORMANCE "performance"
#define GOV_POWERSAVE "powersave"

void set_cpu_gov(int cpu, const char* gov);
void assignToCore(int core_id);
