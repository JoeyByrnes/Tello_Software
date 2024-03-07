#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "../lib/Eigen/Dense"
#include "../lib/DynamicRobot_RDL/dynamic_robot.h"
#include "utilities.h"
#include "timer.h"
#include "mujoco_main.h"

#include "ethercat.h"


void* ecat_comms( void * arg );