#include "owl.hpp"
#include "../lib/Eigen/Dense"
#include "../lib/DynamicRobot_RDL/dynamic_robot.h"
#include "utilities.h"
#include "timer.h"
#include "mujoco_main.h"

void* motion_capture( void * arg );