#include "mujoco_main.h"
#include <random>

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void window_close_callback(GLFWwindow* window);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);
void contactforce(const mjModel* m, mjData* d);
void applyJointTorquesMujoco(VectorXd torques);
Eigen::Vector3d add_Noise_3D(const Eigen::Vector3d& euler_angles, const double std_dev);