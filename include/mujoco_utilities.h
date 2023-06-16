#ifndef __MUJOCO_UTILS__
#define __MUJOCO_UTILS__
#include "mujoco_main.h"
#include <random>
#include <filesystem>
#include <stb_image.h>

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void window_close_callback(GLFWwindow* window);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);
void contactforce(const mjModel* m, mjData* d, int FSM);
void applyJointTorquesMujoco(VectorXd torques);
Eigen::Vector3d add_Noise_3D(const Eigen::Vector3d& euler_angles, const double std_dev);
VectorXd mux_and_smooth(VectorXd initialOutput, VectorXd finalOutput, double start_time, double end_time, double time);
double smoothData(const Eigen::VectorXd& vel, double smoothingFactor);
double firstOrderFilter(const Eigen::VectorXd& x, double alpha);
std::string executeCommand(const std::string& command);
simConfig readSimConfigFromFile(const std::string& filename);
void writeSimConfigToFile(const simConfig& config, const std::string& filename);
bool copyFile(const std::string& sourcePath, const std::string& destinationPath);
bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height);
std::string readActivePlaybackLog(const std::string& filename);
void writeActivePlaybackLog(const std::string log, const std::string& filename);

#endif