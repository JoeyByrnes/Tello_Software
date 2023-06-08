#include "mujoco_utilities.h"

extern simConfig sim_conf;
extern bool en_v2_ctrl;
extern int plot_width;
extern bool showPlotMenu;

// MuJoCo data structures
extern mjModel* m;                  // MuJoCo model
extern mjData* d;                   // MuJoCo data
extern mjvCamera cam;               // abstract camera
extern mjvOption opt;               // visualization options
extern mjvScene scn;                // abstract scene
extern mjrContext con;              // custom GPU context

extern double push_force_x;
extern double push_force_y;
extern double push_force_z;
extern bool pause_sim;

double rfz;
double rbz;
double lfz;
double lbz;

int rf_cnt = 0;
int rb_cnt = 0;
int lf_cnt = 0;
int lb_cnt = 0;

bool rf_seen = false;
bool rb_seen = false;
bool lf_seen = false;
bool lb_seen = false;


extern VectorXd gnd_contacts;
extern VectorXd z_forces;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

extern int hip_motor1_r_idx ;
extern int hip_motor1_l_idx ;
extern int hip_motor2_r_idx ;
extern int hip_motor2_l_idx ;
extern int hip_motor3_r_idx ;
extern int hip_motor3_l_idx ;
extern int knee_motor_r_idx ;
extern int knee_motor_l_idx ;
extern int ankle_motor_r_idx;
extern int ankle_motor_l_idx;

// Callbacks

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        // mj_resetData(m, d);
        // sim_was_restarted = true;
        // mj_forward(m, d);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_X)
    {
        push_force_x = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_Y)
    {
        push_force_y = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_Z)
    {
        push_force_z = 10;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_C)
    {
        mju_zero(d->qfrc_applied, m->nv);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_ENTER)
    {
        pause_sim = !pause_sim;
    }
}

void window_close_callback(GLFWwindow* window)
{
    glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}
double last_xpos = 0;
double last_ypos = 0;
// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
     // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    last_xpos = xpos;
    last_ypos = ypos;
    if(ypos < 100) return;
    if(sim_conf.en_realtime_plot && xpos < plot_width) return;

    if(showPlotMenu && xpos > (width-900)) return;

    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    if(last_ypos < 100) return;
    if(sim_conf.en_realtime_plot && last_xpos < plot_width) return;
    if(showPlotMenu && last_xpos > (width-900)) return;
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void contactforce(const mjModel* m, mjData* d,int FSM)
{
    mjMARKSTACK
    mjtNum* f0_contact_pos = mj_stackAlloc(d, 3);//contact position in unkown frame. check
    mjtNum* f0_contact_frame = mj_stackAlloc(d, 9);
    mjtNum* f0_contact_force = mj_stackAlloc(d, 6);//contact force in contact frame

    bool left_front_contact_detected = false;
    bool left_back_contact_detected = false;
    bool right_front_contact_detected = false;
    bool right_back_contact_detected = false;
    gnd_contacts.setZero();
    rf_seen = false;
    rb_seen = false;
    lf_seen = false;
    lb_seen = false;
    bool right_data_reliable;
    bool left_data_reliable;
    for (int i = 0; i < d->ncon; i++)
    {
        mjContact* cur_contact = &((d->contact)[i]);
        std::string geom1 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom1);
        std::string geom2 = mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2);
        //std::cout << "contact point # " << i + 1 << std::endl;
        //std::cout << mj_id2name(m, mjOBJ_GEOM, cur_contact->geom2) << "  :"; // normal
        int ll = 0;
        int ul = 10000;
        right_data_reliable = (rfz >= ll && rbz >= ll) && (rfz <= ul && rbz <= ul);
        left_data_reliable = (lfz >= ll && lbz >= ll) && (lfz <= ul && lbz <= ul);
        int idx = -1;
        if (geom1.compare("floor") == 0 || geom2.compare("floor") == 0)
        {
            if((geom1.compare("right_foot_toe") == 0 || geom2.compare("right_foot_toe") == 0) && right_data_reliable  )
            {
                gnd_contacts(0) = 1;
                rf_seen = true;
                idx = 0;
            }
            if((geom1.compare("right_foot_heel") == 0 || geom2.compare("right_foot_heel") == 0) && right_data_reliable  )
            {
                gnd_contacts(1) = 1;
                rb_seen = true;
                idx = 1;
            }
            if((geom1.compare("left_foot_toe") == 0 || geom2.compare("left_foot_toe") == 0) && left_data_reliable  )
            {
                gnd_contacts(2) = 1;
                lf_seen = true;
                idx = 2;
            }
            if((geom1.compare("left_foot_heel") == 0 || geom2.compare("left_foot_heel") == 0) && left_data_reliable  )
            {
                gnd_contacts(3) = 1;
                lb_seen = true;
                idx = 3;
            }
            //get robot's geom id
            int bodyid;
            if(geom1.compare("floor") == 0){ //return 0 if same string
                bodyid = cur_contact->geom1;
            }else{
                bodyid = cur_contact->geom2;
            }
            //std::cout << "Collision Point = " << mj_id2name(m, mjOBJ_GEOM, bodyid) << '\n';
            // calculate position/force/normal of contact points.
            mju_copy(f0_contact_pos, cur_contact->pos, 3);
            mju_copy(f0_contact_frame, cur_contact->frame, 9); //contact frame in world frame
            mj_contactForce(m, d, i, f0_contact_force); // torque in contact frame, stored in transposed form
            //Print contact frame
            // std::cout << "Contact Frame = " << '\n';
            // mju_printMat(f0_contact_frame, 3, 3);
            //Print contact force
            mjtNum fw_contact_force[9];
            // std::cout << "Contact Force in Contact Frame = " << '\n';
            mju_transpose(fw_contact_force, f0_contact_force, 3, 3); //
            // mju_printMat(fw_contact_force, 3, 3);
            //std::cout << "Contact Force in World Frame = " << '\n';
            mjtNum f_world[9];
            mju_mulMatMat(f_world, f0_contact_frame, fw_contact_force, 3, 3, 3);
            // mju_printMat(f_world, 3, 3);
            // std::cout << f_world[0] << ", " << f_world[3] << ", " << f_world[6] << endl;

            if(idx != -1)
            {
                z_forces[idx] = abs(f_world[6]);
            }

        } // if one geom is object
    } // for i = 1:ncon

    if(!rf_seen) gnd_contacts(1) = 0;
    if(!rb_seen) gnd_contacts(0) = 0;
    if(!lf_seen) gnd_contacts(3) = 0;
    if(!lb_seen) gnd_contacts(2) = 0;

    if(!rf_seen) z_forces(0) = 0;
    if(!rb_seen) z_forces(1) = 0;
    if(!lf_seen) z_forces(2) = 0;
    if(!lb_seen) z_forces(3) = 0;

    if(FSM == 1)
    {
        gnd_contacts(0) = 0;
        gnd_contacts(1) = 0;
    }
    if(FSM == -1)
    {
        gnd_contacts(2) = 0;
        gnd_contacts(3) = 0;
    }

    // if(rf_cnt < 5) gnd_contacts(0) = 1;
    // if(rb_cnt < 5) gnd_contacts(1) = 1;
    // if(lf_cnt < 5) gnd_contacts(2) = 1;
    // if(lb_cnt < 5) gnd_contacts(3) = 1;
    // cout << rfz << ", \t" << rbz << ", \t" << lfz << ", \t" << lbz << endl;
    // cout << (int)right_data_reliable << ",\t" << (int)left_data_reliable << endl;


    mjFREESTACK
}

void applyJointTorquesMujoco(VectorXd torques)
{
    d->ctrl[hip_motor1_l_idx]  = torques(0);
    d->ctrl[hip_motor2_l_idx]  = torques(1);
    d->ctrl[hip_motor3_l_idx]  = torques(2);
    d->ctrl[knee_motor_l_idx]  = torques(3);
    d->ctrl[ankle_motor_l_idx] = torques(4);
    d->ctrl[hip_motor1_r_idx]  = torques(5);
    d->ctrl[hip_motor2_r_idx]  = torques(6);
    d->ctrl[hip_motor3_r_idx]  = torques(7);
    d->ctrl[knee_motor_r_idx]  = torques(8);
    d->ctrl[ankle_motor_r_idx] = torques(9);
}

double sigmoid(double x) { return 1 / (1 + exp(-x)); }

VectorXd mux_and_smooth(VectorXd initialOutput, VectorXd finalOutput, double start_time, double end_time, double time) 
{

    if( time < start_time ) return initialOutput;
    if( time > end_time ) return finalOutput;
    if(end_time - start_time <= 0) return finalOutput;

    // compute the sigmoid function input for each element in the vector
    double switchFactor = (time - start_time) / (end_time - start_time);  // Calculate the switch factor based on the current time step and direction flag

    if(switchFactor > 1) switchFactor = 1;

    double smoothVal = sigmoid((switchFactor - 0.5) * 14);  // Scale the switch factor to be between -5 and 5, and apply sigmoid function
    if(smoothVal < 0.001) smoothVal = 0;
    if(smoothVal > 0.999) smoothVal = 1;

    VectorXd out = ((1-smoothVal) * initialOutput) + (smoothVal * finalOutput);

    return out;
}

double smoothData(const Eigen::VectorXd& vel, double smoothingFactor) {
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

std::string executeCommand(const std::string& command) {
    std::string result;
    char buffer[128];

    // Open the command using popen and read the output
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    // Close the pipe
    pclose(pipe);

    // Remove the trailing newline character if present
    if (!result.empty() && result.back() == '\n') {
        result.pop_back();
    }

    return result;
}

using json = nlohmann::json;

simConfig readSimConfigFromFile(const std::string& filename) {
    simConfig config;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return config;
    }

    try {
        json jsonData;
        file >> jsonData;

        config.en_data_logging = jsonData["en_data_logging"];
        config.en_auto_record = jsonData["en_auto_record"];
        config.en_HMI_recording = jsonData["en_HMI_recording"];
        config.en_screen_recording = jsonData["en_screen_recording"];
        config.en_realtime_plot = jsonData["en_realtime_plot"];
        config.en_playback_mode = jsonData["en_playback_mode"];
        config.en_autonomous_mode_on_boot = jsonData["en_autonomous_mode_on_boot"];
        config.en_v2_controller = jsonData["en_v2_controller"];
        config.en_safety_monitor = jsonData["en_safety_monitor"];
        en_v2_ctrl = config.en_v2_controller;
    } catch (json::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }

    file.close();
    return config;
}

void writeSimConfigToFile(const simConfig& config, const std::string& filename) {
    json jsonData;
    jsonData["en_data_logging"] = config.en_data_logging;
    jsonData["en_auto_record"] = config.en_auto_record;
    jsonData["en_HMI_recording"] = config.en_HMI_recording;
    jsonData["en_screen_recording"] = config.en_screen_recording;
    jsonData["en_realtime_plot"] = config.en_realtime_plot;
    jsonData["en_playback_mode"] = config.en_playback_mode;
    jsonData["en_autonomous_mode_on_boot"] = config.en_autonomous_mode_on_boot;
    jsonData["en_v2_controller"] = config.en_v2_controller;
    jsonData["en_safety_monitor"] = config.en_safety_monitor;


    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    try {
        file << std::setw(4) << jsonData << std::endl;
        std::cout << "Config successfully written to file: " << filename << std::endl;
    } catch (json::exception& e) {
        std::cerr << "Error writing JSON: " << e.what() << std::endl;
    }

    file.close();
}

namespace fs = std::filesystem;

bool copyFile(const std::string& sourcePath, const std::string& destinationDir) {
    fs::path sourceFile(sourcePath);

    if (!fs::exists(sourceFile) || !fs::is_regular_file(sourceFile)) {
        std::cerr << "The source file does not exist or is not a regular file." << std::endl;
        return false;
    }

    fs::path destinationPath = fs::path(destinationDir) / sourceFile.filename();

    std::ifstream sourceFileStream(sourcePath);
    if (!sourceFileStream) {
        std::cerr << "Failed to open the source file." << std::endl;
        return false;
    }

    std::ofstream destinationFile(destinationPath.string());
    if (!destinationFile) {
        std::cerr << "Failed to open/create the destination file." << std::endl;
        return false;
    }

    destinationFile << sourceFileStream.rdbuf();

    if (sourceFileStream.bad() || destinationFile.bad()) {
        std::cerr << "An error occurred while copying the file." << std::endl;
        return false;
    }

    std::cout << "File copied successfully to: " << destinationPath << std::endl;
    return true;
}

bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
    // Load from file
    int image_width = 0;
    int image_height = 0;
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
    if (image_data == NULL)
        return false;

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    // Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;

    return true;
}

std::string readActivePlaybackLog(const std::string& filename) {

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return "";
    }
    std::string active_pb_log;
    try {
        json jsonData;
        file >> jsonData;

        active_pb_log = jsonData["active_playback_log"];
        // cout << "Active PB Log: " << active_pb_log << endl;
    } catch (json::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }

    file.close();
    return active_pb_log;
}

void writeActivePlaybackLog(const std::string log, const std::string& filename) {
    json jsonData;
    jsonData["active_playback_log"] = log;

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    try {
        file << std::setw(4) << jsonData << std::endl;
        std::cout << "Config successfully written to file: " << filename << std::endl;
    } catch (json::exception& e) {
        std::cerr << "Error writing JSON: " << e.what() << std::endl;
    }

    file.close();
}