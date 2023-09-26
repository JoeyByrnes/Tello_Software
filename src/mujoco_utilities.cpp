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
extern mjtNum push_force[3];
extern double impulse_start_time;
extern bool impulse_scheduled;
extern bool impulse_active;
extern double impulse_force_newtons;
extern bool ball_throw_scheduled;
extern bool start_target_motion;
extern bool ramp_toggle;

extern bool sim_window_close_requested;

extern double screen_recording;
extern double usbcam_recording;
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

// Recording
extern FILE* screen_record_pipe;
extern pid_t screen_rec_pid;
extern bool recording_in_progress;
extern bool usb_recording_in_progress;

//Logging
extern bool log_data_ready;
extern bool sim_step_completed;
extern std::string log_folder;
extern VectorXd x_out, u_out, q_out, qd_out,full_tau_out, tau_out, tau_ext_out, lfv_out, lfdv_out,lfv_comm_out,lfdv_comm_out, t_n_FSM_out, impulse_out, meas_grf_out, xDCM_out;
extern VectorXd target_pos_out;
extern VectorXd target_vel_out;
extern double last_log_time;
extern Human_dyn_data hdd_out;
extern Traj_planner_dyn_data tpdd_out;


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
    // if (act == GLFW_PRESS && key == GLFW_KEY_X)
    // {
    //     push_force_x = 10;
    // }
    // if (act == GLFW_PRESS && key == GLFW_KEY_Y)
    // {
    //     push_force_y = 10;
    // }
    // if (act == GLFW_PRESS && key == GLFW_KEY_Z)
    // {
    //     push_force_z = 10;
    // }
    if(!impulse_scheduled)
    {
        if (act == GLFW_PRESS && key == GLFW_KEY_UP)
        {
            push_force[0] = impulse_force_newtons;
            push_force[1] = 0;
            impulse_scheduled = true;
            impulse_start_time = d->time;
        }
        if (act == GLFW_PRESS && key == GLFW_KEY_DOWN)
        {
            push_force[0] = -impulse_force_newtons;
            push_force[1] = 0;
            impulse_scheduled = true;
            impulse_start_time = d->time;
        }
        if (act == GLFW_PRESS && key == GLFW_KEY_LEFT)
        {
            push_force[1] = impulse_force_newtons;
            push_force[0] = 0;
            impulse_scheduled = true;
            impulse_start_time = d->time;
        }
        if (act == GLFW_PRESS && key == GLFW_KEY_RIGHT)
        {
            push_force[1] = -impulse_force_newtons;
            push_force[0] = 0;
            impulse_scheduled = true;
            impulse_start_time = d->time;
        }
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_H)
    {
        d->mocap_pos[2] = -1;
        mj_kinematics(m,d);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_S)
    {
        d->mocap_pos[2] = -0.579;
        start_target_motion = true;
         mj_kinematics(m,d);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_R)
    {
        ramp_toggle = true;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_T)
    {
        ramp_toggle = false;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_B)
    {
        ball_throw_scheduled = true;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_EQUAL)
    {
        impulse_force_newtons += 10.0;
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_MINUS)
    {
        impulse_force_newtons -=10.0;
    }
    if(impulse_force_newtons > 30.0) impulse_force_newtons = 30.0;
    if(impulse_force_newtons < 10.0) impulse_force_newtons = 10.0;

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
    sim_window_close_requested = true;
    std::cout << "Closing the Simulation." << std::endl;
    screen_recording = false;
    usbcam_recording = false;
    system("killall -2 ffmpeg");
    cout << endl << endl << "Please wait while the recordings are stopped." << endl;
    for(int i=0;i<20;i++)
    {
        usleep(50000);
        cout << ".";
        cout.flush();
    }
    cout << endl;
    cout << "Recording stopped." << endl;

    // Free MuJoCo resources
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

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
    if(ypos < 300) return;
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
        config.en_x_haptic_force = jsonData["en_x_haptic_force"];
        config.en_force_feedback = jsonData["en_force_feedback"];
        config.en_human_control = false;
        config.en_full_hmi_controls = true;
        config.en_live_variable_view = jsonData["en_live_variable_view"];
        config.en_ps4_controller = jsonData["en_ps4_controller"];
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
    jsonData["en_x_haptic_force"] = config.en_x_haptic_force;
    jsonData["en_force_feedback"] = config.en_force_feedback;
    // jsonData["en_human_control"] = config.en_human_control;
    // jsonData["en_full_hmi_controls"] = config.en_full_hmi_controls;
    jsonData["en_live_variable_view"] = config.en_live_variable_view;
    jsonData["en_ps4_controller"] = config.en_ps4_controller;


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

void readProfilesFromJson(const std::string& filename, std::vector<userProfile>& profiles, std::string& active_user)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cout << "Failed to open JSON file: " << filename << std::endl;
        return;
    }

    nlohmann::json root;
    //active_user = root["active_user"].get<std::string>();
    try {
        ifs >> root;
    } catch (const nlohmann::json::parse_error& e) {
        std::cout << "Failed to parse JSON: " << e.what() << std::endl;
        return;
    }

    const auto& profilesArray = root["Profiles"];
    for (std::size_t i = 0; i < profilesArray.size(); ++i) {
        userProfile profile;
        profile.name = profilesArray[i]["name"].get<std::string>();
        profile.lip_height = profilesArray[i]["Height"].get<double>();
        profile.weight = profilesArray[i]["Weight"].get<double>();
        profile.config_filename = profilesArray[i]["srb_file"].get<std::string>();
        profiles.push_back(profile);
    }
    active_user = root["active_user"].get<std::string>();
}

void updateActiveUserInJson(const std::string& filename, const std::string& newActiveUser) {
    // Read the existing JSON file
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        std::cout << "Failed to open JSON file: " << filename << std::endl;
        return;
    }

    nlohmann::json root;
    try {
        ifs >> root;
    } catch (const nlohmann::json::parse_error& e) {
        std::cout << "Failed to parse JSON: " << e.what() << std::endl;
        return;
    }

    // Update the "active_user" field
    root["active_user"] = newActiveUser;

    // Write the modified JSON back to the file
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cout << "Failed to open JSON file for writing: " << filename << std::endl;
        return;
    }

    ofs << root.dump(4);  // Write the JSON with an indentation of 4 spaces
    ofs.close();

    std::cout << "Active user updated successfully in the JSON file." << std::endl;
}

double cubicEaseInOut(double t) {
    if (t < 0.5) {
        return 4 * t * t * t;
    } else {
        double x = (2 * t) - 2;
        return 0.5 * x * x * x + 1;
    }
}

double moveJoint(double currentTime, double startTime, double endTime, double startPoint, double endPoint) {
    if (currentTime < startTime) {
        return startPoint;
    } else if (currentTime > endTime) {
        return endPoint;
    } else {
        double t = (currentTime - startTime) / (endTime - startTime);
        double direction = (endPoint >= startPoint) ? 1.0 : -1.0;
        double smoothT = cubicEaseInOut(t);
        return startPoint + (direction * smoothT * std::abs(endPoint - startPoint));
    }
}

void moveJoint2(double currentTime, double startTime, double endTime, double startPoint, double endPoint, double& result) {
    if (currentTime < startTime || currentTime > endTime) {
        return;
    } else {
        double t = (currentTime - startTime) / (endTime - startTime);
        double direction = (endPoint >= startPoint) ? 1.0 : -1.0;
        double smoothT = cubicEaseInOut(t);
        result = startPoint + (direction * smoothT * std::abs(endPoint - startPoint));
    }
}


void* logging( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    usleep(1000000);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(true)
    {
        handle_start_of_periodic_task(next);
        while(!log_data_ready || !sim_step_completed) usleep(50);
        log_data_ready = false;
        sim_step_completed = false;

        if(t_n_FSM_out(0) != last_log_time){
            last_log_time = t_n_FSM_out(0);
            // logging:
            dash_utils::writeVectorToCsv(x_out,"x.csv");
            dash_utils::writeVectorToCsv(u_out,"u.csv");
            dash_utils::writeVectorToCsv(tau_out,"tau.csv");
            dash_utils::writeVectorToCsv(full_tau_out,"full_sw_st_tau.csv");
            dash_utils::writeVectorToCsv(tau_ext_out,"tau_ext.csv");

            dash_utils::writeVectorToCsv(q_out,"q.csv");
            dash_utils::writeVectorToCsv(qd_out,"qd.csv");

            dash_utils::writeVectorToCsv(lfv_out,"lfv.csv");
            dash_utils::writeVectorToCsv(lfdv_out,"lfdv.csv");

            dash_utils::writeVectorToCsv(lfv_comm_out,"lfv_comm.csv");
            dash_utils::writeVectorToCsv(lfdv_comm_out,"lfdv_comm.csv");

            dash_utils::writeVectorToCsv(t_n_FSM_out,"t_and_FSM.csv");

            dash_utils::writeHumanDynDataToCsv(hdd_out,"human_dyn_data.csv");
            dash_utils::writeTrajPlannerDataToCsv(tpdd_out,"traj_planner_dyn_data.csv");

            dash_utils::writeVectorToCsv(impulse_out,"external_forces.csv");

            dash_utils::writeVectorToCsv(meas_grf_out,"meas_grf_out.csv");

            dash_utils::writeVectorToCsv(xDCM_out,"xDCM.csv");

            VectorXd target_data(6);
            target_data << target_pos_out, target_vel_out;
            dash_utils::writeVectorToCsv(target_data,"target_pos.csv");
            
        }
        // end logging
        usleep(50);
		// handle_end_of_periodic_task(next,period);
	}
    cout << "Human Playback Complete" << endl;
    return  0;
}
int recording_cnt = 0;
void* screenRecord( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    std::string cnt_str;

    usleep(1000000);
    std::string text = executeCommand("xwininfo -root -tree | grep \"Tello Mujoco\"");
    size_t paren_position = text.find(')');
    text = text.substr(paren_position+1);
    cout << "TEXT: " << text << endl;
    std::string win_pos;
    std::string win_size;

    std::regex sizeRegex(R"(\b(\d+x\d+)\b)");
    std::regex posRegex(R"( \+(\d+)\+(\d+))");

    std::smatch sizeMatch, posMatch;

    if (std::regex_search(text, sizeMatch, sizeRegex)) {
        win_size = sizeMatch[1].str();
        std::cout << "win_size: " << win_size << std::endl;
    }

    if (std::regex_search(text, posMatch, posRegex)) {
        win_pos = posMatch[1].str() + "," + posMatch[2].str();
        std::cout << "win_pos: " << win_pos << std::endl;
    }

    if(sim_conf.en_auto_record)
        screen_recording = true;

    // screen_rec_pid = fork();
    while(1)
    {

        while(!screen_recording || recording_in_progress || !(sim_conf.en_screen_recording)) usleep (1000);
        // screen_rec_pid = fork();
        if (!recording_in_progress) {

            std::string text = executeCommand("xwininfo -root -tree | grep \"Tello Mujoco\"");
            size_t paren_position = text.find(')');
            text = text.substr(paren_position+1);
            cout << "TEXT: " << text << endl;
            std::string win_pos;
            std::string win_size;

            std::regex sizeRegex(R"(\b(\d+x\d+)\b)");
            std::regex posRegex(R"( \+(\d+)\+(\d+))");

            std::smatch sizeMatch, posMatch;

            if (std::regex_search(text, sizeMatch, sizeRegex)) {
                win_size = sizeMatch[1].str();
                std::cout << "win_size: " << win_size << std::endl;
            }

            if (std::regex_search(text, posMatch, posRegex)) {
                win_pos = posMatch[1].str() + "," + posMatch[2].str();
                std::cout << "win_pos: " << win_pos << std::endl;
            }

            // Parsing str1
            std::istringstream iss1(win_size);
            std::string token1;
            int size_x, size_y;

            std::getline(iss1, token1, 'x');
            size_x = std::stoi(token1);

            std::getline(iss1, token1);
            size_y = std::stoi(token1)+72;

            // Parsing str2
            std::istringstream iss2(win_pos);
            std::string token2;
            int pos_x, pos_y;

            std::getline(iss2, token2, ',');
            pos_x = std::stoi(token2);

            std::getline(iss2, token2);
            pos_y = std::stoi(token2)-72;

            std::ostringstream oss1;
            oss1 << size_x << 'x' << size_y;
            std::string window_size = oss1.str();

            std::ostringstream oss2;
            oss2 << pos_x << ',' << pos_y;
            std::string window_pos = oss2.str();

            cout << "pos x: " << pos_x << "   pos y: " << pos_y << "   size x: " << size_x << "   size y: " << size_y << endl;
            recording_in_progress = true;
            // Child process - execute FFmpeg command
            cnt_str = to_string(recording_cnt);
            //execl("/usr/bin/ffmpeg", "ffmpeg", "-f", "x11grab", "-video_size", "3840x2160", /*"-loglevel", "quiet",*/ "-framerate", "30", "-i", ":1+eDP-1-1", "-c:v", "h264_nvenc", "-qp", "0", "ScreenCapture.mp4", NULL);
            //cout << "recording to: " << log_folder+"ScreenCapture_"+cnt_str+".mp4" << endl;
            system(("taskset -c 14 ffmpeg -f x11grab -video_size "+window_size+" -loglevel quiet -framerate 30 -i $DISPLAY+"+window_pos+" -c:v h264_nvenc -qp 0 " + log_folder+"ScreenCapture_"+cnt_str+".mp4").c_str());
            recording_cnt++;
        }
        usleep(10000);
    }

    return  0;
}
int usb_recording_cnt = 0;
void* usbCamRecord( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
    std::string cnt_str;
    // screen_rec_pid = fork();
    usleep(1000000);
    if(sim_conf.en_auto_record)
        usbcam_recording = true;
    while(1)
    {

        while(!usbcam_recording || usb_recording_in_progress || !(sim_conf.en_HMI_recording)) usleep (1000);
        // screen_rec_pid = fork();
        if (!usb_recording_in_progress) {
            usb_recording_in_progress = true;
            // Child process - execute FFmpeg command
            cnt_str = to_string(usb_recording_cnt);
            system(("taskset -c 15 ffmpeg -f v4l2 -loglevel quiet -framerate 30 -video_size 800x600 -input_format mjpeg -i /dev/video4 -c:v copy " + log_folder+"usb_camera_"+cnt_str+".mp4").c_str());
            usb_recording_cnt++;
        }
        usleep(10000);
    }

    return  0;
}