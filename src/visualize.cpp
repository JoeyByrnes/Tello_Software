#include "mujoco_main.h"
#include "mujoco_utilities.h"

// MuJoCo data structures
extern mjModel* m;                  // MuJoCo model
extern mjData* d;                   // MuJoCo data
extern mjvCamera cam;               // abstract camera
extern mjvOption opt;               // visualization options
extern mjvScene scn;                // abstract scene
extern mjrContext con;              // custom GPU context

extern GLFWwindow* window;
extern int windowWidth, windowHeight;
extern float separator_thickness;

extern FILE* screen_record_pipe;
extern pid_t screen_rec_pid;
extern bool recording_in_progress;
extern bool usb_recording_in_progress;

extern struct termios originalSettings;

extern bool pause_sim;
extern std::string log_folder;
extern simConfig sim_conf;

extern double vx_desired_ps4;
extern double vy_desired_ps4;
extern bool PS4_connected;
extern bool zero_human;
extern float master_gain;
extern bool screen_recording;
extern bool usbcam_recording;
extern bool usbcam_hw_recording;
extern bool en_v2_ctrl;
extern bool en_safety_monitor;
extern bool bookmarked;
extern bool showCopyErrorPopup;
extern bool init_foot_width;
extern bool playback_error;
extern userProfile activeUser;
extern int hdd_cnt; // human_playback counter
extern std::string active_playback_log;
extern std::string active_playback_log_png;
extern int active_playback_log_index;
extern std::vector<std::string> hddFiles;
extern bool showPlotMenu;
extern bool showTuningMenu;
extern bool playback_changed;
extern bool playback_chosen;
extern bool auto_mode;
extern bool start_target_motion;
extern bool ramp_toggle;
extern bool controller_unstable;
extern int video_pulse_indicator_cnt;
extern char error[1000];
extern char notes[100];
extern double robot_init_foot_width;
extern double human_x_zero;
extern bool simulation_ready_to_run;

extern int sockfd_tx;
extern char hmi_tx_buffer[100];
extern struct sockaddr_in servaddr_tx;

extern VectorXd x0;
extern MatrixXd q0;
extern std::string recording_file_name;

extern pthread_mutex_t plotting_mutex;
extern pthread_mutex_t sim_mutex;
extern pthread_mutex_t sim_step_mutex;
extern pthread_mutex_t tello_ctrl_mutex;
extern pthread_mutex_t tau_share_mutex;

extern HW_CTRL_Data hw_control_data;

void SerializeVizControlData(const HW_CTRL_Data& data, uint8_t* buffer, size_t bufferSize) {
    if (bufferSize < sizeof(HW_CTRL_Data)) {
        // Handle buffer size error
        return;
    }

    memcpy(buffer, &data, sizeof(HW_CTRL_Data));
}

void* visualize_robot( void * arg )
{
    pause_sim = false;
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);

    usleep(1000000);
    Eigen::VectorXd curr_x(21);
    Eigen::VectorXd curr_q(10);

    //setup
	const char *localIP = "192.168.1.2"; // This PC's IP
    const int localPort = VIZ_UDP_RECEIVE_PORT;
    const int bufferSize = 152;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create socket." << std::endl;
    }

    sockaddr_in localAddr;
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(localPort);
    localAddr.sin_addr.s_addr = inet_addr(localIP);

    if (bind(sockfd, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        close(sockfd);
    }


    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while(1)
    {
        socklen_t * len1;
        int len, n;
        VisualizationData vd;

        char buffer[bufferSize];
		n = recvfrom(sockfd, buffer, bufferSize, 0, nullptr, nullptr);
        memcpy(&vd, buffer, sizeof(vd));

        // cout << "x: " << vd.CoM_pos_measured[0] << "   y: " << vd.CoM_pos_measured[1] << "   z: " << vd.CoM_pos_measured[2] << endl;
        // Get robot states
        // pthread_mutex_lock(&sim_mutex);
        // pthread_mutex_lock(&sim_step_mutex);
        d->mocap_pos[0] = vd.CoM_pos_measured[0];
        d->mocap_pos[1] = vd.CoM_pos_measured[1];
        d->mocap_pos[2] = vd.CoM_pos_measured[2];

        Eigen::Quaterniond quat;
        quat =  Eigen::AngleAxisd(M_PI - vd.CoM_rpy_measured[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(-vd.CoM_rpy_measured[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(vd.CoM_rpy_measured[0], Eigen::Vector3d::UnitX());

        d->mocap_quat[0] = quat.z();
        d->mocap_quat[1] = quat.y();
        d->mocap_quat[2] = quat.x();
        d->mocap_quat[3] = quat.w();

        d->qpos[hip_yaw_l_idx] = vd.q_measured[0];
        d->qpos[hip_roll_l_idx] = vd.q_measured[1];
        d->qpos[hip_pitch_l_idx] = vd.q_measured[2];
        d->qpos[knee_pitch_l_idx] = vd.q_measured[3];
        d->qpos[ankle_pitch_l_idx] = vd.q_measured[4];
        d->qpos[hip_yaw_r_idx] = vd.q_measured[5];
        d->qpos[hip_roll_r_idx] = vd.q_measured[6];
        d->qpos[hip_pitch_r_idx] = vd.q_measured[7];
        d->qpos[knee_pitch_r_idx] = vd.q_measured[8];
        d->qpos[ankle_pitch_r_idx] = vd.q_measured[9];

        d->mocap_pos[3] = vd.CoM_pos_desired[0];
        d->mocap_pos[4] = vd.CoM_pos_desired[1];
        d->mocap_pos[5] = vd.CoM_pos_desired[2];

        quat =  Eigen::AngleAxisd(M_PI - vd.CoM_rpy_desired[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(-vd.CoM_rpy_desired[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(vd.CoM_rpy_desired[0], Eigen::Vector3d::UnitX());

        d->mocap_quat[4] = quat.z();
        d->mocap_quat[5] = quat.y();
        d->mocap_quat[6] = quat.x();
        d->mocap_quat[7] = quat.w();

        int jointID = mj_name2id(m, mjOBJ_JOINT, "left_anklev");

        // std::cout << "Joint ID for left_anklev: " << jointID << std::endl;

        d->qpos[hip_yaw_l_idx_viz] = vd.q_desired[0];
        d->qpos[hip_roll_l_idx_viz] = vd.q_desired[1];
        d->qpos[hip_pitch_l_idx_viz] = vd.q_desired[2];
        d->qpos[knee_pitch_l_idx_viz] = vd.q_desired[3];
        d->qpos[ankle_pitch_l_idx_viz] = vd.q_desired[4];
        d->qpos[hip_yaw_r_idx_viz] = vd.q_desired[5];
        d->qpos[hip_roll_r_idx_viz] = vd.q_desired[6];
        d->qpos[hip_pitch_r_idx_viz] = vd.q_desired[7];
        d->qpos[knee_pitch_r_idx_viz] = vd.q_desired[8];
        d->qpos[ankle_pitch_r_idx_viz] = vd.q_desired[9];


        //CoP visualization:
        d->mocap_pos[6] = vd.CoP[0];    // CoP viz X
        d->mocap_pos[7] = vd.CoP[1];  // CoP viz Y 

        handle_end_of_periodic_task(next,period);      
        
    }
   
    return  0;
}

void* visualization_render_thread( void * arg )
{
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* dynamic_robot_ptr = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);

	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(dynamic_robot_ptr);
	// Print the core and priority of the thread
	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("Mujoco thread running on core %d, with priority %d\n", core, priority);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    log_folder = createLogFolder("/home/joey/Desktop/tello_outputs/Logs/");
    dash_utils::setOutputFolder(log_folder);

    sim_conf = readSimConfigFromFile("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/sim_config.json");

    std::vector<userProfile> profiles;
    std::string active_user;
    readProfilesFromJson("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/user_profiles.json",profiles, active_user);
    for(int i=0;i<profiles.size();i++)
    {
        if(active_user.compare(profiles[i].name) == 0)
        {
            activeUser = profiles[i];
            Human_params human_params = tello->controller->get_human_params();
            human_params.hLIP = activeUser.lip_height;
            human_params.m = activeUser.weight;
        }
    }

    hw_control_data.hip_offset_left = 0;
    hw_control_data.hip_offset_right = 0;

    hw_control_data.knee_offset_left = 0;
    hw_control_data.knee_offset_right = 0;

    hw_control_data.ankle_offset_left = 0;
    hw_control_data.ankle_offset_right = 0;


    // INITIALIZE SRBM CONTROLLER ========================================================

    initializeSRBMCtrl();

    dash_utils::writeSRBParamsToTxt(tello->controller->get_SRB_params(),"srb_params.csv");
    dash_utils::writeHumanParamsToTxt(tello->controller->get_human_params(),"human_params.csv");

    // BEGIN SETUP CODE FOR MUJOCO ======================================================================
    
    mj_activate("./lib/Mujoco/mjkey.txt");

    m = mj_loadXML("../../../lib/Mujoco/model/tello/tello-hardware-visualization.xml", NULL, error, 1000);
       
	if (!m)
    {
        printf('r',"%s\n", error);
        exit(1);
    }
	// make data
    d = mj_makeData(m);

    // initializeLegs();

    // install control callback
    // mjcb_control = dummy_callback;//TELLO_locomotion_ctrl;

	// init GLFW
    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

		// create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1920, 1080, "Tello Mujoco Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    // cam.elevation = -18;
    // cam.distance = 1.8;
    // cam.azimuth = 135;
    // cam.lookat[0] = 0;
    // cam.lookat[1] = 0;
    // cam.lookat[2] = -0.2;

    cam.elevation = -18;
    cam.distance = 2.0;
    cam.azimuth = -0;
    cam.lookat[0] = 0;
    cam.lookat[1] = 0;
    cam.lookat[2] = -0.1;

    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetWindowCloseCallback(window,window_close_callback);

    m->opt.timestep = 0.002;

    // set up UDP transmit here: =======================================================================
	// Creating socket file descriptor
	if ( (sockfd_tx = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
    cout << "UDP TRANSMIT SOCKET CREATED" << endl;
	memset(&servaddr_tx, 0, sizeof(servaddr_tx));
	// Filling server information
	servaddr_tx.sin_family = AF_INET;
	servaddr_tx.sin_port = htons(UDP_RECEIVE_PORT);
	servaddr_tx.sin_addr.s_addr = inet_addr(TELLO_IP_ADDRESS);
	// ens UDP setup here: =============================================================================

    // Plotting:

	// END SETUP CODE FOR MUJOCO ========================================================================
    pthread_mutex_lock(&sim_mutex);
    mj_forward(m, d);
    pthread_mutex_unlock(&sim_mutex);
    simulation_ready_to_run = true;
    // initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::StyleColorsDark();


    float baseFontSize = 60.0f; // 13.0f is the size of the default font. Change to the font size you use.
    float iconFontSize = baseFontSize * 2.25f / 3.0f; // FontAwesome fonts need to have their sizes reduced by 2.0f/3.0f in order to align correctly


    ImFont* font;
    ImFont* fontSmall;
    if (std::filesystem::is_directory("/home/tello")) {
        //std::cout << "The directory /home/tello exists!" << std::endl;
        font = io.Fonts->AddFontFromFileTTF("./tello_files/fonts/roboto/Roboto-Light.ttf", baseFontSize);
    }
    else {
        //std::cout << "The directory /home/tello does not exist!" << std::endl;
        fontSmall = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/roboto/Roboto-Light.ttf", baseFontSize/1.75);
        font = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/roboto/Roboto-Light.ttf", baseFontSize);
        // font = io.Fonts->AddFontFromFileTTF("../../../lib/imGUI/fonts/seguisym.ttf", 80);
        
    }
    
    // merge in icons from Font Awesome
    static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_16_FA, 0 };
    ImFontConfig icons_config; 
    icons_config.MergeMode = true; 
    icons_config.PixelSnapH = true; 
    icons_config.GlyphMinAdvanceX = iconFontSize;
    io.Fonts->AddFontFromFileTTF( "../../../lib/imGUI/fonts/fa-solid-900.ttf", iconFontSize, &icons_config, icons_ranges );


    // create a vector of all the plot pngs for selecting playback option
    std::string plotfolderPath = "/home/joey/Desktop/tello_outputs/Hardware_Motion_Library/Plots";
    std::vector<std::string> pngFiles;
    std::vector<GLuint> image_textures;
    std::vector<std::string> image_names;
    GLuint active_texture;
    int im_width = 888;
    int im_height = 500;

    for (const auto& entry : std::filesystem::directory_iterator(plotfolderPath)) {
        if (entry.path().extension() == ".png") {
            std::string name = entry.path().filename().string();
            pngFiles.push_back((plotfolderPath+"/"+name));
            GLuint tex = 0;
            LoadTextureFromFile((plotfolderPath+"/"+name).c_str(), &tex, &im_width, &im_height);
            image_textures.push_back(tex);
            image_names.push_back(name.substr(0, name.length() - 4));
            hddFiles.push_back("/home/joey/Desktop/tello_outputs/Hardware_Motion_Library/"+name.substr(0, name.length() - 4)+"/human_dyn_data.csv");
        }
    }

    // // Print the filenames
    // for (const auto& filename : hddFiles) {
    //     std::cout << filename << std::endl;
    // }

    // dash_utils::start_sim_timer();

	while(!glfwWindowShouldClose(window))
    {
        handle_start_of_periodic_task(next);
		// BEGIN LOOP CODE FOR MUJOCO ===================================================================

        hw_control_data.tare_hmi = false;
        hw_control_data.start_legs = false;
        hw_control_data.balance = false;
        hw_control_data.emergency_stop = false;
        // hw_control_data.enable_teleop = false;
        hw_control_data.start_dcm_tracking = false;
        // hw_control_data.set_full_joint_kp = false;
        
        // set local tello object here:
        pthread_mutex_lock(&tello_ctrl_mutex);
        RoboDesignLab::DynamicRobot* telloLocal = new RoboDesignLab::DynamicRobot(*tello);
        pthread_mutex_unlock(&tello_ctrl_mutex);


        std::string text = "Tello Realtime Visualization of Hardware";
        glfwSetWindowTitle(window, text.c_str());

        // get framebuffer viewport
        mjrRect viewport = { 0, 0, 0, 0 };
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        cam.lookat[0] = d->qpos[torso_x_idx];
        cam.lookat[1] = d->qpos[torso_y_idx];
        
        // set the background color to white
        
        // update scene and render
        pthread_mutex_lock(&sim_mutex);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        pthread_mutex_unlock(&sim_mutex);
        mjr_render(viewport, &scn, &con);
        
        // render ImGui GUI
        ImVec4 dark_navy = hex2ImVec4(0x082032);
        ImVec4 med_navy = hex2ImVec4(0x2C394B);
        ImVec4 light_navy = hex2ImVec4(0x334756);
        ImVec4 lighter_navy = hex2ImVec4(0x54748c);
        
        ImVec4 grey1 = hex2ImVec4(0xd9d9d9);
        ImVec4 grey2 = hex2ImVec4(0xadadad);
        ImVec4 grey3 = hex2ImVec4(0x7d7d7d);
        ImVec4 grey4 = hex2ImVec4(0x424242);
        ImVec4 grey5 = hex2ImVec4(0x1f1f1f);

        ImVec4 black = hex2ImVec4(0x000000);
        ImVec4 white = hex2ImVec4(0xFFFFFF);
        ImVec4 blue1 = hex2ImVec4(0x0d75bf);
        ImVec4 red = hex2ImVec4(0xb51021);
        ImVec4 redHover = hex2ImVec4(0xd92133);
        ImVec4 redActive = hex2ImVec4(0x7a0b16);

        glfwGetWindowSize(window, &windowWidth, &windowHeight);
        double screenScale = ((double)windowWidth/3840.0)*1.2;
        if(screenScale > 1.0) screenScale = 1.0;
        io.FontGlobalScale = screenScale;
        separator_thickness = 15.0*screenScale;
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::PushFont(font);
        ImGui::PushStyleColor(ImGuiCol_MenuBarBg, grey5);
        ImGui::SetNextWindowSizeConstraints(ImVec2(-1, 35+60.0*screenScale), ImVec2(-1, FLT_MAX));
        ImGui::BeginMainMenuBar();
        ImGui::EndMainMenuBar();
        ImGui::PopStyleColor();
        ImGui::SetNextWindowPos(ImVec2(0, 15));
        ImGui::SetNextWindowSizeConstraints(ImVec2(-1, 10*60.0*screenScale), ImVec2(-1, FLT_MAX));
        // add toolbar with button
        ImGui::PushStyleColor(ImGuiCol_MenuBarBg, grey4);
        ImGui::PushStyleColor(ImGuiCol_Button, med_navy);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, light_navy);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, lighter_navy);
        
        ImGui::BeginMainMenuBar();

        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();

        bool dummy;
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);        
        if (ImGui::BeginMenu(" " ICON_FA_BARS " "))
        {
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_FILE_CSV "  Enable Data Logging   ", &(hw_control_data.enable_datalogging));
            ImGui::Separator();
            // ImGui::Checkbox(" " ICON_FA_PLAY "  Enable Playback Mode   ", &(sim_conf.en_playback_mode));
            // ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_DICE_TWO "  Enable V2 Controller   ", &(hw_control_data.enable_v2_controller));
            en_v2_ctrl = hw_control_data.enable_v2_controller;
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_USER_ALT_SLASH "  Boot to Auto Mode (Restart Required)   ", &(hw_control_data.auto_mode));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            // std::string human_label;
            // if(sim_conf.en_playback_mode) human_label = " " ICON_FA_CHILD "  Enable HMI Playback   ";
            // else human_label = " " ICON_FA_CHILD "  Enable HMI Communication   ";
            // ImGui::Checkbox(human_label.c_str(), &(sim_conf.en_human_control));
            // tello->controller->enable_human_dyn_data = sim_conf.en_human_control;
            // ImGui::Separator();  
            ImGui::Checkbox(" " ICON_FA_SIGN_IN_ALT "  Enable X Haptic Force   ", &(hw_control_data.enable_x_force));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_ARROWS_ALT_H "  Enable Force Feedback   ", &hw_control_data.enable_force_feedback);      // Edit bools storing our window open/close state
            ImGui::Separator();//init_foot_width
            ImGui::Checkbox(" " ICON_FA_HARD_HAT "  Enable Safety Monitor   ", &hw_control_data.enable_safety_monitor);      // Edit bools storing our window open/close state
            ImGui::Separator();
            // ImGui::Checkbox(" " ICON_FA_GAMEPAD "  Enable PS4 Controller   ", &sim_conf.en_ps4_controller);      // Edit bools storing our window open/close state
            // ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_CAMERA "  Enable HMI Recording   ", &(sim_conf.en_HMI_recording));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_TV "  Enable Mujoco Recording   ", &(sim_conf.en_screen_recording));
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_VIDEO "  Auto-Record Sessions   ", &(sim_conf.en_auto_record));
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            ImGui::Checkbox(" " ICON_FA_LAPTOP_CODE "  Show Live Variable View  ", &(sim_conf.en_live_variable_view));
            ImGui::Separator();
            // ImGui::Checkbox(" " ICON_FA_SLIDERS_H "  Show Full HMI Controls   ", &(sim_conf.en_full_hmi_controls));
            // ImGui::Separator();
            // ImGui::Checkbox(" " ICON_FA_CHART_LINE "  Display Realtime Plot   ", &(sim_conf.en_realtime_plot));
            // ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);  
            ImGui::Separator();
            if (ImGui::MenuItem(" " ICON_FA_SAVE "  Save Configuration"))
            { 
                writeSimConfigToFile(sim_conf, "/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/sim_config.json");
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();
        char notes[1001]; // String variable to store the entered notes
        
        if (ImGui::BeginMenu((" " ICON_FA_USER " " + activeUser.name + " ").c_str()))
        {
            ImGui::Separator();
            ImGui::Text(" Select User Profile:   ");
            ImGui::Separator();
            for(int p=0;p<profiles.size();p++)
            {
                userProfile user = profiles[p];
                ImGui::SetNextItemWidth(300);
                if(ImGui::Button((" " + user.name + " ").c_str(),ImVec2(500, 0)))
                {
                    activeUser = profiles[p];
                    updateActiveUserInJson("/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/user_profiles.json",activeUser.name);
                    initializeSRBMCtrl();
                }
                ImGui::Separator();
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        ImGui::Separator();
        if(bookmarked)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.84f, 0.0f, 1.0f)); // Set text color to gold
        }
        if (ImGui::BeginMenu(" " ICON_FA_STAR " "))
        {
            if(bookmarked) ImGui::PopStyleColor();
            ImGui::Separator();
            if(!pause_sim || screen_recording || usbcam_recording || usbcam_hw_recording)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // Set button color to grey
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // Set button color to grey
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.4f, 0.4f, 0.4f, 1.0f)); // Set button color to grey
                ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f); // Dim the button
                ImGui::Button(" " ICON_FA_COPY " Copy log folder to favorites ");
                ImGui::PopStyleVar();
                ImGui::PopStyleColor(3);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Please stop simulation and recording to copy"); // Display tooltip on hover
                }
            }
            else
            {
                if (ImGui::Button(" " ICON_FA_COPY " Copy log folder to favorites ")) // Submit button
                {
                    if(pause_sim && !screen_recording && !usbcam_recording && !usbcam_hw_recording)
                    {
                        std::string command = "cp -R " + log_folder + " /home/joey/Desktop/tello_outputs/Favorite_Logs/";
                        system(command.c_str());
                    }
                    else
                    {
                        showCopyErrorPopup = true;
                    }
                }
            }
            ImGui::Separator();
            if(bookmarked)
            {
                if (ImGui::Button(" " ICON_FA_TIMES " Undo Bookmark                    ")) // Submit button
                {
                    bookmarked = false;
                    dash_utils::deleteLogFile("bookmarked.txt");
                }
            }
            else
            {
                if (ImGui::Button(" " ICON_FA_BOOKMARK " Bookmark log folder             ")) // Submit button
                {
                    bookmarked = true;
                    dash_utils::writeStringToFile("","bookmarked.txt");
                }
            }
            ImGui::Separator();
            ImGui::EndMenu();
        }
        
        else if(bookmarked) ImGui::PopStyleColor();

        ImGui::Separator();
        if (ImGui::BeginMenu(" " ICON_FA_PENCIL_ALT " "))
        {
            ImGui::Separator();
            ImGui::Separator();
            ImGui::AlignTextToFramePadding(); // Align label to top
            ImGui::Text("  Notes  "); // Label text
            ImGui::SameLine(); // Place the text entry box next to the label
            ImGui::InputText("  ", notes,1000); // Text entry box for notes                                                                   ");
            ImGui::SameLine();
            if (ImGui::Button(" Save ")) // Submit button
            {
                std::string submittedNotes = notes;
                dash_utils::writeStringToFile(submittedNotes+"\n\n","notes.txt");
                notes[0] = '\0';
            }
            ImGui::SameLine();
            ImGui::Text("  ");
            ImGui::Separator();
            ImGui::Separator();
            ImGui::EndMenu();
        }

        ImGui::PopStyleColor();
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();

        if(pause_sim)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.4f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.0f, 0.6f, 0.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.0f, 0.8f, 0.0f, 1.0f));
            if (ImGui::Button(" " ICON_FA_PLAY_CIRCLE " ")) {
                pause_sim = false;
            }
            ImGui::PopStyleColor(3);
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.0f, 0.45f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.65f, 0.0f, 0.65f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.25f, 0.0f, 0.25f, 1.0f));
            if (ImGui::Button(" " ICON_FA_PAUSE_CIRCLE " ") || controller_unstable) {
                pause_sim = true;
                master_gain = 0.0;
                tello->controller->disable_human_ctrl();
                // screen_recording = false;
                // usbcam_recording = false;
                // system("killall -2 ffmpeg");
                
                recording_in_progress = false;
                usb_recording_in_progress = false;
            }
            ImGui::PopStyleColor(3);
        }
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::Separator();
        ImGui::PopStyleColor();
        if(!screen_recording)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, red);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redActive);
            if (ImGui::Button(" " ICON_FA_VIDEO " ")) {
                screen_recording = true;
                usbcam_recording = true;
                usbcam_hw_recording = true;
            }
            ImGui::PopStyleColor(3);
        }
        else
        {
            double color_offset = 20 * std::sin(video_pulse_indicator_cnt / 1000.0);  // Adjust frequency by dividing counter
            video_pulse_indicator_cnt +=5;
            if(video_pulse_indicator_cnt > 6283) video_pulse_indicator_cnt = 0;
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4((181.0f+color_offset) / 255.0f, 16.0f / 255.0f, 33.0f / 255.0f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHover);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, redActive);
            if (ImGui::Button(" " ICON_FA_VIDEO_SLASH " ")) {
                screen_recording = false;
                usbcam_recording = false;
                usbcam_hw_recording = false;
                system("killall -2 ffmpeg");
                
                recording_in_progress = false;
                usb_recording_in_progress = false;
                
                usleep(2000);
            }
            ImGui::PopStyleColor(3);
        }
        
        ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
        ImGui::PushStyleColor(ImGuiCol_CheckMark,black);
        ImGui::PushStyleColor(ImGuiCol_FrameBg,grey1);
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,white);
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,grey2);

        ImGui::Separator();
        if(!(sim_conf.en_autonomous_mode_on_boot))
        {
            // ImGui::Separator();
            // ImGui::Checkbox(" Real-Time   ", &realtime_enabled);
            if(sim_conf.en_full_hmi_controls || sim_conf.en_playback_mode){
                

                if(!(sim_conf.en_playback_mode))
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                    if (ImGui::Button(" " ICON_FA_WEIGHT " Tare HMI  ")) {
                        hw_control_data.tare_hmi = true;
                    }
                    ImGui::PopStyleColor(3);
                }
                
                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_PLAY " Start Legs  " )) {
                    hw_control_data.start_legs = true;
                }
                ImGui::PopStyleColor(3);

                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_ARROW_ALT_CIRCLE_UP " Kp  " )) {
                    hw_control_data.set_full_joint_kp = true;
                    hw_control_data.set_min_joint_kp = false;
                }
                ImGui::PopStyleColor(3);

                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_ARROW_ALT_CIRCLE_DOWN " Kp  " )) {
                    hw_control_data.set_min_joint_kp = true;
                    hw_control_data.set_full_joint_kp = false;
                }
                ImGui::PopStyleColor(3);


                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_MALE " Balance  ")) {
                    hw_control_data.balance = true;
                }
                ImGui::PopStyleColor(3);

                ImGui::Separator();//init_foot_width
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button(" " ICON_FA_ANGLE_RIGHT " Track DCM  " )) {
                    hw_control_data.start_dcm_tracking = true;
                }
                ImGui::PopStyleColor(3);

                ImGui::Separator();//init_foot_width
                
                if (!hw_control_data.enable_teleop)
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                    if (ImGui::Button(" " ICON_FA_PEOPLE_ARROWS " Teleop  ")) {
                        hw_control_data.enable_teleop = true;
                    }
                }
                else
                {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.45f, 0.0f, 0.45f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.65f, 0.0f, 0.65f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.25f, 0.0f, 0.25f, 1.0f));
                    if (ImGui::Button(" " ICON_FA_TIMES "  Teleop  ")) {
                        hw_control_data.enable_teleop = false;
                    }
                }

                ImGui::PopStyleColor(3);

                ImGui::Separator();
                if(!(sim_conf.en_playback_mode))
                {
                    ImGui::SetNextItemWidth(200.0f*screenScale);
                    ImGui::PushStyleColor(ImGuiCol_FrameBg,grey2);
                    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,grey2);
                    ImGui::PushStyleColor(ImGuiCol_SliderGrab,black);
                    ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,black);
                    ImGui::SliderFloat(" HMI Gain", &hw_control_data.hmi_gain, 0.0f, 1.0f);
                    ImGui::Separator();
                    ImGui::PopStyleColor(4);
                }
                ImGui::PushStyleColor(ImGuiCol_Button, red);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, redHover);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, redActive);
                if (ImGui::Button("  " ICON_FA_TIMES_CIRCLE " E-STOP   ")) {
                    hw_control_data.emergency_stop = true;
                }
                ImGui::PopStyleColor(3);
                ImGui::Separator();//init_foot_width
            }
            else{
                ImGui::PushStyleColor(ImGuiCol_Button, light_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, lighter_navy);
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, med_navy);
                if (ImGui::Button("  " ICON_FA_RULER " Initialize Human & Robot   ")) {
                    init_foot_width = true;
                    MatrixXd new_lfv0(4,3);
                    MatrixXd new_q0(2,5);
                    dash_init::init_foot_width(robot_init_foot_width,telloLocal->controller->get_human_params(),
                                                telloLocal->controller->get_SRB_params(),new_lfv0,q0);
                    Traj_planner_dyn_data tpdd_foot_z = telloLocal->controller->get_traj_planner_dyn_data();
                    tpdd_foot_z.step_z_offset_R = telloLocal->controller->get_human_dyn_data().fzH_R;
                    tpdd_foot_z.step_z_offset_L = telloLocal->controller->get_human_dyn_data().fzH_L;
                    tello->controller->set_traj_planner_human_foot_z_offsets(tpdd_foot_z);
                    tello->controller->set_lfv0(new_lfv0);
                    tello->controller->set_q0(q0);
                    // initializeLegs();
                    master_gain = 1.0;
                    zero_human = true;
                    if(pause_sim)
                        mj_forward(m,d);
                }
                ImGui::PopStyleColor(3);
                ImGui::Separator();//init_foot_width
            }
            if(sim_conf.en_playback_mode)
            {
                if(!showPlotMenu)
                {
                    if (ImGui::Button(" " ICON_FA_PLAY " Show Playback Menu  ")) 
                    {
                        showPlotMenu = true;
                    }
                }
                else
                {
                    if (ImGui::Button(" " ICON_FA_PLAY " Hide Playback Menu  ")) 
                    {
                        showPlotMenu = false;
                    }
                }
            }
        }
        else{
            // ImGui::Separator();
            // ImGui::Checkbox(" Real-Time   ", &realtime_enabled);
            ImGui::Separator();
            ImGui::Text("Autonomous Mode");
        }
        
        ImGui::PopStyleColor(5);
        ImGui::EndMainMenuBar();
        // ImGui::PopFont();
        ImGui::PopStyleColor(4);

        // Load images
        // ImPlot::StyleColorsLight();
        if(showPlotMenu)
        {
            
            double plot_width = (double)windowWidth/5.0;
            double plot_height = plot_width*(9.0/16.0);
            double headerHeight = plot_height+120*screenScale+150*screenScale;
            ImGui::SetNextWindowSize(ImVec2((double)windowWidth/5.0,headerHeight));
            ImGui::SetNextWindowPos(ImVec2( (windowWidth - (double)windowWidth/5.0), (35+60*screenScale)) );
            ImGui::Begin("plotHeader",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

            if(ImGui::Button(" " ICON_FA_TIMES_CIRCLE " "))
            {
                showPlotMenu = false;
            }
            ImGui::SameLine();
            ImGui::Text("  Selected Log:");
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::PushFont(fontSmall);
            ImGui::Text((image_names[active_playback_log_index]).c_str());
            ImGui::PopFont();
            ImGui::Image((void*)(intptr_t)(image_textures[active_playback_log_index]), ImVec2(plot_width, plot_height));
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::Text("  Available Logs:");
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::Separator();
            ImGui::PopStyleColor();
            ImGui::End();

            ImGui::SetNextWindowSize(ImVec2((double)windowWidth/5.0,windowHeight-headerHeight-(35+60*screenScale)));
            ImGui::SetNextWindowPos(ImVec2( (windowWidth - (double)windowWidth/5.0), (35+60*screenScale+headerHeight)) );
            ImGui::Begin("plotSelect",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);
            ImGui::PushFont(fontSmall);
            for(int i=0;i<pngFiles.size();i++)
            {
                ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
                ImGui::Separator();
                ImGui::PopStyleColor();
                ImGui::Text((image_names[i]).c_str());
                if (ImGui::ImageButton((void*)(intptr_t)(image_textures[i]), ImVec2(plot_width, plot_height)))
                {
                    active_playback_log_png = pngFiles[i];
                    active_playback_log_index = i;
                    playback_changed = true;
                    playback_chosen = true;
                    writeActivePlaybackLog(hddFiles[i],"/home/joey/Documents/PlatformIO/Projects/Tello_Software/include/active_playback_log.json");
                }
            }
            ImGui::PopFont();
            
            ImGui::End();
        }
        if(showTuningMenu)
        {
            
            double tuning_width = (double)windowWidth*0.9;
            double tuning_height = windowHeight*0.9- (35+60*screenScale);
            ImGui::SetNextWindowSize(ImVec2(tuning_width,tuning_height));
            ImGui::SetNextWindowPos(ImVec2(windowWidth*0.05, (35+60*screenScale)+windowHeight*0.05));
            ImGui::Begin("plotHeader",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);

            if(ImGui::Button(" " ICON_FA_TIMES_CIRCLE " "))
            {
                showTuningMenu = false;
            }
            ImGui::SameLine();
            ImGui::Text("  Tuning Menu:");
            
            ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,grey5);
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::PushStyleColor(ImGuiCol_FrameBg,grey4);
            ImGui::PushStyleColor(ImGuiCol_FrameBgHovered,grey4);
            ImGui::PushStyleColor(ImGuiCol_SliderGrab,black);
            ImGui::PushStyleColor(ImGuiCol_SliderGrabActive,black);
            float f1,f2,f3,f4,f5,f6,f7,f8,f9,f0;
            ImGui::SliderFloat(" Kp1", &f1, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp2", &f2, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp3", &f3, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp4", &f4, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp5", &f5, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp6", &f6, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            ImGui::SetNextItemWidth(tuning_width*0.9);
            ImGui::SliderFloat(" Kp7", &f7, 0.0f, 1.0f);
            ImGui::Separator();ImGui::Separator();ImGui::Separator();
            
            ImGui::PopStyleColor(5);

            // Reset the ImGui style to its original value
            io.FontGlobalScale = screenScale;

            ImGui::End();
        }
        if(sim_conf.en_live_variable_view)
        {
            ImGui::SetNextWindowSize(ImVec2(800*screenScale, 5*60*screenScale + 5*15*screenScale +65*screenScale));
            ImGui::SetNextWindowPos(ImVec2(50, (35+60*screenScale)+50));
            ImGui::Begin("DebugView",nullptr,ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize);
            
            ImGui::Text(" Live Variable View:");
            ImGui::Separator();

            // ImGui::Text("FSM: %d", telloLocal->controller->get_FSM());
            // ImGui::Separator();
            // ImGui::Text("Next SSP: %d", telloLocal->controller->get_traj_planner_dyn_data().next_SSP);
            // ImGui::Separator();
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            //ImGui::Text("CoM X Velocity: %.2fm/s", telloLocal->controller->get_x()(3));
            // ImGui::SliderInt(" R Hip ", &hw_control_data.hip_offset_right, -150, 150);
            ImGui::SliderFloat(" roll: ", &hw_control_data.roll_adjust, -1.5, 1.5);
            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();
            // ImGui::Text("CoM X Position: %.2fm", telloLocal->controller->get_x()(0));
            // ImGui::SliderInt(" R Knee ", &hw_control_data.knee_offset_right, -150, 150);
            ImGui::SliderFloat(" pitch: ", &hw_control_data.pitch_adjust, -1.5, 1.5);

            ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            ImGui::Separator();
            ImGui::PopStyleColor();

            // ImGui::SliderInt(" R Ankle ", &hw_control_data.ankle_offset_right, -150, 150);
            ImGui::SliderFloat(" yaw: ", &hw_control_data.yaw_adjust, -1.5, 1.5);


            // ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            // ImGui::Separator();
            // ImGui::PopStyleColor();
            // //ImGui::Text("CoM X Velocity: %.2fm/s", telloLocal->controller->get_x()(3));
            // ImGui::SliderInt(" L Hip ", &hw_control_data.hip_offset_left, -150, 150);
            // // ImGui::SliderFloat(" roll: ", &hw_control_data.roll_adjust, -1.5, 1.5);
            // ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            // ImGui::Separator();
            // ImGui::PopStyleColor();
            // //ImGui::Text("CoM X Position: %.2fm", telloLocal->controller->get_x()(0));
            // ImGui::SliderInt(" L Knee ", &hw_control_data.knee_offset_left, -150, 150);
            // // ImGui::SliderFloat(" pitch: ", &hw_control_data.pitch_adjust, -1.5, 1.5);

            // ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4(0,0,0,0));
            // ImGui::Separator();
            // ImGui::PopStyleColor();

            // ImGui::SliderInt(" L Ankle ", &hw_control_data.ankle_offset_left, -150, 150);
            // // ImGui::SliderFloat(" yaw: ", &hw_control_data.yaw_adjust, -1.5, 1.5);
            
            ImGui::End();
        }
        
        // Create the popup dialog
        if (playback_error) {
            ImGui::OpenPopup("Playback Error");

            // Display the popup dialog
            if (ImGui::BeginPopupModal("Playback Error", &playback_error, ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::Text("Error Opening Playback File. Restart Program.");
                ImGui::Separator();

                if (ImGui::Button("Close"))
                {
                    playback_error = false;
                    ImGui::CloseCurrentPopup();
                }
                    

                ImGui::EndPopup();
            }
        }
        
        ImGui::PopFont();
        
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // swap OpenGL buffers (blocking call due to v-sync)
        
        glfwSwapBuffers(window);
        
        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();


        //handle UDP transmit here:
		Human_dyn_data hdd = telloLocal->controller->get_human_dyn_data();
        // if(sim_conf.en_safety_monitor)
        // {
        //     if( (fabs(hdd.FxH_hmi - last_Xf) > 100) || (fabs(hdd.FyH_hmi - last_Yf) > 100) || (fabs(hdd.FxH_spring - last_springf) > 100)){
        //         controller_unstable = true;
        //     }
        // }
        // // if( (fabs(hdd.FyH_hmi - last_Yf) > 100) ){
        // //     controller_unstable = true;
        // // }
        // last_Xf = FxH_hmi_out;
        // last_Yf = FyH_hmi_out;
        // last_springf = FxH_spring_out;

        uint8_t tx_buffer[sizeof(HW_CTRL_Data)];
        SerializeVizControlData(hw_control_data, tx_buffer, sizeof(tx_buffer));
        ssize_t send_result = sendto(sockfd_tx, tx_buffer, sizeof(tx_buffer), 0, (struct sockaddr *)&servaddr_tx, sizeof(servaddr_tx));
        // cout << "Sent " << (int)send_result << " bytes over UDP" << endl;

        // set tello data here:
        // pthread_mutex_lock(&tello_ctrl_mutex);
        // tello->controller->set_hmi_forces(hdd);
        // tello->controller->enable_human_dyn_data = sim_conf.en_human_control;
        // pthread_mutex_unlock(&tello_ctrl_mutex);
        
		// END LOOP CODE FOR MUJOCO =====================================================================
		handle_end_of_periodic_task(next,period);
        
	}
    screen_recording = false;
    usbcam_recording = false;
    usbcam_hw_recording = false;
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    system("killall -2 ffmpeg");
    system("pkill -f -9 ffmpeg");
    usleep(100000);
    tcsetattr(STDIN_FILENO, TCSANOW, &originalSettings);
	// //free visualization storage
    // mjv_freeScene(&scn);
    // mjr_freeContext(&con);
    // // free MuJoCo model and data, deactivate
    // mj_deleteData(d);
    // mj_deleteModel(m);
    // mj_deactivate();

    exit(0);
    return NULL;
}