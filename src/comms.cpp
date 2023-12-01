#include "comms.h"
#include "mujoco_main.h"
#include "user_config.h"

extern uint16_t encoders[10];

// extern CheetahMotor* motors[10];

extern int position_initialized[10];
extern uint16_t encoder_positions[10];
extern uint16_t encoder_offsets[10];

extern int udp_data_ready;
extern char udp_control_packet[UDP_MAXLINE];

extern pthread_mutex_t mutex_CAN_recv;
extern pthread_mutex_t mutex_UDP_recv;

extern HW_CTRL_Data hw_control_data;

extern int can_data_ready_to_save;

extern vn::math::vec3f tello_ypr;
extern bool calibrate_IMU_bias;
int cal_index = 0;
MatrixXd acc_cal(20000,3);
MatrixXd gyro_cal(20000,3);

double initial_yaw = 0;
bool yaw_offset_recorded = false;

long long print_index = 0;

extern simConfig sim_conf;
extern double robot_init_foot_width_HW;
double robot_init_foot_width = 0.175;
extern bool use_current_foot_width;

extern double xH_Commanded;

int r_h=0;
int r_k=0;
int r_a=0;
int l_h=0;
int l_k=0;
int l_a=0;

float roll_adjust = 0;
float pitch_adjust = 0;
float yaw_adjust = 0;


VectorXd fdxH_R_vec = VectorXd(100);
VectorXd fdyH_R_vec = VectorXd(100);
VectorXd fdzH_R_vec = VectorXd(100);
VectorXd fdxH_L_vec = VectorXd(100);
VectorXd fdyH_L_vec = VectorXd(100);
VectorXd fdzH_L_vec = VectorXd(100);

VectorXd grf_lf = VectorXd(100);
VectorXd grf_rf = VectorXd(100);
VectorXd grf_lb = VectorXd(100);
VectorXd grf_rb = VectorXd(100);

// Human_dyn_data_filter hdd_filter;


void* rx_CAN( void * arg ){
    
	TPCANStatus Status;
	TPCANMsg Message;
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
    void* arg1 = std::get<1>(*arg_tuple_ptr);
	void* arg0 = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);
    int pcd = *reinterpret_cast<int*>( arg1 );
	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(arg0);
	// Print the core and priority of the thread
	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("CAN channel %d rx thread running on core %d, with priority %d\n", pcd-64, core, priority);


    usleep(100);
	while(1){
		
		if (     !((Status=CAN_Read(pcd, &Message, NULL)) == PCAN_ERROR_QRCVEMPTY)     )//{;}
		{
			if (Status != PCAN_ERROR_OK) {
				//pthread_exit(NULL);
				//printf("Error on CAN bus %d\n",pcd-64);
				//break;
			}
			else
			{
				pthread_mutex_lock(&mutex_CAN_recv);
				uint8_t id = Message.DATA[0];
				if(id > 0 && id < 11){
					process_motor_data(Message, tello);
				}
				else if(id == 18 || id == 19){
					process_foot_sensor_data(Message, tello);
					// cout << "RF: " << tello->_GRFs.right_front << "                  \r";
					// cout.flush();
					// if(id == 18){
					// 	printf("L: %f, \t %f \t\t R: %f, \t %f          \r",tello->_GRFs.left_front,tello->_GRFs.left_back,tello->_GRFs.right_front,tello->_GRFs.right_back);
					// 	std::cout.flush();
					// }
				}
				else if(id > 20)
				{
					process_joint_encoder_data(Message, tello);
				}
				pthread_mutex_unlock(&mutex_CAN_recv);

			}

		}

		usleep(period);
	}
    return NULL;
}

void process_motor_data(TPCANMsg Message, RoboDesignLab::DynamicRobot* robot)
{
	uint8_t id = Message.DATA[0];
	uint16_t pos = (Message.DATA[1] << 8) + Message.DATA[2];
	uint16_t vel  = (Message.DATA[3] << 4) + ((Message.DATA[4] & 0xF0) >> 4);
	uint16_t cur = ((Message.DATA[4] & 0x0F) << 8) + Message.DATA[5];
	
	

	encoder_positions[id-1] = pos;
	if(!position_initialized[id-1]){
		encoder_offsets[id-1] = pos;
		position_initialized[id-1] = 1;
		//printf("ID: %d , OFFSET: %d \n",id,pos);
		printf("Motor %d Connected\n", id);
	}

	encoders[id-1] = pos;
	robot->motors[id-1]->updateState(pos,vel,cur);
	robot->motor_timeouts[id-1] = 0;

}

// Define calibration counters and Eigen matrices globally
constexpr int MAX_SAMPLES = 100;
int left_samples_counter = 0;
int right_samples_counter = 0;

Eigen::VectorXd grf_lf1(MAX_SAMPLES);
Eigen::VectorXd grf_lb1(MAX_SAMPLES);
Eigen::VectorXd grf_rf1(MAX_SAMPLES);
Eigen::VectorXd grf_rb1(MAX_SAMPLES);

void process_foot_sensor_data(TPCANMsg Message, RoboDesignLab::DynamicRobot* robot){
	
	uint8_t id = Message.DATA[0];
	uint16_t front_force_uint16 = (Message.DATA[1] << 8) | Message.DATA[2];
	uint16_t back_force_uint16 = (Message.DATA[3] << 8) | Message.DATA[4];

	// cout << "GOT LOADCELL DATA, ID=" << (int)id << endl;

	// Given front_force_uint16 and back_force_uint16 variables
	double front_force = (double)((int)front_force_uint16 - 32768)/100.0;
	double back_force = (double)((int)back_force_uint16 - 32768)/100.0;

	if(id == 18){
		if (!robot->_left_loadcells_calibrated) {
            // Append forces to Eigen matrices
            grf_lf1[left_samples_counter] = front_force;
            grf_lb1[left_samples_counter] = back_force;

            // Increment sample count
            left_samples_counter++;

            if (left_samples_counter == MAX_SAMPLES) {
                // Calculate average for calibration offset
                robot->_GRF_biases.left_front = grf_lf1.mean();
                robot->_GRF_biases.left_back = grf_lb1.mean();

                robot->_left_loadcells_calibrated = true;
                cout << "Left GRF Offsets:  Front: " << robot->_GRF_biases.left_front
                     << "   Back: " << robot->_GRF_biases.left_back << endl;
            }
        }
		// write to left foot

		grf_lf.tail(99) = grf_lf.head(99).eval();
		grf_lf[0] = -(front_force - robot->_GRF_biases.left_front);

		grf_lb.tail(99) = grf_lb.head(99).eval();
		grf_lb[0] = -(back_force - robot->_GRF_biases.left_back);

		float lf = dash_utils::smoothData(grf_lf, 0.2/*alpha*/);
		float lb = dash_utils::smoothData(grf_lb, 0.2/*alpha*/);


		robot->_GRFs.left_front = lf;
		robot->_GRFs.left_back = lb;
	}
	if(id == 19){
		if (!robot->_right_loadcells_calibrated) {
            // Append forces to Eigen matrices
            grf_rf1(right_samples_counter) = front_force;
            grf_rb1(right_samples_counter) = back_force;

            // Increment sample count
            right_samples_counter++;

            if (right_samples_counter == MAX_SAMPLES) {
                // Calculate average for calibration offset
                robot->_GRF_biases.right_front = grf_rf1.mean();
                robot->_GRF_biases.right_back = grf_rb1.mean();

                robot->_right_loadcells_calibrated = true;
                cout << "Right GRF Offsets:  Front: " << robot->_GRF_biases.right_front
                     << "   Back: " << robot->_GRF_biases.right_back << endl;
            }
        }
		// write to right foot

		grf_rf.tail(99) = grf_rf.head(99).eval();
		grf_rf[0] = -(front_force - robot->_GRF_biases.right_front);

		grf_rb.tail(99) = grf_rb.head(99).eval();
		grf_rb[0] = -(back_force - robot->_GRF_biases.right_back);

		float rf = dash_utils::smoothData(grf_rf, 0.2/*alpha*/);
		float rb = dash_utils::smoothData(grf_rb, 0.2/*alpha*/);

		robot->_GRFs.right_front = rf;
		robot->_GRFs.right_back = rb;
	}
}

															
double joint_zeros[10] = {0,5528,11090-65,2758+50,4240+55,0,2603,4367+25,15047+0,5530-60}; //{0,5528,11090,2808,4290-15,0,2603,4367,15047+25,5530-30};
double joint_directions[10] =      { 1,-1,1,1,-1,    1,-1,-1,-1,1};
double joint_measured_zero_offsets[10] = {0,0,-0.15708,0.382,0,0,0,0.15708,-0.382,0};
void process_joint_encoder_data(TPCANMsg Message, RoboDesignLab::DynamicRobot* robot){

	joint_zeros[2] = 11090-65-10 + l_h;
	joint_zeros[3] = 2758+50 + l_k;
	joint_zeros[4] = 4240+55-60 + l_a;

	joint_zeros[7] = 4367+25 + r_h;
	joint_zeros[8] = 15047+0-15 + r_k;
	joint_zeros[9] = 5530-60+60 + r_a;
	
	uint8_t id = Message.DATA[0];
	uint16_t joint_position = (Message.DATA[1] << 8) | Message.DATA[2];

	// uint16_t joint_velocity = (Message.DATA[3] << 8) | Message.DATA[4];

	// Given front_force_uint16 and back_force_uint16 variables
	double joint_rad = joint_directions[id-20]*((((double)joint_position-joint_zeros[id-20]) *(2.0*M_PI)/16384.0) + joint_measured_zero_offsets[id-20]);
	// double joint_rad_per_sec = ((double)(joint_velocity-8192)*(2.0*M_PI)/16384.0);

	robot->setJointEncoderPosition(joint_rad,static_cast<JointName>(id-20));
	// robot->setJointEncoderVelocity(joint_rad_per_sec,static_cast<JointName>(id-20));
	// if(id==22)
	// {
	// 	cout << "ID: " << ((int)id) << ",   joint_position-zero: " << joint_position << "             \r";
	// }
	
}

// void* rx_UDP( void * arg ){

// 	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
//     void* arg1 = std::get<1>(*arg_tuple_ptr);
// 	void* arg0 = std::get<0>(*arg_tuple_ptr);
// 	int period = std::get<2>(*arg_tuple_ptr);
// 	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(arg0);

// 	//setup
// 	int sockfd;
// 	char rx_buffer[100];

// 	struct sockaddr_in servaddr, cliaddr;
		
// 	// Creating socket file descriptor
// 	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
// 		perror("socket creation failed");
// 		exit(EXIT_FAILURE);
// 	}
		
// 	memset(&servaddr, 0, sizeof(servaddr));
// 	memset(&cliaddr, 0, sizeof(cliaddr));
		
// 	// Filling server information
// 	servaddr.sin_family = AF_INET; // IPv4
// 	servaddr.sin_addr.s_addr = INADDR_ANY;
// 	servaddr.sin_port = htons(UDP_RECEIVE_PORT);
		
// 	// Bind the socket with the server address
// 	if ( bind(sockfd, (const struct sockaddr *)&servaddr,
// 			sizeof(servaddr)) < 0 )
// 	{
// 		perror("bind failed");
// 		exit(EXIT_FAILURE);
// 	}

// 	int core = sched_getcpu();
// 	int policy;
// 	sched_param param;
//     pthread_t current_thread = pthread_self();
//     int result = pthread_getschedparam(current_thread, &policy, &param);
// 	int priority = param.sched_priority;
// 	printf("UDP rx thread running on core %d, with priority %d\n", core, priority);

// 	int len, n;

// 	while(1)
// 	{
// 		socklen_t * len1;
// 		n = recvfrom(sockfd, (char *)rx_buffer, 100,
// 			MSG_WAITALL, ( struct sockaddr *) &cliaddr,
// 			len1 );
// 		uint8_t checksum = 0;
// 		for(int i=0;i<n-1;i++){
// 			checksum += rx_buffer[i]&0xFF;
// 		}
// 		if(checksum == rx_buffer[n-1])
// 		{
// 			Human_dyn_data human_dyn_data;
// 			dash_utils::unpack_data_from_hmi(human_dyn_data,(uint8_t*)rx_buffer);
// 			tello->controller->set_human_dyn_data(human_dyn_data);
// 			dash_utils::print_human_dyn_data(human_dyn_data);
// 		}


// 		// udp_data_ready = 0;
// 		// memcpy(udp_control_packet,buffer,UDP_MAXLINE);
// 		// udp_data_ready = 1;
// 		// HERE WE HAVE A UDP CONTROL PACKET TO SEND TO THE UPDATE LOOP
		

// 		usleep(100);
// 	}
// }

VectorXd  xHvec(100);
VectorXd dxHvec(100);
VectorXd pxHvec(100);
VectorXd  yHvec(100);
VectorXd dyHvec(100);
VectorXd pyHvec(100);
double xHval, dxHval, pxHval, yHval, dyHval, pyHval;
int dyn_data_idx = 0;


extern double dtime;

// Function to copy button data from a uint8_t buffer back into the struct
void DeserializeVizControlData(const uint8_t* buffer, size_t bufferSize, HW_CTRL_Data& data) {
    if (bufferSize < sizeof(HW_CTRL_Data)) {
        // Handle buffer size error
        return;
    }

    memcpy(&data, buffer, sizeof(HW_CTRL_Data));
}

void* rx_UDP( void * arg ){

	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
    void* arg1 = std::get<1>(*arg_tuple_ptr);
	void* arg0 = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);
	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(arg0);

	//setup
	int sockfd;
	char rx_buffer[100];

	struct sockaddr_in servaddr, cliaddr;
		
	// Creating socket file descriptor
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
		
	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));
		
	// Filling server information
	servaddr.sin_family = AF_INET; // IPv4
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(UDP_RECEIVE_PORT);
		
	// Bind the socket with the server address
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,
			sizeof(servaddr)) < 0 )
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}

	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("UDP rx thread running on core %d, with priority %d\n", core, priority);

	int len, n;

	Eigen::VectorXd xHvec(100);
    Eigen::VectorXd dxHvec(100);
    Eigen::VectorXd pxHvec(100);
    Eigen::VectorXd yHvec(100);
    Eigen::VectorXd dyHvec(100);
    Eigen::VectorXd pyHvec(100);
    Eigen::VectorXd fxH_Rvec(100);
    Eigen::VectorXd fyH_Rvec(100);
    Eigen::VectorXd fzH_Rvec(100);
    Eigen::VectorXd fxH_Lvec(100);
    Eigen::VectorXd fyH_Lvec(100);
    Eigen::VectorXd fzH_Lvec(100);
    Eigen::VectorXd fdxH_Rvec(100);
    Eigen::VectorXd fdyH_Rvec(100);
    Eigen::VectorXd fdzH_Rvec(100);
    Eigen::VectorXd fdxH_Lvec(100);
    Eigen::VectorXd fdyH_Lvec(100);
    Eigen::VectorXd fdzH_Lvec(100);
    Eigen::VectorXd FxH_hmi_vec(100);
    Eigen::VectorXd FyH_hmi_vec(100);
    Eigen::VectorXd FxH_spring_vec(100);
    double xHval;
    double dxHval;
    double pxHval;
    double yHval;
    double dyHval;
    double pyHval;
    double fxH_Rval;
    double fyH_Rval;
    double fzH_Rval;
    double fxH_Lval;
    double fyH_Lval;
    double fzH_Lval;
    double fdxH_Rval;
    double fdyH_Rval;
    double fdzH_Rval;
    double fdxH_Lval;
    double fdyH_Lval;
    double fdzH_Lval;
    double FxH_hmi_val;
    double FyH_hmi_val;
    double FxH_spring_val;

	while(1)
	{
		socklen_t * len1;
		// dash_utils::print_timer();
		// dash_utils::start_timer();
		struct sockaddr_in sender_addr;  // Temporary variable to capture sender's address
		socklen_t sender_addr_len = sizeof(sender_addr);

		n = recvfrom(sockfd, (char *)rx_buffer, 100,
			MSG_WAITALL, (struct sockaddr*)&sender_addr, &sender_addr_len);


		if (strcmp(inet_ntoa(sender_addr.sin_addr), HMI_IP_ADDRESS) == 0) {
            
			uint8_t checksum = 0;
			for(int i=0;i<n-1;i++){
				checksum += rx_buffer[i]&0xFF;
			}
			// if(checksum == rx_buffer[n-1])
			// {
				Human_dyn_data human_dyn_data;
			// }
			
			if(tello->controller->is_human_ctrl_enabled())
			{
				dash_utils::unpack_data_from_hmi(human_dyn_data,(uint8_t*)rx_buffer);
				// cout << "Using Human Data" << endl;
			}
			else
			{
				Human_params hp;
				dash_init::Human_Init(hp,human_dyn_data);

				// code for initializing foot width
				Human_dyn_data hdd;
				dash_utils::unpack_data_from_hmi(hdd,(uint8_t*)rx_buffer);

				double hR = tello->controller->get_SRB_params().hLIP;
				double hH = hp.hLIP; 
				double fyH_home = hp.fyH_home;

				double fyH_R = hdd.fyH_R;
				double fyH_L = hdd.fyH_L;

				double joystick_base_separation = 1.525;
				double foot_center_to_joystick = FOOT_2_JOYSTICK;

				// double pyH_lim_lb = -1 * human_nom_ft_width + (fyH_R - fyH_home);
				// double pyH_lim_ub = human_nom_ft_width - (fyH_L - fyH_home);
				double human_foot_width = joystick_base_separation - 2*foot_center_to_joystick - fyH_R - fyH_L;
				if(use_current_foot_width)
				{
					robot_init_foot_width_HW = human_foot_width*(hR/hH);
					use_current_foot_width = false;
				}

				robot_init_foot_width = human_foot_width*(hR/hH);
				// cout << "Robot Width: " << robot_init_foot_width << "    Human Width: " << human_foot_width << "    R: "<< fyH_R << "    L: "<< fyH_L << endl;
			}
			
			// =======================================================================================================
					
			xHvec.tail(99) = xHvec.head(99).eval();
			xHvec[0] = human_dyn_data.xH;

			dxHvec.tail(99) = dxHvec.head(99).eval();
			dxHvec[0] = human_dyn_data.dxH;

			pxHvec.tail(99) = pxHvec.head(99).eval();
			pxHvec[0] = human_dyn_data.pxH;

			yHvec.tail(99) = yHvec.head(99).eval();
			yHvec[0] = human_dyn_data.yH;

			dyHvec.tail(99) = dyHvec.head(99).eval();
			dyHvec[0] = human_dyn_data.dyH;

			pyHvec.tail(99) = pyHvec.head(99).eval();
			pyHvec[0] = human_dyn_data.pyH;

			fxH_Rvec.tail(99) = fxH_Rvec.head(99).eval();
			fxH_Rvec[0] = human_dyn_data.fxH_R;

			fyH_Rvec.tail(99) = fyH_Rvec.head(99).eval();
			fyH_Rvec[0] = human_dyn_data.fyH_R;

			fzH_Rvec.tail(99) = fzH_Rvec.head(99).eval();
			fzH_Rvec[0] = human_dyn_data.fzH_R;

			fxH_Lvec.tail(99) = fxH_Lvec.head(99).eval();
			fxH_Lvec[0] = human_dyn_data.fxH_L;

			fyH_Lvec.tail(99) = fyH_Lvec.head(99).eval();
			fyH_Lvec[0] = human_dyn_data.fyH_L;

			fzH_Lvec.tail(99) = fzH_Lvec.head(99).eval();
			fzH_Lvec[0] = human_dyn_data.fzH_L;

			fdxH_Rvec.tail(99) = fdxH_Rvec.head(99).eval();
			fdxH_Rvec[0] = human_dyn_data.fdxH_R;

			fdyH_Rvec.tail(99) = fdyH_Rvec.head(99).eval();
			fdyH_Rvec[0] = human_dyn_data.fdyH_R;

			fdzH_Rvec.tail(99) = fdzH_Rvec.head(99).eval();
			fdzH_Rvec[0] = human_dyn_data.fdzH_R;

			fdxH_Lvec.tail(99) = fdxH_Lvec.head(99).eval();
			fdxH_Lvec[0] = human_dyn_data.fdxH_L;

			fdyH_Lvec.tail(99) = fdyH_Lvec.head(99).eval();
			fdyH_Lvec[0] = human_dyn_data.fdyH_L;

			fdzH_Lvec.tail(99) = fdzH_Lvec.head(99).eval();
			fdzH_Lvec[0] = human_dyn_data.fdzH_L;

			FxH_hmi_vec.tail(99) = FxH_hmi_vec.head(99).eval();
			FxH_hmi_vec[0] = human_dyn_data.FxH_hmi;

			FyH_hmi_vec.tail(99) = FyH_hmi_vec.head(99).eval();
			FyH_hmi_vec[0] = human_dyn_data.FyH_hmi;

			FxH_spring_vec.tail(99) = FxH_spring_vec.head(99).eval();
			FxH_spring_vec[0] = human_dyn_data.FxH_spring;

			xHval = dash_utils::smoothData(xHvec, 0.1/*alpha*/);
			dxHval = dash_utils::smoothData(dxHvec, 2.0/*alpha*/);
			pxHval = dash_utils::smoothData(pxHvec, 0.1/*alpha*/);
			yHval = dash_utils::smoothData(yHvec, 0.1/*alpha*/);
			dyHval = dash_utils::smoothData(dyHvec, 2.0/*alpha*/);
			pyHval = dash_utils::smoothData(pyHvec, 0.1/*alpha*/);
			fxH_Rval = dash_utils::smoothData(fxH_Rvec, 0.2/*alpha*/);
			fyH_Rval = dash_utils::smoothData(fyH_Rvec, 0.2/*alpha*/);
			fzH_Rval = dash_utils::smoothData(fzH_Rvec, 0.2/*alpha*/);
			fxH_Lval = dash_utils::smoothData(fxH_Lvec, 0.2/*alpha*/);
			fyH_Lval = dash_utils::smoothData(fyH_Lvec, 0.2/*alpha*/);
			fzH_Lval = dash_utils::smoothData(fzH_Lvec, 0.2/*alpha*/);
			fdxH_Rval = dash_utils::smoothData(fdxH_Rvec, 4.0/*alpha*/);
			fdyH_Rval = dash_utils::smoothData(fdyH_Rvec, 4.0/*alpha*/);
			fdzH_Rval = dash_utils::smoothData(fdzH_Rvec, 4.0/*alpha*/);
			fdxH_Lval = dash_utils::smoothData(fdxH_Lvec, 4.0/*alpha*/);
			fdyH_Lval = dash_utils::smoothData(fdyH_Lvec, 4.0/*alpha*/);
			fdzH_Lval = dash_utils::smoothData(fdzH_Lvec, 4.0/*alpha*/);
			FxH_hmi_val = dash_utils::smoothData(FxH_hmi_vec, 0.1/*alpha*/);
			FyH_hmi_val = dash_utils::smoothData(FyH_hmi_vec, 0.1/*alpha*/);
			FxH_spring_val = dash_utils::smoothData(FxH_spring_vec, 0.1/*alpha*/);

			human_dyn_data.xH = xHval;
			human_dyn_data.dxH = dxHval;
			human_dyn_data.pxH = pxHval;
			human_dyn_data.yH = yHval;
			human_dyn_data.dyH = dyHval;
			human_dyn_data.pyH = pyHval;
			human_dyn_data.fxH_R = fxH_Rval;
			human_dyn_data.fyH_R = fyH_Rval;
			human_dyn_data.fzH_R = fzH_Rval;
			human_dyn_data.fxH_L = fxH_Lval;
			human_dyn_data.fyH_L = fyH_Lval;
			human_dyn_data.fzH_L = fzH_Lval;
			human_dyn_data.fdxH_R = fdxH_Rval;
			human_dyn_data.fdyH_R = fdyH_Rval;
			human_dyn_data.fdzH_R = fdzH_Rval;
			human_dyn_data.fdxH_L = fdxH_Lval;
			human_dyn_data.fdyH_L = fdyH_Lval;
			human_dyn_data.fdzH_L = fdzH_Lval;
			human_dyn_data.FxH_hmi = FxH_hmi_val;
			human_dyn_data.FyH_hmi = FyH_hmi_val;
			human_dyn_data.FxH_spring = FxH_spring_val;

			tello->controller->updateStepZHistoryL(fzH_Lval);
			tello->controller->updateStepZHistoryR(fzH_Rval);
			tello->controller->updateStepTimeHistory(dtime);

			Traj_planner_dyn_data tpdds = tello->controller->get_traj_planner_dyn_data();
			tpdds.step_z_history_L = tello->controller->getStepZHistoryL();
			tpdds.step_z_history_R = tello->controller->getStepZHistoryR();
			if(tpdds.human_FSM != 0)
				tpdds.curr_SSP_sample_count = tpdds.curr_SSP_sample_count + 1;
			tello->controller->set_traj_planner_step_data(tpdds);
			
			// =======================================================================================================

			// if(!(sim_conf.en_playback_mode))
			// {
				tello->controller->set_human_dyn_data_without_forces(human_dyn_data);
			// }
		}
		else if (strcmp(inet_ntoa(sender_addr.sin_addr), VIZ_IP_ADDRESS) == 0) {
            // Call the deserialize function on the data
            pthread_mutex_lock(&mutex_UDP_recv);
            DeserializeVizControlData((const uint8_t*)rx_buffer, n, hw_control_data);
			pthread_mutex_unlock(&mutex_UDP_recv);
			l_h = hw_control_data.hip_offset_left;
			l_k = hw_control_data.knee_offset_left;
			l_a = hw_control_data.ankle_offset_left;

			r_h = hw_control_data.hip_offset_right;
			r_k = hw_control_data.knee_offset_right;
			r_a = hw_control_data.ankle_offset_right;
			// cout << "Hip: " << hw_control_data.hip_offset_right << endl;

			roll_adjust = hw_control_data.roll_adjust;
			pitch_adjust = hw_control_data.pitch_adjust;
			yaw_adjust = hw_control_data.yaw_adjust;
			// cout << "Client: " << inet_ntoa(sender_addr.sin_addr) << "      \r";
			// cout 
			// 	<< "  tare: " << hw_ctrl_data.tare_hmi 
			// 	<< "  startL: " << hw_ctrl_data.start_legs 
			// 	<< "  balance: " << hw_ctrl_data.balance 
			// 	<< "  teleop: " << hw_ctrl_data.enable_teleop 
			// 	<< "  e-stop: " << hw_ctrl_data.emergency_stop 
			// 	<< "  hmi gain: " << hw_ctrl_data.hmi_gain 
			// 	<< "                    \r";
			// cout.flush();
            // ... process the button data from VIZ ...
        }
		//dash_utils::print_human_dyn_data(human_dyn_data);

		// udp_data_ready = 0;
		// memcpy(udp_control_packet,buffer,UDP_MAXLINE);
		// udp_data_ready = 1;
		// HERE WE HAVE A UDP CONTROL PACKET TO SEND TO THE UPDATE LOOP
		

		usleep(10);
	}
}


//    IMU   =================================================================================

using namespace std;

Eigen::Matrix3d rotateAlign(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    Eigen::Vector3d axis = v1.cross(v2).normalized();
    float dotProduct = v1.dot(v2);
	if(dotProduct< -1) dotProduct = -1;
	if(dotProduct > 1) dotProduct = 1;
    float angleRadians = acos(dotProduct);

    const float sinA = sin(angleRadians);
    const float cosA = cos(angleRadians);
    const float oneMinusCosA = 1.0f - cosA;

    Eigen::Matrix3d result;
    result << (axis[0] * axis[0] * oneMinusCosA) + cosA,
              (axis[1] * axis[0] * oneMinusCosA) - (sinA * axis[2]), 
              (axis[2] * axis[0] * oneMinusCosA) + (sinA * axis[1]),
              (axis[0] * axis[1] * oneMinusCosA) + (sinA * axis[2]),  
              (axis[1] * axis[1] * oneMinusCosA) + cosA,      
              (axis[2] * axis[1] * oneMinusCosA) - (sinA * axis[0]),
              (axis[0] * axis[2] * oneMinusCosA) - (sinA * axis[1]),  
              (axis[1] * axis[2] * oneMinusCosA) + (sinA * axis[0]),  
              (axis[2] * axis[2] * oneMinusCosA) + cosA;

    return result;
}

VectorXd calculate_bias(const MatrixXd& acceleration_data) {
    int num_samples = acceleration_data.rows(); // get the number of samples
    VectorXd accelerometer_bias(3); // create a vector to store the biases

    // compute the mean of each column
    accelerometer_bias = acceleration_data.colwise().mean();

    return accelerometer_bias;
}

std::chrono::_V2::system_clock::time_point lastTime;
long long getDeltaT() {
  auto currentTime = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(currentTime - lastTime).count();
  lastTime = currentTime;
  return duration;
}

Eigen::VectorXd subtractGravity2(const Eigen::Vector3d& ypr, const Eigen::VectorXd& acc)
{
    // Construct rotation matrix from roll-pitch-yaw angles
    AngleAxisd pitchRotation(ypr[1], Vector3d::UnitY());
    AngleAxisd rollRotation(ypr[2], Vector3d::UnitX());

    Matrix3d R = pitchRotation.matrix() * rollRotation.matrix();


    // Calculate gravitational acceleration in body frame
    Eigen::Vector3d g_body(0, 0, 10.02);
    Eigen::Vector3d g_enu = R.transpose() * g_body;

    // Subtract gravitational acceleration from measured acceleration
    Eigen::VectorXd acc_out(3);
    acc_out = acc - g_enu;

	// printf("    %.5f, \t\t %.5f, \t\t %.5f                   \r", g_enu[0], g_enu[1], g_enu[2]);
	// cout.flush();
    return acc_out;
}
Eigen::VectorXd subtractAccelBiases(const Eigen::Vector3d& ypr, const Eigen::VectorXd& acc)
{
    // Construct rotation matrix from roll-pitch-yaw angles
    AngleAxisd pitchRotation(ypr[1], Vector3d::UnitY());
    AngleAxisd rollRotation(ypr[2], Vector3d::UnitX());

    Matrix3d R = pitchRotation.matrix() * rollRotation.matrix();


    // Calculate gravitational acceleration in body frame
    Eigen::Vector3d g_body(0, 0, 10.02);
    Eigen::Vector3d g_enu = R.transpose() * g_body;

    // Subtract gravitational acceleration from measured acceleration
    Eigen::VectorXd acc_out(3);
    acc_out = acc - g_enu;

	// printf("    %.5f, \t\t %.5f, \t\t %.5f                   \r", g_enu[0], g_enu[1], g_enu[2]);
	// cout.flush();
    return acc_out;
}

void* BNO055_Comms( void * arg ){

	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* arg0 = std::get<0>(*arg_tuple_ptr);
	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(arg0);

	BNO055* imu = new BNO055(0,40,3);
	imu->begin(BNO055::OPERATION_MODE_CONFIG);
	imu->setExtCrystalUse(false);
	imu->setAxisRemap(BNO055::REMAP_CONFIG_P7);
	imu->setAxisSign(BNO055::REMAP_SIGN_P7);

	imu->setAccelConfig(BNO055::BNO055_ACCEL_CONFIG_16G, BNO055::BNO055_ACCEL_CONFIG_1000Hz);
	imu->setMode(BNO055::OPERATION_MODE_ACCGYRO);


	Eigen::Vector3d v(0.05, -0.99, 9.84);
    Eigen::Vector3d z_axis(0, 0, 1 );

    // Step 1: Calculate the angle between v and the z-axis
    double angle = std::acos(v.dot(z_axis) / v.norm());

    // Step 2: Calculate the rotation axis using cross product between v and z-axis
    Eigen::Vector3d axis = v.cross(z_axis).normalized();

    // Step 3: Construct the rotation matrix
    Eigen::AngleAxisd rotation(angle, axis);
    Eigen::Matrix3d offset_correction = rotation.toRotationMatrix();

    std::cout << "Rotation matrix:\n" << offset_correction << std::endl << std::endl;

	
	lastTime = chrono::high_resolution_clock::now();
	while(1){
		
		VectorXd xyz = (imu->getVector(BNO055::VECTOR_ACCELEROMETER));
		xyz = Vector3d(-xyz[1], xyz[0], xyz[2]);
		xyz = subtractGravity2(tello->_ypr, xyz);

		// if(calibrate_IMU_bias){
		// 	if(cal_index < 1000){
		// 		acc_cal.row(cal_index) = xyz.transpose();
		// 		cal_index++;
		// 	}
		// 	else{
		// 		VectorXd biases = calculate_accelerometer_bias(acc_cal);
		// 		printf("Biases: %.6f,\t %.6f,\t %.6f                    \n\n",biases[0], biases[1], biases[2]);
		// 		calibrate_IMU_bias = false;
		// 	}
		// }
		// tello->_acc = xyz;
		// int delta_T = getDeltaT();
		// double dt = (double)delta_T/1000000.0;
		// tello->updatePosFromIMU(tello->_acc, tello->_ypr,dt,tello->_pos, tello->_vel);

		//printf("POS: %.3f,\t %.3f,\t %.3f               \r",tello->_pos[0], tello->_pos[1], tello->_pos[2]);
		std::cout.flush();

		// if(print_index%50 == 0){
		// 	printf("    %.5f, \t\t %.5f, \t\t %.5f                   \r", xyz[0], xyz[1], xyz[2]);
		// 	//printf("IMU: x: %f, \ty: %f, \tz: %f                   \r", xa*10, ya*10, za*10);
		// 	std::cout.flush();
		// }
		// print_index++;

		usleep(1000);
	}

}

using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace vn::math;

void IMUMessageReceived(void * robot, Packet & p, size_t index);

void* IMU_Comms( void * arg ){ //readImuMeasurements()

	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	void* arg0 = std::get<0>(*arg_tuple_ptr);
	RoboDesignLab::DynamicRobot* tello = reinterpret_cast<RoboDesignLab::DynamicRobot*>(arg0);

	// Open the serial port to communicate with the VN100 IMU
    const std::string port = "/dev/imu";
	optimize_serial_communication(port);
    const int baudrate = 921600;
    VnSensor vs;
    vs.connect(port, baudrate);
    printf('g',"\n\nConnected to VN100 IMU on port %s \n",port.c_str());

	// Make sure no generic async output is registered
  	vs.writeAsyncDataOutputType(VNOFF);

	 // Query the sensor's model number.
	string mn = vs.readModelNumber();
	string fv = vs.readFirmwareVersion();
	uint32_t hv = vs.readHardwareRevision();
	uint32_t sn = vs.readSerialNumber();
	printf("Model Number: %s, Firmware Version: %s\n", mn.c_str(), fv.c_str());
	printf("Hardware Revision : %d, Serial Number : %d\n\n", hv, sn);

    vn::math::mat3f vnMat;

	vnMat.e00 = -1;
	vnMat.e01 = 0;
	vnMat.e02 = 0;

	vnMat.e10 = 0;
	vnMat.e11 = -1;
	vnMat.e12 = 0;

	vnMat.e20 = 0;
	vnMat.e21 = 0;
	vnMat.e22 = 1;

	vs.writeReferenceFrameRotation(vnMat, true);

	VpeEnable en;
	 // Set the VPE basic control configuration
    VpeBasicControlRegister vpeControl;
    vpeControl.enable = en;
    vpeControl.headingMode = HEADINGMODE_RELATIVE;
    vpeControl.filteringMode = VPEMODE_MODE1;

    vs.writeVpeBasicControl(vpeControl);


	// Configure binary output message
	BinaryOutputRegister bor(
		ASYNCMODE_PORT2,
		1,  // update rate [ms]
		COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_ACCEL,
		TIMEGROUP_NONE, 
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE, //ATTITUDEGROUP_YPRU,  //<-- returning yaw pitch roll uncertainties
		INSGROUP_NONE, //INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELBODY | INSGROUP_ACCELECEF |
		//INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU,
		GPSGROUP_NONE);

	// An empty output register for disabling output 2 and 3 if previously set
	BinaryOutputRegister bor_none(
		0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE, ATTITUDEGROUP_NONE,
		INSGROUP_NONE, GPSGROUP_NONE);

	vs.writeBinaryOutput1(bor);
	vs.writeBinaryOutput2(bor_none);
	vs.writeBinaryOutput3(bor_none);

	vs.writeSettings();

	lastTime = chrono::high_resolution_clock::now();
	vs.registerAsyncPacketReceivedHandler(tello, IMUMessageReceived);

	// read IMU
	while(1){
		usleep(2500);
	}

}

#define FILTER_LEN 20
vector<float> samples(FILTER_LEN, 0.0);
long idx = 0;
float filterFloat(const vector<float>& samples) {
  // Compute the average of the last 25 samples
  float sum = 0.0;
  for (int i = 0; i < FILTER_LEN; i++) {
    sum += samples[i];
  }
  return sum / (float)FILTER_LEN;
}

Eigen::Vector3d vn2Eig_3D(const vn::math::vec3f& vec) {
  // Create a new Eigen::Vector3d object and copy the x, y, and z values from the vn::math::vec3f object
  Eigen::Vector3d result(vec.x, vec.y, vec.z);
  return result;
}

void subtractGravity(Eigen::Vector3d& accel, const Eigen::Vector3d& ypr) {
	// Calculate the gravity vector based on the yaw, pitch, and roll angles
	Eigen::Matrix3d R;
	double yaw = ypr(0 );
	double pitch = ypr(1 );
	double roll = ypr(2 );
	double cy = cos(yaw);
	double sy = sin(yaw);
	double cp = cos(pitch);
	double sp = sin(pitch);
	double cr = cos(roll);
	double sr = sin(roll);
	R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
		sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
		-sp, cp*sr, cp*cr;
		
	double calibration_error = 0.227395;
	Eigen::Vector3d gravity(0.0, 0.0, 9.81+calibration_error);
	Eigen::Vector3d biases(-0.001718,-0.000485,0.227395);
	Eigen::Vector3d gravity_in_imu_frame = R.transpose() * (gravity);

	// Subtract the gravity vector from the acceleration vector
	accel -= gravity_in_imu_frame;
}
int cutoff = 0; // 15 millisecond cutoff;
void IMUMessageReceived(void * robot, Packet & p, size_t index)
{
	RoboDesignLab::DynamicRobot* tello = (RoboDesignLab::DynamicRobot*) robot;
	vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

	vec3f ypr = cd.yawPitchRoll();
	vec3f acc = cd.acceleration();
	vec3f gyro = cd.angularRate();

	// if(fabs(acc[0]) < 0.5) acc[0] = 0;
	// if(fabs(acc[1]) < 0.5) acc[1] = 0;
	// if(fabs(acc[2]) < 0.5) acc[2] = 0;
	// if( fabs(acc[0]) < 0.5 && fabs(acc[1]) < 0.5 && fabs(acc[2]) < 0.5 ) tello->_vel = VectorXd::Zero(3);

	if(ypr[2] > 0){
		ypr[2] = -180.0 + ypr[2];
	}
	else if(ypr[2] < 0){
		ypr[2] = 180.0 + ypr[2];
	}
	ypr[1] = -ypr[1];

	if(!yaw_offset_recorded) 
	{
		initial_yaw = ypr[0];
		yaw_offset_recorded = true;
	}

	ypr[0] = -ypr[0] + initial_yaw;

	tello_ypr = ypr;
	tello->_ypr = vn2Eig_3D(ypr)*DEGREES_TO_RADIANS;
	tello->_rpy = tello->_ypr.reverse().eval();
	tello->_rpy(2) = 0;
	Vector3d acc_eig = vn2Eig_3D(acc).array()*(9.81/10.02);
	//acc_eig = subtractGravity2(tello->_ypr,acc_eig);
	tello->_acc = acc_eig;
	tello->_gyro = vn2Eig_3D(gyro);
	if(calibrate_IMU_bias){
		if(cal_index < 20000){
			acc_cal.row(cal_index) = acc_eig.transpose();
			gyro_cal.row(cal_index) = tello->_gyro.transpose();
			cal_index++;
		}
		else{ // NOTE: VN100 seems to be calibrated without measureable bias over 20,000 samples
			VectorXd acc_biases = calculate_bias(acc_cal);
			printf("ACC Biases: %.6f,\t\t %.6f,\t\t %.6f                    \n\n",acc_biases[0], acc_biases[1], acc_biases[2]);

			VectorXd gyro_biases = calculate_bias(gyro_cal);
			printf("GYR Biases: %.6f,\t\t %.6f,\t\t %.6f                    \n\n",gyro_biases[0], gyro_biases[1], gyro_biases[2]);

			calibrate_IMU_bias = false;
			cal_index = 0;
		}
	}


	// printf("YPR: %.2f,\t %.2f,\t %.2f          \r",ypr[0], ypr[1], ypr[2]);
	// cout.flush();
	// printf("%.4f,\t\t %.4f,\t\t %.4f               \n",acc_eig[0], acc_eig[1], acc_eig[2]);
	// cout.flush();

	int delta_T = getDeltaT();
	double dt = (double)delta_T/1000000.0;

	tello->updatePosFromIMU(tello->_acc, tello->_ypr,dt,tello->_pos, tello->_vel);
	// printf("POS: %.3f,\t %.3f,\t %.3f         \n",tello->_pos[0], tello->_pos[1], tello->_pos[2]);
	// cout.flush();

}