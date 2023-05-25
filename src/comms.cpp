#include "comms.h"
#include "mujoco_main.h"

extern uint16_t encoders[10];

// extern CheetahMotor* motors[10];

extern int position_initialized[10];
extern uint16_t encoder_positions[10];
extern uint16_t encoder_offsets[10];

extern int udp_data_ready;
extern char udp_control_packet[UDP_MAXLINE];

extern pthread_mutex_t mutex_CAN_recv;
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

void* rx_CAN( void * arg ){
    
	TPCANStatus Status;
	TPCANMsg Message;
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
    void* arg1 = std::get<1>(*arg_tuple_ptr);
	void* arg0 = std::get<0>(*arg_tuple_ptr);
	int period = std::get<2>(*arg_tuple_ptr);
    int pcd = *reinterpret_cast<int*>(arg1);
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
					// if(id == 18){
					// 	printf("L: %f, \t %f \t\t R: %f, \t %f          \r",tello->_GRFs.left_front,tello->_GRFs.left_back,tello->_GRFs.right_front,tello->_GRFs.right_back);
					// 	std::cout.flush();
					// }
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

void process_foot_sensor_data(TPCANMsg Message, RoboDesignLab::DynamicRobot* robot){
	
	uint8_t id = Message.DATA[0];
	uint16_t back_force_uint16 = (Message.DATA[1] << 8) | Message.DATA[2];
	uint16_t front_force_uint16 = (Message.DATA[3] << 8) | Message.DATA[4];

	// Given front_force_uint16 and back_force_uint16 variables
	double front_force = (double)((int)front_force_uint16 - 32768)/1000.0;
	double back_force = (double)((int)back_force_uint16 - 32768)/1000.0;

	if(id == 18){
		if(!robot->_left_loadcells_calibrated){
			robot->_GRF_biases.left_front = front_force;
			robot->_GRF_biases.left_back = back_force;
			robot->_left_loadcells_calibrated = true;
		}
		// write to left foot
		robot->_GRFs.left_front = -(front_force - robot->_GRF_biases.left_front);
		robot->_GRFs.left_back = -(back_force - robot->_GRF_biases.left_back);
	}
	if(id == 19){
		if(!robot->_right_loadcells_calibrated){
			robot->_GRF_biases.right_front = front_force;
			robot->_GRF_biases.right_back = back_force;
			robot->_right_loadcells_calibrated = true;
		}
		// write to right foot
		robot->_GRFs.right_front = -(front_force - robot->_GRF_biases.right_front);
		robot->_GRFs.right_back = -(back_force - robot->_GRF_biases.right_back);
	}
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
// 			len1);
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

	while(1)
	{
		socklen_t * len1;
		n = recvfrom(sockfd, (char *)rx_buffer, 100,
			MSG_WAITALL, ( struct sockaddr *) &cliaddr,
			len1);
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
		}
		else
		{
			Human_params hp;
			dash_init::Human_Init(hp,human_dyn_data);
		}

	// 	// smooth data here
	// 	xHvec.tail(99) = xHvec.head(99).eval();
    //     xHvec[0] = human_dyn_data.xH;
	// 	dxHvec.tail(99) = dxHvec.head(99).eval();
    //     dxHvec[0] = human_dyn_data.dxH;
	// 	pxHvec.tail(99) = pxHvec.head(99).eval();
    //     pxHvec[0] = human_dyn_data.pxH;

	// 	yHvec.tail(99) = yHvec.head(99).eval();
    //     yHvec[0] = human_dyn_data.yH;
	// 	dyHvec.tail(99) = dyHvec.head(99).eval();
    //     dyHvec[0] = human_dyn_data.dyH;
	// 	pyHvec.tail(99) = pyHvec.head(99).eval();
    //     pyHvec[0] = human_dyn_data.pyH;
        

    //    //dash_utils::start_timer();
	// 	xHval = dash_utils::smoothData(xHvec,2);
	// 	dxHval = dash_utils::smoothData(dxHvec,0.5);
	// 	pxHval = dash_utils::smoothData(pxHvec,0);
	// 	yHval = dash_utils::smoothData(yHvec,2);
	// 	dyHval = dash_utils::smoothData(dyHvec,0.5);
	// 	pyHval = dash_utils::smoothData(pyHvec,0);
	// 	//dash_utils::print_timer();
    //     human_dyn_data.xH  =  xHval;
	// 	human_dyn_data.dxH = dxHval;
	// 	human_dyn_data.pxH = pxHval;
	// 	human_dyn_data.yH  =  yHval;
	// 	human_dyn_data.dyH = dyHval;
	// 	human_dyn_data.pyH = pyHval;

		if(!(sim_conf.en_playback_mode))
		{
			tello->controller->set_human_dyn_data(human_dyn_data);
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
    Eigen::Vector3d z_axis(0, 0, 1);

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
	double yaw = ypr(0);
	double pitch = ypr(1);
	double roll = ypr(2);
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