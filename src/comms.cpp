#include "comms.h"

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

void* rx_CAN( void * arg ){
	
    
	TPCANStatus Status;
	TPCANMsg Message;
	//int pcd = *((int*)arg);
	// int pcd = *((int*) (std::get<1>(*((std::tuple<void*, void*, int, int>*)arg))) );
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
    void* arg1 = std::get<1>(*arg_tuple_ptr);
    int pcd = *reinterpret_cast<int*>(arg1);
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
				uint8_t id = Message.DATA[0];
				if(id < 11){
					process_motor_data(Message);
				}
				else if(id == 18 || id == 19){
					process_foot_sensor_data(Message);
				}

			}

		}

		usleep(50);
	}
    return NULL;
}

void process_motor_data(TPCANMsg Message)
{
	unsigned int id = Message.DATA[0];
	unsigned int pos = (Message.DATA[1] << 8) + Message.DATA[2];
	unsigned int vel  = (Message.DATA[3] << 4) + ((Message.DATA[4] & 0xF0) >> 4);
	unsigned int cur = ((Message.DATA[4] & 0x0F) << 8) + Message.DATA[5];
	
	pthread_mutex_lock(&mutex_CAN_recv);

	encoder_positions[id-1] = pos;
	if(!position_initialized[id-1]){
		encoder_offsets[id-1] = pos;
		position_initialized[id-1] = 1;
		//printf("ID: %d , OFFSET: %d \n",id,pos);
		printf("Motor %d Connected\n", id);
	}

	encoders[id-1] = pos;
	// motors[id-1]->updateState(pos,vel,cur);

	pthread_mutex_unlock(&mutex_CAN_recv);
}

void process_foot_sensor_data(TPCANMsg Message){
	// uint8_t id = Message.DATA[0];
	// uint16_t front_force = (Message.DATA[1] << 8) | Message.DATA[2];
	// uint16_t back_force = (Message.DATA[3] << 8) | Message.DATA[4];
	// if(id == 18){
	// 	// write to left foot
	// }
	// if(id == 19){
	// 	// write to right foot
	// }
}

void* rx_UDP( void * arg ){
	//setup
	int sockfd;
	char buffer[UDP_MAXLINE];

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
	servaddr.sin_port = htons(UDP_PORT);
		
	// Bind the socket with the server address
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,
			sizeof(servaddr)) < 0 )
	{
		perror("bind failed");unsigned int pcd3 = PCAN_PCIBUS3;
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
		n = recvfrom(sockfd, (char *)buffer, UDP_MAXLINE,
			MSG_WAITALL, ( struct sockaddr *) &cliaddr,
			len1);
		uint8_t checksum = 0;
		for(int i=0;i<n-1;i++){
			checksum += buffer[i]&0xFF;
		}
		//TODO: use checksum

		// HERE WE HAVE A UDP CONTROL PACKET TO SEND TO THE UPDATE LOOP
		udp_data_ready = 0;
		memcpy(udp_control_packet,buffer,UDP_MAXLINE);
		udp_data_ready = 1;

		usleep(100);
	}
}

//    IMU   =================================================================================

using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;
using namespace vn::math;
using namespace std;
void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index);

// Custom user data to pass to packet callback function
struct UserData
{
  // the vectornav device identifier
  int device_family;
  // frame id used only for Odom header.frame_id
  std::string map_frame_id;
  // frame id used for header.frame_id of other messages and for Odom child_frame_id
  std::string frame_id;
  // Boolean to use ned or enu frame. Defaults to enu which is data format from sensor.
  bool tf_ned_to_enu;
  bool frame_based_enu;
  // Initial position after getting a GPS fix.
  vec3d initial_position;
  bool initial_position_set = false;

  //Unused covariances initialized to zero's
//   boost::array<double, 9ul> linear_accel_covariance = {};
//   boost::array<double, 9ul> angular_vel_covariance = {};
//   boost::array<double, 9ul> orientation_covariance = {};

  // ROS header time stamp adjustments
  double average_time_difference{0};
  bool adjust_ros_timestamp{false};

  // strides
  unsigned int imu_stride;
  unsigned int output_stride;
};

bool optimize_serial_communication(std::string portName)
{
  int portFd = -1;

  portFd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

  if (portFd == -1) {
    return false;
  }

  struct serial_struct serial;
  ioctl(portFd, TIOCGSERIAL, &serial);
  serial.flags |= ASYNC_LOW_LATENCY;
  ioctl(portFd, TIOCSSERIAL, &serial);
  ::close(portFd);
  return true;
}
std::chrono::_V2::system_clock::time_point lastTime;

void* IMU_Comms( void * arg ){
	// Initialize IMU 
	UserData user_data;

	// Open the serial port to communicate with the VN100 IMU
    const std::string port = "/dev/imu";
	optimize_serial_communication(port);
    const int baudrate = 921600;
    VnSensor vs;
    vs.connect(port, baudrate);
    std::cout << "\n\nConnected to VN100 IMU on port " << port << std::endl;

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

	// vn::math::mat3f rot = vs.readReferenceFrameRotation();

	// for(int i=0;i<9;i++){
	// 	printf(" %f ", rot.e[i]);
	// }
	// printf("\n");

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

	vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

	// read IMU
	while(1){

		usleep(2500);
	}

}
long long getLastTime() {
  auto currentTime = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(currentTime - lastTime).count();
  lastTime = currentTime;
  return duration;
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

void BinaryAsyncMessageReceived(void * userData, Packet & p, size_t index)
{
	vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

	vec3f ypr = cd.yawPitchRoll();
	tello_ypr = ypr;
	// printf("YPR: %.2f,\t %.2f,\t %.2f\n",ypr[0], ypr[1], ypr[2]);
	// cout << getLastTime() << endl;
	// cout.flush();
}