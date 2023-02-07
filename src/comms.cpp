#include "comms.h"

extern uint16_t encoders[10];

extern int position_initialized[10];
extern uint16_t encoder_positions[10];
extern uint16_t encoder_offsets[10];

extern int udp_data_ready;
extern char udp_control_packet[UDP_MAXLINE];

extern pthread_mutex_t mutex_CAN_recv;
extern int can_data_ready_to_save;

void* rx_CAN( void * arg ){
	
    
	TPCANStatus Status;
	TPCANMsg Message;
	int pcd = *((int*)arg);
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

				pthread_mutex_unlock(&mutex_CAN_recv);

			}

		}
		// if (Status != PCAN_ERROR_OK) {
		// 	pthread_exit(NULL);
		// 	printf("Error on CAN bus %d\n",pcd-64);
		// 	break;
		// }
		// unsigned int id = Message.DATA[0];
		// unsigned int pos = (Message.DATA[1] << 8) + Message.DATA[2];
		// unsigned int vel  = (Message.DATA[3] << 4) + ((Message.DATA[4] & 0xF0) >> 4);
		// unsigned int cur = ((Message.DATA[4] & 0x0F) << 8) + Message.DATA[5];
		
		// pthread_mutex_lock(&mutex_CAN_recv);

		// encoder_positions[id-1] = pos;
		// if(!position_initialized[id-1]){
		// 	encoder_offsets[id-1] = pos;
		// 	position_initialized[id-1] = 1;
		// 	//printf("ID: %d , OFFSET: %d \n",id,pos);
		// 	printf("Motor %d Connected\n", id);
		// }

		// encoders[id-1] = pos;

		// pthread_mutex_unlock(&mutex_CAN_recv);

		usleep(50);
	}
    return NULL;
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

