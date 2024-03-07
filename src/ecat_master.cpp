#include "ecat_master.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

void* ecat_comms( void * arg )
{
	
	auto arg_tuple_ptr = static_cast<std::tuple<void*, void*, int, int>*>(arg);
	int period = std::get<2>(*arg_tuple_ptr);

	int core = sched_getcpu();
	int policy;
	sched_param param;
    pthread_t current_thread = pthread_self();
    int result = pthread_getschedparam(current_thread, &policy, &param);
	int priority = param.sched_priority;
	printf("EtherCAT Communication thread running on core %d, with priority %d\n", core, priority);

    // ETHERCAT INITIAL SETUP

    int i, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    /* initialise SOEM, bind socket to ifname */
   if (ec_init("enp11s0"))
   {
      printf("ec_init on %s succeeded.\n","enp11s0");
      /* find and auto-config slaves */


        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            if (forceByteAlignment)
            {
            ec_config_map_aligned(&IOmap);
            }
            else
            {
            ec_config_map(&IOmap);
            }

            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 200;
            /* wait for all slaves to reach OP state */
            do
            {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
            printf("Operational state reached for all slaves.\n");
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            while(1)
            {
                // simple example of setting bits:
                // ec_slave[1].outputs[0] = 1;
                // ec_slave[1].outputs[1] = 0;
                // ec_slave[1].outputs[2] = 1;
                // ec_slave[1].outputs[3] = 0;
                // ec_slave[1].outputs[4] = 1;
                // ec_slave[1].outputs[5] = 0;
                // ec_slave[1].outputs[6] = 1;
                // ec_slave[1].outputs[7] = 0;

                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                // ETHERCAT PDO COMMUNICATION

            }
        }

    }
    else{
        printf('r',"EtherCAT port failed to open");
    }

}