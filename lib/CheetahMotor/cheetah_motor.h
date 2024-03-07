#ifndef __CHEETAH_MOTOR_H__
#define __CHEETAH_MOTOR_H__
#include <stdint.h>
#include "PCANBasic.h"
#include <string.h>

// #define ENCODER_COUNTS 16384
// #define POSITION_MIDPOINT (double)32768
// #define COUNTS_TO_RADIANS (double)(TWO_PI/(double)ENCODER_COUNTS)
// #define COUNTS_TO_DEGREES (double)(360.0/(double)ENCODER_COUNTS)
// #define DEGREES_TO_COUNTS (double)((double)ENCODER_COUNTS/360.0)
#define DEGREES_TO_RADIANS (double)0.01745329252

struct cmd_packet
{
    uint16_t vel = 2048;
    uint16_t feedforward = 2048;
    uint16_t kp = 1000;
    uint16_t kd = 100;
    uint16_t pos = 32767;
    
};
struct motor_state
{
    uint16_t vel = 2048;
    uint16_t pos = 32767;
    uint16_t cur = 2048;
    
};

class CheetahMotor
{
    public:
        CheetahMotor();
        CheetahMotor(int can_id,int pcan_channel); // pcan_channel = PCAN_PCIBUS1, PCAN_PCIBUS2 , etc...
        CheetahMotor(int can_id,int pcan_channel, int direction);

        void disableMotor();
        void enableMotor();
        void zeroEncoder();
        void updateMotor();
        void updateState(uint16_t pos, uint16_t vel, uint16_t cur);

        void setPos(uint16_t pos);
        void setKp(uint16_t kp);
        void setKd(uint16_t kd);
        void setff(int ff);
        // void addff(int ff);
        void setVel(int vel);

        void setSoftLimits(int pos_min, int pos_max, int ff_min, int ff_max);

        bool isEnabled();

        motor_state getMotorState();
        cmd_packet getCmdPacket();
        int _trajectory_setpoint = 0;

    private:

        void limitPos(uint16_t &pos);
        void limitTorque(uint16_t &ff);

        int _pcan_bus;
        //CAN_FRAME _can_msg;
        TPCANMsg Message;
        motor_state _state;
        cmd_packet _command;
        bool _enabled = 0;
        int _direction = 1;

        int _pos_min=0;
        int _pos_max=65535;
        int _ff_min=1048;
        int _ff_max = 3048;

};

#endif