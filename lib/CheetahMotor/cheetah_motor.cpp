#include "cheetah_motor.h"
#include <sys/time.h>
#include <libpcanfd.h>
#include <vector>


int write8BytesToCANFD(TPCANHandle channel, DWORD id, std::vector<BYTE> data)
{
    int status;

    // Create a CAN FD message
    pcanfd_msg message;
    message.id = id;
    message.type = PCAN_MESSAGE_STANDARD;
    message.data_len = data.size();
    for (int i = 0; i < data.size(); i++)
        message.data[i] = data[i];

    // Write the message to the specified CAN FD channel
    status = pcanfd_send_msg(channel, &message);

    return status;
}

CheetahMotor::CheetahMotor(){
    _pcan_bus = PCAN_PCIBUS1;
    Message.ID = 0x01;
    Message.LEN = 8;
    Message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    memset(Message.DATA, 0xff, sizeof(Message.DATA));
}

CheetahMotor::CheetahMotor(int can_id,int pcan_channel){
    _pcan_bus = pcan_channel;
    Message.ID = can_id;
    Message.LEN = 8;
    Message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    memset(Message.DATA, 0xff, sizeof(Message.DATA));
}

CheetahMotor::CheetahMotor(int can_id,int pcan_channel, int direction)
:CheetahMotor(can_id,pcan_channel)
{
    _direction = direction;
}

void CheetahMotor::disableMotor(){
    Message.DATA[0] = 0xFF;
    Message.DATA[1] = 0xFF;
    Message.DATA[2] = 0xFF;
    Message.DATA[3] = 0xFF;
    Message.DATA[4] = 0xFF;
    Message.DATA[5] = 0xFF;
    Message.DATA[6] = 0xFF;
    Message.DATA[7] = 0xFD;
    CAN_Write(_pcan_bus, &Message);
    _enabled = 0;
}

void CheetahMotor::enableMotor(){
    Message.DATA[0] = 0xFF;
    Message.DATA[1] = 0xFF;
    Message.DATA[2] = 0xFF;
    Message.DATA[3] = 0xFF;
    Message.DATA[4] = 0xFF;
    Message.DATA[5] = 0xFF;
    Message.DATA[6] = 0xFF;
    Message.DATA[7] = 0xFC;
    CAN_Write(_pcan_bus, &Message);
    _enabled = true;
}

bool CheetahMotor::isEnabled(){
    return _enabled;
}

void CheetahMotor::zeroEncoder(){
    Message.DATA[0] = 0xFF;
    Message.DATA[1] = 0xFF;
    Message.DATA[2] = 0xFF;
    Message.DATA[3] = 0xFF;
    Message.DATA[4] = 0xFF;
    Message.DATA[5] = 0xFF;
    Message.DATA[6] = 0xFF;
    Message.DATA[7] = 0xFE;
    CAN_Write(_pcan_bus, &Message);
}

void CheetahMotor::updateMotor(){
    Message.DATA[0] = _command.pos >> 8;
    Message.DATA[1] = _command.pos & 0x00FF;
    Message.DATA[2] = (_command.vel >> 4) & 0xFF;
    Message.DATA[3] = ((_command.vel & 0x000F) << 4) + ((_command.kp >> 8) & 0xFF);
    Message.DATA[4] = _command.kp & 0xFF;
    Message.DATA[5] = _command.kd >> 4;
    Message.DATA[6] = ((_command.kd & 0x000F)<<4) + (_command.feedforward >> 8);
    Message.DATA[7] = _command.feedforward & 0xff;
   CAN_Write(_pcan_bus, &Message);
}

void CheetahMotor::updateState(uint16_t pos, uint16_t vel, uint16_t cur){
    _state.pos = pos;
    _state.vel = vel;
    _state.cur = cur;
}

motor_state CheetahMotor::getMotorState(){
    return _state;
}
cmd_packet CheetahMotor::getCmdPacket(){
    return _command;
}

void CheetahMotor::setSoftLimits(int pos_min, int pos_max, int ff_min, int ff_max){
    _pos_min = pos_min;
    _pos_max = pos_max;
    _ff_min = ff_min;
    _ff_max = ff_min;
}

void CheetahMotor::limitPos(uint16_t &pos){
    if(pos > _pos_max){
        pos = _pos_max;
    }
    else if(pos < _pos_min){
        pos = _pos_min;
    }
}

void CheetahMotor::limitTorque(uint16_t &ff){
    if(ff > _ff_max){
        ff = _ff_max;
    }
    else if(ff < _ff_min){
        ff = _ff_min;
    }
}

void CheetahMotor::setKp(uint16_t kp){
    _command.kp = kp;
}

void CheetahMotor::setKd(uint16_t kd){
    _command.kd = kd;
}

void CheetahMotor::setPos(uint16_t pos){
    //limitPos(pos);
    _command.pos = pos;
}

void CheetahMotor::setVel(int vel){
    _command.vel = 2048+vel;
}

void CheetahMotor::setff(int ff){
    //limitTorque(ff);
    _command.feedforward = 2048+ff;
}

void CheetahMotor::addff(int ff){
    uint16_t new_ff = _command.feedforward + ff;
    //limitTorque(new_ff);
    _command.feedforward =new_ff;
}

