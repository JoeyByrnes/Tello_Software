/* Copyright 2019-2021 The MathWorks, Inc. */

#ifndef udp_hpp
#define udp_hpp

#define UDP_MAX_WIDTH 65507

#include "ip.hpp"
#include <stdint.h>

#ifdef __cplusplus

#include <stdio.h>
#include <string>
#include <iostream>
#include <memory>

namespace slrealtime {
    namespace ip {
        namespace udp {

            class Socket : public slrealtime::ip::Socket {
            private:
                std::string arrayToString(uint8_t *AddArg);
            protected:
                struct Impl;
                std::unique_ptr<Impl> impl_;

            public:
                IPAPI Socket(std::string address, uint16_t port);
                IPAPI ~Socket();
                IPAPI void close();
                IPAPI size_t bytesToRead();
                IPAPI bool is_open();
                IPAPI void bind();
                IPAPI void addMulticastMembership(uint8_t *multicastAddArg,int multicastAddCount, uint8_t *multicastInterfaceArg,int multicastInterfaceCount );
                IPAPI void setRemoteEndpoint(uint8_t *remoteAddressArg, uint16_t remotePort);
                IPAPI void resetRemoteEndpoint(uint8_t *remoteAddressArg, uint16_t *remotePort);
                IPAPI size_t send(const char* sendBuf, size_t bytesToSend);
                IPAPI size_t receive(char* rcvBuf, size_t bytesToRcv, bool enablefilter, uint8_t* fmAddress, uint8_t *remoteAddress, uint16_t *remotePort);
                IPAPI size_t receive(char* rcvBuf, size_t bytesToRcv, bool enablefilter, uint8_t* fmAddress);
                IPAPI SocketType type() {
                    return SocketType::UDP;
                }
                IPAPI void disable();
            };

            class UDP_Packet{
                public:
                uint8_t IP_Address[4];
                uint16_t IP_Port;
                uint16_t Length;
                uint8_t Data[UDP_MAX_WIDTH];
            };

        }
    }
}

#endif

IPAPI void *  slrealtime_udp_init(std::string address, uint16_t port, bool useHostTargetConnection);

IPAPI void slrealtime_udp_term(std::string address, uint16_t port, bool useHostTargetConnection);

IPAPI uint16_t slrealtime_udp_rx(void * handle, uint8_t *buffer, bool rcvFromAny, uint8_t *fmAddArg, uint8_t *remoteAddress, uint16_t *remotePort);

IPAPI uint16_t slrealtime_udp_rx(void * handle, void * packet, bool rcvFromAny, uint8_t *fmAddArg);

IPAPI uint16_t slrealtime_udp_tx(void * handle, uint8_t *data, uint16_t dataLen, uint8_t *remoteAddress, uint16_t remotePort);

IPAPI uint16_t slrealtime_udp_tx(void * handle, void * packet, uint16_t maxDataLen);

#endif

