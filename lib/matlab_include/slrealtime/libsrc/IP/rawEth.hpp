/* Copyright 2021-2022 The MathWorks, Inc. */

#ifndef RawEth_h__
#define RawEth_h__

#include <iostream>
#include <arpa/inet.h>
#include <fcntl.h>
#include <net/bpf.h>
#include <net/if.h>
#include <net/if_ether.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>
#include <utilities.hpp>
#include <vector>
#include <queue>
#include <sstream>
#include <errno.h>
#include <pcap.h>

#ifdef __cplusplus

namespace {

struct IamRoot {
    IamRoot() {
        old_gid_ = getegid();
        old_uid_ = geteuid();

        if (setegid(0) < 0) {
            std::stringstream ss;
            ss << "Failed to become root group: " << strerror(errno);
            throw std::runtime_error(ss.str());
        }

        if (seteuid(0) < 0) {
            std::stringstream ss;
            ss << "Failed to become root user: " << strerror(errno);
            throw std::runtime_error(ss.str());
        }
    }

    ~IamRoot() {
        if (setegid(old_gid_) < 0) {
           
        }

        if (seteuid(old_uid_) < 0) {
           
        }
    }

    gid_t old_gid_;
    uid_t old_uid_;
};
}

namespace slrealtime {
    namespace ip {
        namespace ethernet {
            class RawEth {
            public:
                RawEth(const char* interface, const bool overwriteSrcMAC, const int receiveWidth, const char* filter_string);
                ~RawEth();
                void writePacket(const uint8_t* data, uint16_t length);
                void readPacket(uint8_t* data, uint16_t* length);
                void set_filter(const char* filter_string);
            private:
                int bpf;
                pcap_t * pcap_handle_;
                int receiveWidth_;
                char * buf = NULL;
                char * p = NULL;
                ssize_t bytesRead = 0;
                size_t blen = 1;
                std::string interface_;
            };
        }
    }
}

#endif

#ifdef __cplusplus
extern "C" {
#endif

uint64_t initializeRawEth(const char* interface, const bool overwriteSrcMAC, const int receiveWidth, const char* filter_string);

void terminateRawEth(uint64_t handle);

void readRawEth(uint64_t handle, uint8_t* data, uint16_t* length);

void writeRawEth(uint64_t handle, const uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif

