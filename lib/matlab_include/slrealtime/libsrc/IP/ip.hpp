/* Copyright 2019-2021 The MathWorks, Inc. */

#ifndef ip_hpp
#define ip_hpp

#if defined(MATLAB_MEX_FILE)

#if defined (_WIN32)
#define IPAPI __declspec(dllexport)
#elif defined (__linux)
#define IPAPI __attribute__((visibility("default")))
#endif

#else
#define IPAPI
#endif

#ifdef __cplusplus

#include    <stdio.h>
#include    <string>
#include    <memory>
#include    <algorithm>

namespace slrealtime {
    namespace ip {
       
        #define HostIPAddrForUseHostTargetConn "0.0.0.0"

        enum class SocketType {
            UDP = 0,
            TCPClient,
            TCPServer
        };

        IPAPI void ThrowWarning(std::string);
        IPAPI std::string Trim(std::string);

        IPAPI std::string GetTargetIPAddr();

        class Socket
        {
        protected:
            std::string localAddress_;
            uint16_t port_ = 0;
            bool disabled_ = false;
           
        public:
            IPAPI Socket(std::string localAddress, uint16_t port);
            IPAPI virtual ~Socket() {}
            IPAPI std::string localAddress() const { return localAddress_; }
            IPAPI uint16_t port() const { return port_; }
            IPAPI virtual SocketType type() = 0;
            IPAPI virtual void disable() = 0;
        };

    }
}

#endif
#endif

