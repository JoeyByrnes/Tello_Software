/* Copyright 2021 The MathWorks, Inc. */

#ifndef socket_Factory_hpp
#define socket_Factory_hpp

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

#include    <map>
#include    <atomic>
#include    "ip.hpp"

namespace slrealtime {
    namespace ip {

        class SocketFactory
        {
        public:
            struct entry {
                Socket*             socket_;
                std::atomic<int>    useCount_;
            };

            static IPAPI Socket* getSocket(std::string localAddress, uint16_t port);
            static IPAPI Socket* createSocket(SocketType type, std::string localAddress, uint16_t port, std::string remoteAddress, uint16_t remotePort);
            static IPAPI Socket* createSocket(SocketType type, std::string localAddress, uint16_t port);
            static IPAPI void releaseSocket(std::string localAddress, uint16_t port);
        private:
            SocketFactory() {}
            ~SocketFactory() {}
            static std::map<std::pair<std::string, uint16_t>, entry> sockets_;
        };
       
    }
}

#endif

#endif