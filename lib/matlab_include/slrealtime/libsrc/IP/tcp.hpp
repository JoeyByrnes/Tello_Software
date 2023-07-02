/* Copyright 2019 The MathWorks, Inc. */

#ifndef tcp_hpp
#define tcp_hpp

#include	"ip.hpp"

namespace slrealtime {
    namespace ip {
        namespace tcp {
            class Socket : public slrealtime::ip::Socket
            {
            protected:
                struct Impl;
                std::unique_ptr<Socket::Impl> bImpl_;
            public:
                IPAPI Socket(std::string address, uint16_t port);
                IPAPI ~Socket();
                IPAPI bool connected() const;
                IPAPI virtual void reset() = 0;
                IPAPI virtual void connect() = 0;
                IPAPI void close();
                IPAPI size_t send(const char* sendBuf, size_t bytesToSend);
                IPAPI size_t receive(char* rcvBuf, size_t bytesToRcv);
                IPAPI void disable();
            };

            class Server : public Socket
            {
            private:
                struct Impl;
                std::unique_ptr<Server::Impl> sImpl_;
            public:
                IPAPI Server(std::string address, uint16_t port);
                IPAPI ~Server();
                IPAPI void reset();
                IPAPI void connect();
                IPAPI SocketType type(){return SocketType::TCPServer;}
            };

            class Client : public Socket
            {
            private:
                struct Impl;
                std::unique_ptr<Client::Impl> cImpl_;
            public:
                IPAPI Client(std::string clientAddress, uint16_t clientPort, std::string remoteAddress, uint16_t remotePort);
                IPAPI ~Client();
                IPAPI void reset();
                IPAPI void connect();
                IPAPI SocketType type(){return SocketType::TCPClient;}
            };
        }
    }
}

#endif