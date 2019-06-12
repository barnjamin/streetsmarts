#pragma once
#include <boost/asio.hpp>

class NtripClient 
{

public:
    NtripClient();
    NtripClient(std::string host, int port, 
            std::string mount, std::string user, 
            std::string pw);
    ~NtripClient();

    bool start(boost::asio::serial_port& serial);
    bool stop();


private:
    void sendRtcm();
    std::string connString();
    std::string authString();


    boost::asio::serial_port* serial_;

    std::string host_;
    int port_;
    std::string mount_;
    std::string user_;
    std::string pw_;

    std::thread rtcm;
};
