#pragma once

class NtripClient 
{

public:
    NtripClient(std::string host, int port, 
            std::string mount, std::string user, 
            std::string pw);

    ~NTripClient();

    bool start();
    bool stop();


private:
    void sendRtcm();
    std::string connString();
    std::string authString();


    std::string host_;
    int port_;
    std::string mount_;
    std::string user_;
    std::string pw_;

    std::thread rtcm;
};
