#include "serial/serial.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <fstream>
#include "config.h"

class GPS {
public:
    GPS(Config conf);
    ~GPS() = default; 

    void Start();
    void Stop();

private:

    std::thread serial_reader;
    std::thread ntrip_reader;

    void handle_gbs(std::string msg);
    void handle_gst(std::string msg);
    void handle_gsv(std::string msg);
    void handle_gll(std::string msg);
    void handle_gga(std::string msg);
    void handle_gsa(std::string msg);
    void handle_rmc(std::string msg);
    void handle_vtg(std::string msg);


    bool init_ntrip();
    bool send_ntrip(const char gpgga[]);
    void recv_ntrip();

    std::string ntrip_host;
    int ntrip_port;
    std::string ntrip_mount; 
    std::string ntrip_user;
    std::string ntrip_pw;

    serial::Serial ser;
    int m_sock;  
    bool running;

    unsigned long last_nmea;

    std::ofstream gps_file;

};
