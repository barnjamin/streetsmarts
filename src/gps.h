#include "serial/serial.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <fstream>

class GPS {
public:
    GPS(std::string port, std::string log_file);
    ~GPS() = default; 

    void Start();
    void Stop();

private:

    std::thread serial_reader;
    std::thread ntrip_reader;

    void handle_gsv(std::string msg);
    void handle_gll(std::string msg);
    void handle_gga(std::string msg);
    void handle_gsa(std::string msg);
    void handle_rmc(std::string msg);
    void handle_vtg(std::string msg);


    bool init_ntrip();
    bool send_ntrip(const char gpgga[]);
    void recv_ntrip();

    serial::Serial ser;
    int m_sock;  
    bool running;

    unsigned long last_nmea;

    std::ofstream gps_file;

};
