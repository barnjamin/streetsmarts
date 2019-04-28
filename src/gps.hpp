#pragma once


#include <string>
#include <iostream>
#include <sstream>
#include <bitset>
#include <fstream>
#include <ublox/ublox.h>
#include <mutex>
#include "utils.h"

// https://github.com/GAVLab/ublox

inline void DefaultInfoMsgCallback(const std::string &msg) {
    //std::cout << "Ublox Info: " << msg << std::endl;
}

using namespace GeographicLib;

class GPS {
    ublox::Ublox sensor;

    ublox::NavStatus status;
  
    std::string port;
    int         baud;
    int         rate;

    bool         running;
    unsigned int interval;
    std::thread  poll_thread;

    std::ofstream gps_file;

    std::mutex  mtx;
public:
    GPS(std::string log_file){
        port = std::string("/dev/ttyACM1");
        baud = 115200;
        rate = 1;  

        gps_file.open(log_file);

        sensor.log_info_ = DefaultInfoMsgCallback;
        sensor.log_debug_ = DefaultInfoMsgCallback;

        sensor.set_nav_status_callback([&](ublox::NavStatus &s, double &ts){
            status = s;
        });

        Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

        sensor.set_nav_solution_callback([&](ublox::NavSol& pos, double &ts){
            double lat, lon, h;

            earth.Reverse(pos.ecefX/100.0, pos.ecefY/100.0, pos.ecefZ/100.0, lat, lon, h);

            gps_file << get_timestamp() << "," << lat << "," << lon << "," <<  h
                << "," << pos.pAcc << "," << pos.sAcc << "," << pos.pDop
                << "," << pos.ecefVX << "," << pos.ecefVY << "," << pos.ecefVZ 
                << std::endl;
        });
    }

    GPS(const GPS& g){
        sensor   = g.sensor;
        status   = g.status;
        port     = g.port;
        baud     = g.baud;
        running  = g.running;
        interval = g.interval;
    }

    ~GPS(){ Stop(); };

    bool Connect(){ 
        if(!sensor.Connect(port,baud)) return false; 

        // nav status at 1 Hz
        sensor.ConfigureMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_STATUS, 1); 

        // nav sol at $rate
        sensor.ConfigureMessageRate(MSG_CLASS_NAV, MSG_ID_NAV_SOL, rate); 

        std::cout << "Waiting for fix" << std::endl;
        // wait for 3D fix
        while (status.fixtype != 0x03) usleep(200*1000);

        running = true;
        interval = uint8_t(1000/rate);

        return true;
    };

    void Stop() { 
        if(!running) return;

        running = false;
        gps_file.close();

        poll_thread.join();
        sensor.Disconnect(); 
    };


    bool Sleep(){
        if(running){
            usleep(interval * 1000);
            return true;
        }
        return false;
    }

    void Start() {
        poll_thread = std::thread([&](){
            while(Sleep()) sensor.PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_SOL);
        });
    };
};
