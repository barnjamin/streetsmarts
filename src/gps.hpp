#pragma once


#include <string>
#include <iostream>
#include <sstream>
#include <bitset>
#include <fstream>
#include <ublox/ublox.h>
#include <mutex>

// https://github.com/GAVLab/ublox

class GPS {
    ublox::Ublox sensor;

    ublox::NavStatus status;
    ublox::NavSol pos;
  
    std::string port;
    int         baud;
    int         rate;

    bool         running;
    unsigned int interval;
    std::thread  poll_thread;


    std::mutex  mtx;
public:
    GPS(){
        port = std::string("/dev/ttyACM1");
        baud = 115200;
        rate = 1;  

        sensor.set_nav_status_callback([&](ublox::NavStatus &s, double &ts){
            status = s;
        });

        sensor.set_nav_solution_callback([&](ublox::NavSol& p, double &ts){
            std::lock_guard<std::mutex> lck(mtx);
            pos = p;
        });
    }

    GPS(const GPS& g){
        sensor   = g.sensor;
        status   = g.status;
        pos      = g.pos;
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

        // wait for 3D fix
        while (status.fixtype != 0x03) usleep(200*1000);

        running = true;
        interval = uint8_t(1000/rate);

        return true;
    };

    std::string GetPosition(){
        std::lock_guard<std::mutex> lck(mtx);

        std::stringstream line;
        line << pos.ecefX << "," << pos.ecefY << "," << pos.ecefZ
            << "," << pos.pAcc << "," << pos.sAcc << "," << pos.pDop
            << "," << pos.ecefVX << "," << pos.ecefVY << "," << pos.ecefVZ 
            << std::endl;

        std::cout << "gps sol: "<< line.str();

        return line.str();
    }

    void Stop() { 
        if(!running){
            return;
        }

        running = false;
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
            while(Sleep()){
                std::cout << "Polling" << std::endl;
                sensor.PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_SOL);
            }
        });
    };
};
