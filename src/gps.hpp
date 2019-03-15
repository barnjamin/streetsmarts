#pragma once


#include <string>
#include <iostream>
#include <sstream>
#include <bitset>
#include <fstream>
#include <ublox/ublox.h>

// https://github.com/GAVLab/ublox

class GPS {
    ublox::Ublox sensor;

    ublox::NavStatus status;
    ublox::NavPosLLH pos;
  
    std::string port;
    int baud;

public:
    GPS(){
        port = std::string("/dev/ttyACM1");
        baud = 115200;

        sensor.set_nav_status_callback([&](ublox::NavStatus &s, double &ts){
            std::cout<<"status"<< std::endl;
            status = s;
        });

        sensor.set_nav_position_llh_callback([&](ublox::NavPosLLH& p, double &ts){
            std::cout<<"pos"<< std::endl;
            pos = p;
        });
    }

    GPS(const GPS& g){
        sensor = g.sensor;
        status = g.status;
        pos = g.pos;
        port = g.port;
        baud = g.baud;
    }

    ~GPS(){ Disconnect(); };

    bool Connect(){ return sensor.Connect(port,baud); };
    void Disconnect() { sensor.Disconnect(); };

    void Start() {
        // nav status at 1 Hz
        sensor.ConfigureMessageRate(0x01,0x03,1); 

        // Make sure receiver gets a fix
        while (status.fixtype != 0x03){// wait for 3D fix
            usleep(200*1000);
        } 

        while(1){
            sensor.PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_POSLLH);
            usleep(1000 * 1000);
        }
    };

};
