#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>

#include "serial/serial.h"


#include <mutex>
#include <thread>

#include "gps.h"
#include "config.h"
#include "utils.h"

#include <boost/algorithm/string.hpp>

GPS::GPS(Config conf) :
    ser(conf.gps_port, 115200, serial::Timeout::simpleTimeout(1000)),
    ntrip_host(conf.ntrip_host),
    ntrip_port(conf.ntrip_port),
    ntrip_mount(conf.ntrip_mount),
    ntrip_user(conf.ntrip_user),
    ntrip_pw(conf.ntrip_pw)
{
    gps_file.open(conf.GPSFile());
}

void GPS::Start(){
    init_ntrip();

    serial_reader = std::thread([&](){
        while(true){
            std::string msg;

            ser.readline(msg);

            if(msg.size()<6) continue; 

            std::string prefix = msg.substr(1,5);
            if(prefix.compare("GAGSV")==0)
                handle_gsv(msg);
            else if(prefix.compare("GBGSV")==0)
                handle_gsv(msg);
            else if(prefix.compare("GLGSV")==0)
                handle_gsv(msg);
            else if(prefix.compare("GPGSV")==0)
                handle_gsv(msg);
            else if(prefix.compare("GNGGA")==0)
                handle_gga(msg);
            else if(prefix.compare("GNGLL")==0)
                handle_gll(msg);
            else if(prefix.compare("GNGSA")==0)
                handle_gsa(msg);
            else if(prefix.compare("GNRMC")==0)
                handle_rmc(msg);
            else if(prefix.compare("GNVTG")==0)
                handle_vtg(msg);
            else if(prefix.compare("GNGBS")==0)
                handle_gbs(msg);
            else if(prefix.compare("GNGST")==0)
                handle_gst(msg);
            else
                std::cout << msg << std::endl;
        }
    });

    ntrip_reader = std::thread([&](){
        recv_ntrip();
    });

}

void GPS::Stop() {
    serial_reader.join();
    ntrip_reader.join();
}

double get_decimal(std::string val){
    double dec = std::stod(val);
    return dec / 100;
}

void GPS::handle_gst(std::string msg) {
    //std::cout << msg;
    return;
}

void GPS::handle_gbs(std::string msg) {
    //std::cout << msg;
    return;
}

void GPS::handle_gsv(std::string msg) {
    //std::cout << msg;
    return;
}

void GPS::handle_gll(std::string msg) {
    //std::cout << msg;
    return;
}

void GPS::handle_gga(std::string msg){
    //pass to client connection
    //std::cout << msg;

    if (get_timestamp() - last_nmea > 30000){
        std::cout << "Sending nmea\n" ;
        if(!send_ntrip(msg.c_str())){
            std::cerr << "Failed to send ntrip correction" << std::endl; 
        }
        last_nmea = get_timestamp();
    }

    std::vector<std::string> results;
    boost::split(results, msg, [](char c){return c == ',';});

    gps_file << std::setprecision(10);
    gps_file << get_timestamp() << "," << get_decimal(results[2]) << "," 
            << get_decimal(results[4]) * -1.0 << "," << results[9] << "," 
            << results[8] << "," << results[6] << std::endl;

    return;
}

void GPS::handle_gsa(std::string msg){
    //std::cout << msg;
    return;
}

void GPS::handle_rmc(std::string msg){
    //std::cout << msg;
    return;
}

void GPS::handle_vtg(std::string msg){
    //std::cout << msg;
    return;
}


bool GPS::init_ntrip() {
  int ret;
  char recv_buf[1024] = {0};
  char request_data[1024] = {0};
  
  //TODO: set from conf
  char userinfo[] = "YmFybmphbWluOkNvYXN0ZXJvbmllMQ==";
  char server_ip[] = "161.11.223.1";
  //char mountpoint[] = ntrip_mount.c_str();
  char mountpoint[] = "near_msm";

  int server_port = 8080;

  last_nmea = 0;

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(server_port);
  server_addr.sin_addr.s_addr = inet_addr(server_ip);

   // Generate request data format of ntrip.
  snprintf(request_data, 1023,
           "GET /%s HTTP/1.1\r\n"
           "User-Agent: StreetSmarts/1.0\r\n"
           "Accept: */*\r\n"
           "Connection: close\r\n"
           "Authorization: Basic %s\r\n"
           "\r\n", mountpoint, userinfo);

  std::cout << "connecting to ntrip..." << std::endl;
  m_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (m_sock == -1) {
    std::cerr <<"create socket fail\n" ;
    return false;
  }

  // Connect to caster.
  if (connect(m_sock, reinterpret_cast<struct sockaddr *>(&server_addr),
              sizeof(struct sockaddr_in)) < 0) {
    std::cerr << "connect caster failed!!!\n";
    return false;
  }

  std::cout << "connected to ntrip" << std::endl;

  int flags = fcntl(m_sock, F_GETFL);
  fcntl(m_sock, F_SETFL, flags | O_NONBLOCK);

  // Send request data.
  if (send(m_sock, request_data, strlen(request_data), 0) < 0) {
    std::cerr << "send request failed!!!\n";
    return false;
  }
  
  while (1) {
    ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0);
    if ((ret > 0) && !strncmp(recv_buf, "ICY 200 OK\r\n", 12)) {
        return true;
    } else if (ret == 0) {
        return false;
    } else {
      std::cout << recv_buf;
    }
  }

  return true;
}


bool GPS::send_ntrip(const char gpgga[]){
  return send(m_sock, gpgga, strlen(gpgga), 0) > 0;
}

void GPS::recv_ntrip(){
  int ret;
  char recv_buf[1024] = {0};
  while (1) {
    ret = recv(m_sock, recv_buf, sizeof(recv_buf), 0);
    if (ret > 0) {
        ser.write(std::string(recv_buf, ret));
    } else if (ret == 0) {
        break;
    }
  }
}



