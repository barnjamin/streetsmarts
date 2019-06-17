#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include <thread>

#include <sys/types.h>
#include <sys/socket.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mutex>


bool running = false;
std::mutex mtx;


void PrintChar(const char *src, const int &len) {
  for (int i = 0; i < len; ++i) {
    printf("%c", (unsigned char)src[i]);
  }
  printf("\n");
}

void handle_gsv(std::string msg) {
    
}

void handle_gll(std::string msg) {

}

void handle_gga(std::string msg){
    //pass to client connection
    std::cout << msg;
}

void handle_gsa(std::string msg){

}

void handle_rmc(std::string msg){

}

void handle_vtg(std::string msg){

}

void init_ntrip(const char gpgga[], serial::Serial& s) {
  running = true;

  int ret;
  int m_sock;
  time_t start, stop;
  char recv_buf[1024] = {0};
  char request_data[1024] = {0};
  char userinfo[] = "YmFybmphbWluOkNvYXN0ZXJvbmllMQ==";
  
  char server_ip[] = "161.11.223.1";
  int server_port = 8080;
  char mountpoint[] = "near_msm";
  char user[] = "barnjamin";
  char passwd[] = "Coasteronie1";


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
           "\r\n",
           mountpoint, userinfo);

  std::cout << "Connecting.." ;
  m_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (m_sock == -1) {
    std::cout <<"create socket fail\n" ;
    exit(1);
  }

  // Connect to caster.
  if (connect(m_sock, reinterpret_cast<struct sockaddr *>(&server_addr),
              sizeof(struct sockaddr_in)) < 0) {
      std::cout << "connect caster failed!!!\n";
    exit(1);
  }
  std::cout << "Connected" << std::endl;

  int flags = fcntl(m_sock, F_GETFL);
  fcntl(m_sock, F_SETFL, flags | O_NONBLOCK);

  // Send request data.
  if (send(m_sock, request_data, strlen(request_data), 0) < 0) {
    printf("send request failed!!!\n");
    exit(1);
  }

  // Wait for request to connect caster success.
  while (1) {
    ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0);
    if ((ret > 0) && !strncmp(recv_buf, "ICY 200 OK\r\n", 12)) {
      ret = send(m_sock, gpgga, strlen(gpgga), 0);
      if (ret < 0) {
        printf("send gpgga data fail\n");
        exit(1);
      }
      std::cout << "send gpgga data ok\n";
      break;
    } else if (ret == 0) {
      printf("remote socket close!!!\n");
      goto out;
    } else {
        std::cout << recv_buf;
    }
  }
  std::cout << "Here" << std::endl;

  // Receive data that returned by caster.
  start = time(NULL);
  while (1) {
    memset(recv_buf, 0, sizeof(recv_buf));
    ret = recv(m_sock, recv_buf, sizeof(recv_buf), 0);
    if (ret > 0) {
      stop = time(NULL);
      std::cout << "Got Data: " << int(ret) << std::endl;
      auto wrote = s.write(std::string(recv_buf, ret));
      std::cout << "Wrote Data: " << int(wrote) << std::endl;

      s.flush();
      //PrintChar(recv_buf, ret);
      start = time(NULL);
    } else if (ret == 0) {
      printf("remote socket close!!!\n");
      break;
    }
  }

out:
  close(m_sock);
  running = false;
  return;
}




int main(int argc, char **argv) {
      
      serial::Serial s("/dev/ttyACM1", 115200, serial::Timeout::simpleTimeout(1000));
      //char gpgga[] = "$GNGGA,125515.00,4304.19501,N,07606.23080,W,1,12,0.56,146.1,M,-34.4,M,,*7A\r\n";

      std::thread t([&](){
          while(true){
              std::string msg;

              mtx.lock();
              s.readline(msg);
              mtx.unlock();

              if(msg.size()<6){
                continue; 
              }

              std::string prefix = msg.substr(1,5);
              if(prefix.compare( "GAGSV")==0)
                  handle_gsv(msg);
              else if(prefix.compare( "GBGSV")==0)
                  handle_gsv(msg);
              else if(prefix.compare( "GLGSV")==0)
                  handle_gsv(msg);
              else if(prefix.compare( "GPGSV")==0)
                  handle_gsv(msg);
              else if(prefix.compare( "GNGGA")==0){
                  handle_gga(msg);
              } else if(prefix.compare( "GNGLL")==0)
                  handle_gll(msg);
              else if(prefix.compare( "GNGSA")==0)
                  handle_gsa(msg);
              else if(prefix.compare( "GNRMC")==0)
                  handle_rmc(msg);
              else if(prefix.compare( "GNVTG")==0)
                  handle_vtg(msg);
              else std::cout << "unhanled: " << prefix << std::endl;
          }

      });



      while(true){
          std::string msg;

          mtx.lock();
          s.readline(msg);
          mtx.unlock();

          if(msg.size()<6){
            continue; 
          }

          //std::cout << msg;
          std::string prefix = msg.substr(1,5);
          if(prefix.compare( "GNGGA")==0 && ! running){
              init_ntrip(msg.c_str(), s);
          }
      }

      t.join();
}



