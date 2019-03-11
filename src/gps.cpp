#include <string>
#include <iostream>
#include <sstream>
#include <bitset>
#include <fstream>
#include <ublox/ublox.h>
using namespace ublox;
using namespace std;

NavStatus cur_nav_status;
NavPosLLH cur_pos;

void NavigationStatusCallback(NavStatus &status, double &time_stamp) {
    cur_nav_status = status;
}

void NavPosCallback(NavPosLLH& pos, double &time_stamp) {
    std::cout << pos.latitude_scaled/1e7 << "," << pos.longitude_scaled/1e7 << " " << pos.height/1000.0 << std::endl;
    cur_pos = pos;
}

int main(int argc, char **argv)
{
   Ublox my_gps;

   std::string port("/dev/ttyACM1");
   int baudrate=115200;

   my_gps.set_nav_status_callback(NavigationStatusCallback);
   my_gps.set_nav_position_llh_callback(NavPosCallback);

   // Connect to Receiver
   bool result = my_gps.Connect(port,baudrate);
   if (!result) {
       cout << "Failed to connect." << endl;
       return -1;
   }

   // Make sure receiver gets a fix
   while (cur_nav_status.fixtype !=0x03) // wait for 3D fix
       usleep(200*1000);

   while(1)
       my_gps.PollMessage(MSG_CLASS_NAV, MSG_ID_NAV_POSLLH);
       usleep(1000 * 1000);

   // Disconnect receiver
   std::cout << "Disconnecting from receiver." << endl;
   my_gps.Disconnect();

   return 0;
}
