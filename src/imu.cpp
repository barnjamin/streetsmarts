#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include "utils.h"
#include "json.hpp"

using namespace std;
using namespace nlohmann;

int main(int argc, char* argv[])
{
	Config conf;
	conf.parseArgs(argc, argv);

	std::ostringstream sstream;
	sstream << "stty -F " << conf.imu_src << " 115200";
	cout << sstream.str() << endl;
	system((sstream.str()).c_str());

	cout << "Attempting to open file\n";

	string line;
	ifstream arduino (conf.imu_src);

	if(!arduino.is_open()) {
	  cout << "Unable to open file\n";
	}

	cout << "Opened, discarding lines\n";

	for(int discard=0; discard<3; discard++){
	  getline(arduino, line); 
	} 

	cout << "Reading data...\n";

	while(getline(arduino, line)) {
  	  //cout << line << endl;

	  if(line.length()<16){ //Garbage
	    continue; 
	  }

	  auto j = json::parse(line);

	  auto a = j["accel"];
	  float a_x = a.value("x", 0.0);
	  float a_y = a.value("y", 0.0);
	  float a_z = a.value("z", 0.0);

	  cout << "Accel: " << a_x << " " << a_y << " " << a_z << endl;

	  //auto q = j["quat"];
	  //float q_x = q.value("x", 0.0);
	  //float q_y = q.value("y", 0.0);
	  //float q_z = q.value("z", 0.0);
	  //float q_w = q.value("w", 0.0);
	  //cout << "Quat: " << q_x << " " << q_y << " " << q_z  << " " << q_w << endl;

	}

	cout << "Done?\n";

	arduino.close();
	return 0;
}
