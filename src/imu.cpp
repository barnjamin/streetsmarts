#include <string>
#include <iostream>
#include <fstream>
#include "utils.h"
#include "json.hpp"

using namespace std;

int main(int argc, char* argv[])
{


	system("stty -F /dev/ttyACM1 115200");

	string line;
        ifstream arduino ("/dev/ttyACM1");




	if(!arduino.is_open()) {
	  cout << "Unable to open file\n";

	}

	for(int discard=0; discard<3; discard++){
	  getline(arduino, line); 
	} 

	while(getline(arduino, line)) {
	  if(line.length()<10){ //Garbage
	    continue; 
	  }

	  auto j = nlohmann::json::parse(line);
	  auto f = j["velo"]["x"].get<float>();
	  cout << f << endl;

	  cout<<j.dump()<<endl;
	}


	arduino.close();

	return 0;
}
