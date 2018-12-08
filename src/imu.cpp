#include <string>
#include <iostream>
#include <fstream>
#include "utils.h"

using namespace std;

int main(int argc, char* argv[])
{


	system("stty -F /dev/ttyACM1 115200");

	string line;
        ifstream arduino ("/dev/ttyACM1");


	if(arduino.is_open()) {
	  while(getline(arduino, line)) {
	    cout << line << endl;
	  }
	  arduino.close();
	}
	else  cout << "Unable to open file\n";


	return 0;
}
