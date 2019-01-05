#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>

#include <GL/glut.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include "utils.h" 
#include "pose.h"
#include "display.h"

using namespace std;

int main(int argc, char * argv[])
{

    Config conf;
    conf.parseArgs(argc, argv);


    string dirname =  "dumps/20190105102137"; //get_latest_dump_dir();

    ifstream imufile(dirname + "/imu.csv");

    Pose p(30);
    Display d(argc, argv, &p);
    d.start();

    while(imufile){
        string s;
        if(!getline(imufile, s)) break;

        istringstream ss( s );
        vector <string> record;

        while(ss) {
            string s;
            if (!getline( ss, s, ',' )) break;
            record.push_back( s );
        }

        auto idx = record.at(0);

        vector<double> accel{atof(record.at(1).c_str()), atof(record.at(2).c_str()), atof(record.at(3).c_str())} ;
        vector<double> gyro{atof(record.at(4).c_str()), atof(record.at(5).c_str()), atof(record.at(6).c_str())} ;
        

        p.Update(accel, gyro);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    d.stop();
}
