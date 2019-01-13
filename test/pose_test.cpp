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

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align(RS2_STREAM_COLOR);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

    rs2::pipeline_profile profile = pipe.start(cfg);

    Pose p(200);
    Display d(argc, argv, &p);
    d.start();

    rs2::frameset frameset;
    rs2_vector accel_data, gyro_data;

    while(true) {
        frameset = pipe.wait_for_frames();

        auto accel_frame = frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        auto gyro_frame  = frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        accel_data = accel_frame.get_motion_data();
        gyro_data  = gyro_frame.get_motion_data();

        vector<double> accel{accel_data.x, accel_data.y, accel_data.z};
        vector<double> gyro{gyro_data.x, gyro_data.y, gyro_data.z};

        p.Update(accel, gyro);
    }
        

    d.stop();
}
