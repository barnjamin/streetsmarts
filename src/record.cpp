#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>

#include <GL/glut.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <IO/IO.h>

#include "utils.h" 


std::string basedir = "/home/ben/streetsmarts/build/dumps";

bool create_dump_dirs(std::string dirname){
    auto colordir = dirname + "/color";
    auto depthdir = dirname + "/depth";

    std::cout << "Creating " << dirname << std::endl;
    if (mkdir(dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1){
        return false;
    }

    if (mkdir(colordir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1){
        return false;
    }

    if (mkdir(depthdir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1){
        return false;
    }

    std::string linkdir = basedir + "/latest";

    remove(linkdir.c_str());

    if(symlink(dirname.c_str(), linkdir.c_str()) == -1){
        return false;
    }

    return true;
}

std::string get_dump_dir(){
    auto t = std::time(nullptr); 
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d%H%M%S");
    return  basedir + "/" + oss.str();
}

int main(int argc, char * argv[]) try
{

    Config conf;
    conf.parseArgs(argc, argv);

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

    rs2::pipeline_profile profile = pipe.start(cfg);

    std::string dirname = get_dump_dir();
    if(!create_dump_dirs(dirname)) {
        std::cerr << "Failed to create directories " << std::endl;
        return -1;
    }

    std::cout << "Created dirs starting to read frames" << std::endl;

    std::ofstream dump;
    dump.open (dirname + "/imu.csv");

    open3d::PinholeCameraIntrinsic intrinsics = get_intrinsics(profile);
    open3d::WriteIJsonConvertible(dirname + "/intrinsic.json", intrinsics);

    //Discard waiting for auto exposure
    //for(int x = 0; x<conf.framestart; x++) pipe.wait_for_frames();
    for(int x = 0; x<90; x++) pipe.wait_for_frames();

    std::cout << "Reading frames..." << std::endl;
    for(int x=0; x<conf.frames; x++) {
        auto frameset = pipe.wait_for_frames();

        cv::Mat color = frame_to_mat(frameset.first(RS2_STREAM_COLOR));
        cv::Mat depth = frame_to_mat(frameset.get_depth_frame());

        cv::imwrite(dirname+"/color/"+std::to_string(x)+".jpg", color);
        cv::imwrite(dirname+"/depth/"+std::to_string(x)+".png", depth);

        auto aframe =  frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        auto gframe =  frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        auto accel = aframe.get_motion_data();
        auto gyro = gframe.get_motion_data();

        dump << std::to_string(x) << "," 
            << accel.x << "," << accel.y << "," << accel.z  << "," 
            << gyro.x << "," << gyro.y << "," << gyro.z  << "," << std::endl;
    }

    dump.close();
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


