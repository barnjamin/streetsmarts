// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <mutex>
#include <thread>
#include "config.h"
#include "pose.h"
#include "display.h"

//RS2_OPTION_BACKLIGHT_COMPENSATION 	
//Enable / disable color backlight compensation

//RS2_OPTION_BRIGHTNESS 	
//Color image brightness

//RS2_OPTION_CONTRAST 	
//Color image contrast

//RS2_OPTION_EXPOSURE 	
//Controls exposure time of color camera. Setting any value will disable auto exposure

//RS2_OPTION_WHITE_BALANCE 	
//Controls white balance of color image. Setting any value will disable auto white balance

//RS2_OPTION_ENABLE_AUTO_EXPOSURE 	
//Enable / disable color image auto-exposure

//RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE 	
//Enable / disable color image auto-white-balance

//  0: Backlight Compensation
//       Current Value : 0
//  1: Brightness
//       Current Value : 0
//  2: Contrast
//       Current Value : 50
//  3: Exposure
//       Current Value : 166
//  9: White Balance
//       Current Value : 4600
//  10: Enable Auto Exposure
//       Current Value : 1
//  11: Enable Auto White Balance
//       Current Value : 1


void print_sensor_options(const rs2::sensor& sensor)
{
    // Sensors usually have several options to control their properties
    //  such as Exposure, Brightness etc.

    std::cout << "Sensor supports the following options:\n" << std::endl;

    // The following loop shows how to iterate over all available options
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        if (i > 11) {
            continue;
        }
        rs2_option option_type = static_cast<rs2_option>(i);

        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << "  " << i << ": " << option_type << std::endl;

            // Get a human readable description of the option
            //const char* description = sensor.get_option_description(option_type);
            //std::cout << "       Description   : " << description << std::endl;

            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;

            //To change the value of an option, please follow the change_sensor_option() function
        } 
    }

    return;
}


// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) try
{
    //rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    Config conf;
    conf.parseArgs(argc, argv);

    std::mutex mutex;

    rs2::config cfg;
    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);

    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::context ctx;
    std::string rcam("RGB Camera");
    // Using the context we can get all connected devices in a device list
    auto sensors = ctx.query_all_sensors();
    rs2::sensor cam;
    for(rs2::sensor sensor: sensors){
        if (sensor.supports(RS2_CAMERA_INFO_NAME)){
            std::string name = sensor.get_info(RS2_CAMERA_INFO_NAME);
            if(name.compare(rcam) == 0)
                cam = sensor;
        }
    }

    Pose pose(200);

    rs2_vector accel_data, gyro_data;
    std::vector<double> accel;
    std::vector<double> gyro;
    Display d(argc, argv, &pose);
    d.start();
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::frameset fs = frame.as<rs2::frameset>()) {
                //print_sensor_options(cam);
        } else { //Gyro/Accel
            if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL){
                accel_data = frame.as<rs2::motion_frame>().get_motion_data();
                accel = {accel_data.x, accel_data.y, accel_data.z};
            }else{
                gyro_data = frame.as<rs2::motion_frame>().get_motion_data();
                gyro = {gyro_data.x, gyro_data.y, gyro_data.z};
                pose.Update(accel, gyro, frame.as<rs2::motion_frame>().get_timestamp()/1000);
            }
        }
    };

    rs2::pipeline pipe;
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    std::cout << "RealSense callback sample" << std::endl << std::endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::lock_guard<std::mutex> lock(mutex);
    }
    //d.stop();

    return EXIT_SUCCESS;
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
