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
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

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
            //frameset = align.process(frameset);

            //color_frame = frameset.first(RS2_STREAM_COLOR);
            //depth_frame = frameset.get_depth_frame();	       

            //if(conf.use_filter){
            //    depth_frame = conf.filter(depth_frame);
            //}

            //if (!depth_frame || !color_frame) { continue; }




        } else { //Gyro/Accel
            if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL){
                std::cout << "Accel frame: " << frame.get_profile().unique_id() << std::endl;
                accel_data = frame.as<rs2::motion_frame>().get_motion_data();
                accel = {accel_data.x, accel_data.y, accel_data.z};
            }else{
                std::cout << "Gyro frame: " << frame.get_profile().unique_id() << std::endl;
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
        std::cout << "hi" << std::endl;
    }
    d.stop();

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
