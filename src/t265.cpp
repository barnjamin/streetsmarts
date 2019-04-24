// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include "utils.h"

using namespace cv;

int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    // Define frame callback

    // The callback is executed on a sensor thread and can be called simultaneously from multiple sensors
    // Therefore any modification to common memory should be done under lock
    std::mutex data_mutex;
    uint64_t pose_counter = 0;
    uint64_t frame_counter = 0;
    bool first_data = true;
    auto last_print = std::chrono::system_clock::now();


    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        // Only start measuring time elapsed once we have received the
        // first piece of data
        if (first_data) {
            first_data = false;
            last_print = std::chrono::system_clock::now();
        }

        if (auto fp = frame.as<rs2::pose_frame>()) {
            pose_counter++;
        } else if (auto fs = frame.as<rs2::frameset>()) {

            bool left = true;
            for(auto ptr = fs.begin(); ptr!=fs.end(); ++ptr){

                Mat f = frame_to_mat(*ptr);
                std::stringstream ssl;

                std::string shit = left ? "left/":"right/";

                ssl << "/home/ben/test265/" + shit;
                ssl << std::setw(6) << std::setfill('0') <<  frame_counter << ".jpg";
                imwrite(ssl.str(), f);
                left = false;
            
            }

            frame_counter++;
        }

        // Print the approximate pose and image rates once per second
        auto now = std::chrono::system_clock::now();
        if (now - last_print >= std::chrono::seconds(1)) {
            std::cout << "\r" << std::setprecision(0) << std::fixed
                      << "Pose rate: "  << pose_counter << " "
                      << "Image rate: " << frame_counter << std::flush;
            pose_counter = 0;
            //frame_counter = 0;
            last_print = now;
        }
    };

    // Start streaming through the callback
    rs2::pipeline_profile profiles = pipe.start(cfg, callback);

    // Get fisheye sensor intrinsics parameters
    rs2::stream_profile fisheye_stream = profiles.get_stream(RS2_STREAM_FISHEYE, 1);
    rs2_intrinsics intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();


    std::cout << "Width: " << intrinsics.width << " Height: " << intrinsics.height << std::endl; 
    std::cout << "PPX: " << intrinsics.ppx << " PPY: " << intrinsics.ppy << std::endl;
    std::cout << "FX: " << intrinsics.fx << " FY: " << intrinsics.fy << std::endl;
    std::cout << "Distortion: " <<  intrinsics.model << std::endl;
    std::cout << "Coeffs: ";
    for(int i=0; i<5; i++ ){
        std::cout << intrinsics.coeffs[i] << " ";
    }
    std::cout << std::endl;

    rs2_extrinsics extrinsics = fisheye_stream.get_extrinsics_to(profiles.get_stream(RS2_STREAM_FISHEYE, 2));
    std::cout << "Rotation: ";
    for(int i=0; i<9; i++ ){
        std::cout << extrinsics.rotation[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "Translation: ";
    for(int i=0; i<3; i++ ){
        std::cout << extrinsics.translation[i] << " ";
    }
    std::cout << std::endl;






    // Sleep this thread until we are done
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

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
