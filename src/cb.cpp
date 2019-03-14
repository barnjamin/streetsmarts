// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <thread>

#include "callbacks.hpp"
#include "config.h"

// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) try
{
    Config conf(argc, argv);

    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    rs2::pipeline pipe;

    std::mutex mtx;
    RecordContext * ctx = new RecordContext(&mtx, conf);

    rs2::pipeline_profile profiles = pipe.start(CallBack(*ctx));

    std::cout << "RealSense callback sample" << std::endl << std::endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ctx->PrintState();
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
