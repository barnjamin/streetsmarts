// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <map>
#include <chrono>
#include <thread>

#include "callbacks.hpp"

// The callback example demonstrates asynchronous usage of the pipeline
int main(int argc, char * argv[]) try
{

    // Declare RealSense pipeline, encapsulating the actual device and sensors.
    rs2::pipeline pipe;

    SaveToDisk cb;

    rs2::pipeline_profile profiles = pipe.start(cb);

    // Collect the enabled streams names
    for (auto p : profiles.get_streams())
        cb.AddStream(p.unique_id(), p.stream_name());

    std::cout << "RealSense callback sample" << std::endl << std::endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cb.PrintState();
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
