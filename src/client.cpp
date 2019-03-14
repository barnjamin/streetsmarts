#include <librealsense2/rs.hpp> 
#include <thread>

#include <Core/Core.h>
#include <IO/IO.h>
#include "config.h"
#include "handlers.hpp"

int main(int argc, char * argv[]) try
{
    Config conf(argc, argv);

    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);
    
    rs2::frame_queue imu_q;
    rs2::frame_queue img_q;

    rs2::pipeline_profile profile = pipe.start(cfg, [&](rs2::frame frame){
        if (frame.is<rs2::frameset>()) img_q.enqueue(frame);
        //else imu_q.enqueue(frame); 
    });

    //std::thread imu_thread(record_imu, conf, imu_q);
    std::thread img_thread(make_fragments, conf, profile, img_q);

    //imu_thread.join();
    img_thread.join();

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
