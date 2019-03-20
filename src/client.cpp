#include <librealsense2/rs.hpp> 
#include <thread>

#include <Core/Core.h>
#include <IO/IO.h>
#include "gps.hpp"
#include "pose.h"
#include "config.h"
#include "handlers.hpp"

int main(int argc, char * argv[]) try
{
    Config conf(argc, argv);

    Pose pose(conf.fps);

    GPS gps;
    std::thread gps_thread;
    if(conf.capture_gps){
        open3d::PrintInfo("Initializing GPS\n");
        if(!gps.Connect()){
            std::cout << "Failed to connect" << std::endl;
            return 1;
        }
        open3d::PrintInfo("GPS Inititalized, starting stream\n");

        gps.Start();

        gps_thread = std::thread([&](){
            std::ofstream gps_file;
            gps_file.open(conf.GPSFile());
            while(gps.Sleep()) gps_file << gps.GetPosition();
            gps_file.close();
        });
    }

    open3d::PrintInfo("Initializing Image pipeline\n");

    rs2::pipeline pipe;
    rs2::frame_queue imu_q;
    rs2::frame_queue img_q;

    std::thread imu_thread;
    std::thread img_thread;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);

    if(conf.capture_imu){
        cfg.enable_stream(RS2_STREAM_ACCEL);
        cfg.enable_stream(RS2_STREAM_GYRO);
    }

    open3d::PrintInfo("Starting Image pipeline\n");
    rs2::pipeline_profile profile = pipe.start(cfg, [&](rs2::frame frame){
        if (frame.is<rs2::frameset>()) img_q.enqueue(frame);
        else imu_q.enqueue(frame); 
    });


    if(conf.capture_imu){
        imu_thread = std::thread(record_imu, conf, pose, imu_q);
    }

    if(conf.make_fragments){
        img_thread = std::thread(make_fragments, conf, profile, img_q);
    }else{
        img_thread = std::thread(record_img, conf, profile, img_q);
    }

    open3d::PrintInfo("Kicked off threads, waiting for finish\n");

    img_thread.join();
    pipe.stop();

    if(conf.capture_imu) imu_thread.join();

    if(conf.capture_gps) {
        gps.Stop();
        gps_thread.join();
    }

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
