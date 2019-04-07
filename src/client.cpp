#include <librealsense2/rs.hpp> 
#include <thread>
#include <queue>
#include <atomic>
#include <Open3D/Open3D.h>
#include "gps.hpp"
#include "pose.h"
#include "config.h"
#include "handlers.hpp"
#include "refinement.hpp"
#include "integrate.hpp"

int main(int argc, char * argv[]) try
{

    //open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::VerboseAlways);

    Config conf;
    // Assume json
    if(argc==2){
        std::string config_path = argv[1];
        if(!open3d::io::ReadIJsonConvertible(config_path, conf)) {
            open3d::utility::PrintError("Failed to read config\n");
            return 1;
        }
    }else{
        conf = Config(argc, argv);
    }

    open3d::utility::PrintInfo("Starting..\n");

    conf.CreateLocalSession();

    Pose pose(conf.fps);

    GPS gps(conf.GPSFile());
    std::thread gps_thread;
    if(conf.capture_gps){
        open3d::utility::PrintInfo("Initializing GPS\n");
        if(!gps.Connect()){
            std::cout << "Failed to connect" << std::endl;
            return 1;
        }
        open3d::utility::PrintInfo("GPS Inititalized, starting stream\n");

        gps.Start();
    }

    open3d::utility::PrintInfo("Initializing Image pipeline\n");

    rs2::pipeline pipe;

    rs2::frame_queue imu_q;
    rs2::frame_queue img_q;

    std::thread imu_thread;


    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, conf.width, conf.height, RS2_FORMAT_Y8, conf.fps);

    if(conf.capture_imu){
        cfg.enable_stream(RS2_STREAM_ACCEL);
        cfg.enable_stream(RS2_STREAM_GYRO);
    }

    open3d::utility::PrintInfo("Starting Image pipeline\n");
    rs2::pipeline_profile profile = pipe.start(cfg, [&](rs2::frame frame){
        if (frame.is<rs2::frameset>()) img_q.enqueue(frame);
        else imu_q.enqueue(frame); 
    });

    std::atomic<bool> running(true);
    if(conf.capture_imu){
        imu_thread = std::thread(record_imu, conf, std::ref(running), imu_q);
    }


    std::mutex mtx;
    std::queue<int>  pg_queue;
    std::queue<int>  frag_queue;

    std::thread img_thread(make_posegraph, conf, profile, img_q, std::ref(pg_queue), std::ref(mtx));
    std::thread frag_thread(make_fragments, conf, std::ref(pg_queue), std::ref(frag_queue), std::ref(running), std::ref(mtx));
    std::thread refine_thread(refine_fragments_streaming, conf, std::ref(frag_queue), std::ref(running), std::ref(mtx));

    open3d::utility::PrintInfo("Kicked off threads, waiting for finish\n");

    img_thread.join();

    open3d::utility::PrintInfo("Finished capturing images\n");


    if(conf.capture_gps) {
        gps.Stop();
        gps_thread.join();
    }

    pipe.stop();

    running = false;
    open3d::utility::PrintInfo("Stopped pipeline\n");

    frag_thread.join();
    refine_thread.join();

    if(conf.capture_imu) imu_thread.join();

    open3d::utility::PrintInfo("Refining Fragments\n");
    RefineFragments(conf);

    open3d::utility::PrintInfo("Integrating Scene\n");
    IntegrateScene(conf);


    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
