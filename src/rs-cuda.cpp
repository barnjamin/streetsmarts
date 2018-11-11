#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include "utils.h" 
#include <Core/Utility/Timer.h>


int main(int argc, char * argv[]) try
{
    Config conf;
    conf.parseArgs(argc, argv);

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);

    Intrinsic intr = get_intrinsics(profile);

    rs2::align align(RS2_STREAM_COLOR);

    open3d::Timer t;
    for(int i=0; i<100; i++)
    {

        t.Start(); 
        rs2::frameset frameset = pipe.wait_for_frames();
        t.Stop();
        t.Print("Got Frame");

        t.Start();
        //Get processed aligned frame
        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(RS2_STREAM_COLOR);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!aligned_depth_frame || !other_frame)
        {
            continue;
        }

        auto depth = conf.filter(aligned_depth_frame);

        t.Stop();
        t.Print("Processed");

    }

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
