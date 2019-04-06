#include <string>
#include <vector>

#include <Open3D/Open3D.h>
#include <Cuda/Open3DCuda.h>


#include <array>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h>

#include "utils.h" 
#include "pose.h"
#include "config.h"

#include <iostream>
#include <csignal>

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::utility;
using namespace cv;
using namespace std;

int main(int argc, char * argv[]) try
{
    Config conf(argc, argv);

    std::cout << "Writing to " << conf.session_path<<std::endl;

    PrintInfo("Initializing camera...\n");
    rs2::pipeline pipeline;
    rs2::config cfg;
    rs2::colorizer color_map;

    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);

    rs2::pipeline_profile pipeline_profile = pipeline.start(cfg);

    for(int i=0; i<conf.framestart; i++) pipeline.wait_for_frames(); 

    // BaseLine
    rs2::frameset frameset = pipeline.wait_for_frames();
    rs2::depth_frame depth_frame = frameset.get_depth_frame();

    auto intrin = get_color_intrinsic(pipeline_profile);

    auto invalid_depth_band = conf.GetInvalidDepth(depth_frame, intrin);

    cv::Mat df = frame_to_mat(depth_frame);

    auto cmap = frame_to_mat(color_map.process(depth_frame));


    cv::Rect invalid(0, 0, invalid_depth_band*2, conf.height);
    cv::rectangle(cmap, invalid, cv::Scalar(0,0,0), CV_FILLED, 8, 0);

    cv::imshow("raw", df);
    cv::imshow("mapped", cmap);

    cv::waitKey(0);

    //PrintInfo("Starting to read frames...");
    //for(int i = 0; i < conf.fragments * conf.frames_per_fragment; ++i)
    //{
    //    frameset = pipe.wait_for_frames();

    //    //Get processed aligned frame
    //    frameset = align.process(frameset);

    //    color_frame = frameset.first(RS2_STREAM_COLOR);
    //    depth_frame = frameset.get_depth_frame();	       



    //}

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
