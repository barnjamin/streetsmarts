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

    while(true){
        rs2::frameset frameset = pipeline.wait_for_frames();
        rs2::depth_frame depth_frame = frameset.get_depth_frame();

        depth_frame = conf.depth_to_disparity.process(depth_frame);

        auto mat = frame_to_mat(depth_frame);

        double min;
        double max;

        minMaxIdx(mat, &min, &max);

        mat.convertTo(mat,CV_8UC1, 255 / (max-min), -min);

        applyColorMap(mat, mat, COLORMAP_JET);

        // Apply the colormap:
        cv::imshow("mapped", mat);
        cv::waitKey(1);


    }

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
