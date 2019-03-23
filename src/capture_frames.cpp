#include <string>
#include <vector>

#include <Open3D/Open3d.h>

#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>


#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
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
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2_vector accel_data, gyro_data;

    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    rs2::pipeline_profile profile = pipe.start(cfg);

    PrintInfo("Saving intrinsic\n");
    camera::PinholeCameraIntrinsic intrinsic = get_intrinsics(profile);
    io::WriteIJsonConvertible(conf.IntrinsicFile(), intrinsic);


    auto depth_image = std::make_shared<geometry::Image>();
    auto color_image = std::make_shared<geometry::Image>();
    depth_image->PrepareImage(conf.width, conf.height, 1, 2);
    color_image->PrepareImage(conf.width, conf.height, 3, 1);

    FPSTimer timer("Process RGBD stream", conf.fragments * conf.frames_per_fragment);

    PrintInfo("Discarding first %d frames\n", conf.framestart);
    for(int i=0; i<conf.framestart; i++) rs2::frameset frameset = pipe.wait_for_frames(); 

    PrintInfo("Starting to read frames...");
    for(int i = 0; i < conf.fragments * conf.frames_per_fragment; ++i)
    {
        frameset = pipe.wait_for_frames();

        //Get processed aligned frame
        frameset = align.process(frameset);

        color_frame = frameset.first(RS2_STREAM_COLOR);
        depth_frame = frameset.get_depth_frame();	       

        if (!depth_frame || !color_frame) { continue; }

        if(conf.use_filter){ depth_frame = conf.Filter(depth_frame); }

        memcpy(depth_image->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        WriteImage(conf.DepthFile(i), *depth_image);
        WriteImage(conf.ColorFile(i), *color_image);

        timer.Signal();
    }

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
