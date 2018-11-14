#include <string>
#include <vector>
#include <Core/Core.h>
#include <IO/IO.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>
#include <Visualization/Visualization.h>
#include <Core/Utility/Timer.h>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include "utils.h" 

using namespace open3d;
using namespace cv;


int main(int argc, char * argv[]) try
{
    Config conf;
    conf.parseArgs(argc, argv);

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);

    PinholeCameraIntrinsic intrinsic = get_intrinsics(profile);
    PinholeCameraIntrinsicCuda cudaintrin(intrinsic);

    rs2::align align(RS2_STREAM_COLOR);

    auto depth_image_ptr = std::make_shared<Image>();
    auto color_image_ptr = std::make_shared<Image>();
    depth_image_ptr->PrepareImage(640, 480, 1, 2);
    color_image_ptr->PrepareImage(640, 480, 3, 1);


    RGBDImageCuda rgbd(0.1f, 4.0f, 1000.0f);

    for(int i=0; i<100; i++)
    {

        if (i<10){
            continue;
        }
        rs2::frameset frameset = pipe.wait_for_frames();

        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!depth_frame || !color_frame) { continue; }

        memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), 640 * 480 * 2);
        memcpy(color_image_ptr->data_.data(), color_frame.get_data(), 640 * 480 * 3);

        rgbd.Upload(*depth_image_ptr, *color_image_ptr);

        PointCloudCuda pcl(VertexWithColor, 300000);
        pcl.Build(rgbd, cudaintrin);

        WritePointCloudToPLY("test_pcl.ply", *pcl.Download(), true);
        break;

    }

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
