#include <string>
#include <vector>
#include <Core/Core.h>
#include <IO/IO.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>
#include <Visualization/Visualization.h>
#include <Core/Utility/Timer.h>

#include <Visualization/Visualization.h>
#include <Geometry/ImageCuda.h>
#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Cuda/Common/TransformCuda.h>
#include <Geometry/TriangleMeshCuda.h>
#include <Geometry/RGBDImageCuda.h>
#include <Integration/ScalableTSDFVolumeCuda.h>
#include <Integration/ScalableMeshVolumeCuda.h>
#include <Cuda/Geometry/PointCloudCuda.h>

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

    auto depth_img = std::make_shared<Image>();
    auto color_img = std::make_shared<Image>();
    depth_img->PrepareImage(640, 480, 1, 2);
    color_img->PrepareImage(640, 480, 3, 1);


    RGBDImageCuda rgbd(0.1f, 4.0f, 1000.0f);


    Eigen::Matrix4d flipper;
    flipper << -1,0,0,0,
                0,-1,0,0,
                0,0,-1,0,
                0,0,0,1;


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

        memcpy(depth_img->data_.data(), depth_frame.get_data(), 640 * 480 * 2);
        memcpy(color_img->data_.data(), color_frame.get_data(), 640 * 480 * 3);

        auto rgbdimg = CreateRGBDImageFromColorAndDepth(*color_img, *depth_img);

        auto pcl = CreatePointCloudFromRGBDImage(*rgbdimg, intrinsic);

        pcl->Transform(flipper);

        auto pclc = PointCloudCuda(VertexWithColor, 3000000);

        pclc.Upload(*pcl);

        DrawGeometries({pclc.Download()});

        //rgbd->Upload(*depth_image_ptr, *color_image_ptr);
       
        //std::shared_ptr<PointCloudCuda> pcl
        //        = std::make_shared<PointCloudCuda>(VertexWithColor, 300000);
        //pcl->Build(rgbd, cudaintrin);

         //VisualizerWithCustomAnimation visualizer;
         //if (! visualizer.CreateVisualizerWindow("Visualizer", 640, 480, 0, 0)) {
         //    PrintWarning("Failed creating OpenGL window.\n");
         //    return 0;
         //}
         //visualizer.AddGeometry(rgbd);

         //visualizer.GetRenderOption().show_coordinate_frame_ = true;
         //visualizer.BuildUtilities();
         //visualizer.UpdateWindowTitle();

         //bool should_close = false;
         //while (! should_close) {
         //    should_close = !visualizer.PollEvents();
         //}
         //visualizer.DestroyVisualizerWindow();
    }

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
