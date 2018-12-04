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
using namespace open3d::cuda;
using namespace cv;

int main(int argc, char * argv[]) try
{

    Config conf;
    conf.parseArgs(argc, argv);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);

    PinholeCameraIntrinsic intrinsics = get_intrinsics(profile);
    PinholeCameraIntrinsicCuda cuda_intrinsics(intrinsics);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsics);
    odometry.SetParameters(OdometryOption());
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

    float voxel_length = 0.01f;
    TransformCuda extrinsics = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, 
            3 * voxel_length, extrinsics);

    auto depth_image_ptr = std::make_shared<Image>();
    auto color_image_ptr = std::make_shared<Image>();
    depth_image_ptr->PrepareImage(640, 480, 1, 2);
    color_image_ptr->PrepareImage(640, 480, 3, 1);

    RGBDImageCuda rgbd_prev(0.1f, 4.0f, 1000.0f);
    RGBDImageCuda rgbd_curr(0.1f, 4.0f, 1000.0f);

    ScalableMeshVolumeCuda<8> mesher(40000, VertexWithNormalAndColor, 6000000, 12000000);

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();
    for(int i=0; i<conf.framestart; i++){
        rs2::frameset frameset = pipe.wait_for_frames();
    }
    
    FPSTimer timer("Process RGBD stream", conf.frames);

    int save_index = 0;
    rs2::frameset frameset;
    rs2::frame color_frame;
    rs2::frame depth_frame;
    PrintInfo("Starting to read frames, reading %d frames\n", conf.frames);
    Timer t;
    for(int i=0; i< conf.frames; i++){
        t.Start();
        frameset = pipe.wait_for_frames();
        t.Stop();
        t.Print("Frame Wait");
        //Get processed aligned frame
        t.Start();
        frameset = align.process(frameset);
        t.Stop();
        t.Print("Frame Align");

        // Trying to get both other and aligned depth frames
        color_frame = frameset.first(RS2_STREAM_COLOR);
        depth_frame = frameset.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!depth_frame || !color_frame) { continue; }

        //depth_frame = conf.filter(depth_frame);
        t.Start();
        memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), 640 * 480 * 2);
        memcpy(color_image_ptr->data_.data(), color_frame.get_data(), 640 * 480 * 3);
        t.Stop();
        t.Print("Copy Frames To Images");

        t.Start();
        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);
        t.Stop();
        t.Print("Upload to RGBD");

        if (i > 0 ) {
            t.Start();
            odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();
            odometry.Initialize(rgbd_curr, rgbd_prev);
            t.Stop();
            t.Print("Prepare");
            t.Start();
            odometry.ComputeMultiScale();
            t.Stop();
            t.Print("Apply");

            target_to_world = target_to_world * odometry.transform_source_to_target_;
        }

        extrinsics.FromEigen(target_to_world);
        t.Start();
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);
        t.Stop();
        t.Print("Integrate");

        //if (i > 0 && i % 30 == 0) {
        //    tsdf_volume.GetAllSubvolumes();
        //    mesher.MarchingCubes(tsdf_volume);
        //    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", 
        //                *mesher.mesh().Download());
        //    //tsdf_volume.Reset();
        //    save_index++;
        //}

        rgbd_prev.CopyFrom(rgbd_curr);
        timer.Signal();
    }

    t.Start();
    tsdf_volume.GetAllSubvolumes();
    t.Stop();
    t.Print("Got subvolumes");
    t.Start();
    mesher.MarchingCubes(tsdf_volume);
    t.Stop();
    t.Print("Marching Cubes");

    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
