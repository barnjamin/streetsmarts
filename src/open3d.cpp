#include <string>
#include <vector>
#include <Core/Core.h>
#include <IO/IO.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>
#include <Visualization/Visualization.h>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include "utils.h" 

using namespace open3d;
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

    Intrinsic i = get_intrinsics(profile);
    PinholeCameraIntrinsic intrinsics(i.width, i.height, i.intrinsics.fx, i.intrinsics.fy, i.intrinsics.ppx, i.intrinsics.ppy);

    PinholeCameraIntrinsicCuda cuda_intrinsics(intrinsics);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsics);
    odometry.SetParameters(0.0f, 0.1f, 4.0f, 0.07f);
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

    float voxel_length = 0.01f;
    TransformCuda extrinsics = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, 3 * voxel_length, extrinsics);

    auto depth_image_ptr = std::make_shared<Image>();
    depth_image_ptr->PrepareImage(640, 480, 1, 2);

    auto color_image_ptr = std::make_shared<Image>();
    color_image_ptr->PrepareImage(640, 480, 3, 1);

    RGBDImageCuda rgbd_prev(0.1f, 4.0f, 1000.0f);
    RGBDImageCuda rgbd_curr(0.1f, 4.0f, 1000.0f);

    ScalableMeshVolumeCuda<8> mesher(40000, VertexWithNormalAndColor, 6000000, 12000000);

    std::shared_ptr<TriangleMeshCuda> mesh = std::make_shared<TriangleMeshCuda>();

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();

    PinholeCameraParameters params;
    params.intrinsic_ = PinholeCameraIntrinsicParameters::PrimeSenseDefault;


    FPSTimer timer("Process RGBD stream",conf.frames);

    int save_index = 0;

    PrintInfo("Starting to poll for frames\n");
    for(int i=0; i< conf.frames; i++){
        rs2::frameset frameset = pipe.wait_for_frames();

        //Get processed aligned frame
        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!depth_frame || !color_frame) {
            continue;
        }

        //Further process for hole filling/temporal smoothing
        //depth_frame = conf.filter(depth_frame);

        //auto color = frame_to_mat(color_frame);
        //cvtColor(color, color, COLOR_BGR2GRAY);
        //color.convertTo(color, CV_32FC1, 1.0f / 255.0f);

        //auto depth = frame_to_mat(depth_frame);
        //depth.convertTo(depth, CV_32FC1, 0.0001);
        //rgbd_curr.Upload(depth, color);

        memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), 640 * 480 * 2);
        memcpy(color_image_ptr->data_.data(), color_frame.get_data(), 640 * 480 * 3);

        //auto rgbd = CreateRGBDImageFromColorAndDepth(*color_image_ptr, *depth_image_ptr);
        //auto pcd = CreatePointCloudFromRGBDImage(*rgbd, intrinsics);
        //DrawGeometries({pcd});

        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        if (i > 0 ) {
            odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();
            odometry.PrepareData(rgbd_curr, rgbd_prev);
            odometry.Apply();
            target_to_world = target_to_world * odometry.transform_source_to_target_;
        }

        extrinsics.FromEigen(target_to_world);
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);

        params.extrinsic_ = extrinsics.ToEigen().inverse();

        if (i > 0 && i % 30 == 0) {
            tsdf_volume.GetAllSubvolumes();
            mesher.MarchingCubes(tsdf_volume);
            WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());
            tsdf_volume.Reset();
            save_index++;
        }

        rgbd_prev.CopyFrom(rgbd_curr);
        timer.Signal();
    }

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);
    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
