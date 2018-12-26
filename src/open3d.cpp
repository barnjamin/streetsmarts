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

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);

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
    depth_image_ptr->PrepareImage(conf.width, conf.height, 1, 2);
    color_image_ptr->PrepareImage(conf.width, conf.height, 3, 1);

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
    rs2::frame color_frame, depth_frame;
    rs2::motion_frame accel_frame, gyro_frame;
    rs2_vector accel, gyro;

    PrintInfo("Starting to read frames, reading %d frames\n", conf.frames);
    for(int i=0; i< conf.frames; i++){
        frameset = pipe.wait_for_frames();

        //Get processed aligned frame
        //frameset = align.process(frameset);

        // Trying to get both other and aligned depth frames
        color_frame = frameset.first(RS2_STREAM_COLOR);
        depth_frame = frameset.get_depth_frame();

        accel_frame = frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        gyro_frame  = frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        accel_data = aframe.get_motion_data();
        gyro_data  = gframe.get_motion_data();

        if (!depth_frame || !color_frame) { continue; }

        //depth_frame = conf.filter(depth_frame);

	MadgwickAHRSupdateIMU(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);


        memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image_ptr->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        if (i > 0 ) {
            q = Eigen::Quaternionf(q0, q1, q2, q3);

            odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();
            odometry.Initialize(rgbd_curr, rgbd_prev);
            odometry.ComputeMultiScale();

            target_to_world = target_to_world * odometry.transform_source_to_target_;
        }

        extrinsics.FromEigen(target_to_world);
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);

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

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);

    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
