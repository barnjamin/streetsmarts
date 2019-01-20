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
#include "pose.h"
#include "display.h"
#include "config.h"

using namespace open3d;
using namespace open3d::cuda;
using namespace cv;
using namespace std;

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
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

    PrintInfo("Initializing camera...\n");

    rs2::pipeline_profile profile = pipe.start(cfg);

    PinholeCameraIntrinsic intrinsics = get_intrinsics(profile);
    PinholeCameraIntrinsicCuda cuda_intrinsics(intrinsics);

    PinholeCameraTrajectory trajectory;

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsics);
    odometry.SetParameters(OdometryOption());
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

    float voxel_length = 0.01f;
    TransformCuda extrinsics = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, 3 * voxel_length, extrinsics);

    auto depth_image_ptr = std::make_shared<Image>();
    auto color_image_ptr = std::make_shared<Image>();
    depth_image_ptr->PrepareImage(conf.width, conf.height, 1, 2);
    color_image_ptr->PrepareImage(conf.width, conf.height, 3, 1);

    RGBDImageCuda rgbd_prev(0.1f, 4.0f, 1000.0f);
    RGBDImageCuda rgbd_curr(0.1f, 4.0f, 1000.0f);

    ScalableMeshVolumeCuda<8> mesher(100000, VertexWithNormalAndColor, 10000000, 20000000);

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();

    PrintInfo("Discarding first %d frames\n");

    for(int i=0; i<conf.framestart; i++){
        rs2::frameset frameset = pipe.wait_for_frames();
    }
    
    FPSTimer timer("Process RGBD stream", conf.frames);

    int save_index = 0;

    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2_vector accel_data, gyro_data;
    
    Pose pose(conf.fps);

    bool success;
    Eigen::Matrix4d delta;
    std::vector<std::vector<float>> losses;

    double last_ts;

    PrintInfo("Starting to read frames, reading %d frames\n", conf.frames);
    for(int i=0; i< conf.frames; i++){
        frameset = pipe.wait_for_frames();

        //Get processed aligned frame
        frameset = align.process(frameset);

        color_frame = frameset.first(RS2_STREAM_COLOR);
        depth_frame = frameset.get_depth_frame();	       

        if(conf.use_filter){
            depth_frame = conf.filter(depth_frame);
        }

        if (!depth_frame || !color_frame) { continue; }

        memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image_ptr->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        // Get IMU Values
        auto accel_frame = frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        auto gyro_frame  = frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        accel_data = accel_frame.get_motion_data();
        gyro_data  = gyro_frame.get_motion_data();

        vector<double> accel{accel_data.x, accel_data.y, accel_data.z};
        vector<double> gyro{gyro_data.x, gyro_data.y, gyro_data.z};

        // Update Pose Estimate
        pose.Update(accel, gyro, gyro_frame.get_timestamp()/1000);
        
        //Upload images to GPU
        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        if (i > 0 ) {
            //Seed transform with current estimate
            odometry.transform_source_to_target_ =  pose.GetTransform();

            //Initialize odometry with current and previous images
            odometry.Initialize(rgbd_curr, rgbd_prev);

            //Compute Odometry
            std::tie(success, delta, losses) = odometry.ComputeMultiScale();

            if(success){
                //Update Target to world
                target_to_world = target_to_world * odometry.transform_source_to_target_;

                //Improve Pose Estimation using odometry values
                pose.Improve(odometry.transform_source_to_target_, target_to_world);

                extrinsics.FromEigen(target_to_world);
                tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);
            } 

            //if (!success || (i > 0 && i % conf.fps*2 == 0)) {
            //    tsdf_volume.GetAllSubvolumes();

            //    mesher.MarchingCubes(tsdf_volume);

            //    WriteTriangleMeshToPLY( "fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());
            //    
            //    tsdf_volume.Reset();

            //    //pose.Reset();

            //    save_index++;
            //}
        }
        
        rgbd_prev.CopyFrom(rgbd_curr);

        timer.Signal();
    }

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);

    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());

    WritePoseGraph("pose_graph.json", pose.GetGraph());

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
