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

    float voxel_length = 0.03f;
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

    
    FPSTimer timer("Process RGBD stream", conf.frames);

    //Visualizer visualizer;
    //if (!visualizer.CreateVisualizerWindow("Sequential IC RGBD Odometry", 1920, 1080, 0, 0)) {
    //    PrintWarning("Failed creating OpenGL window.\n");
    //    return -1;
    //}
    //visualizer.BuildUtilities();
    //visualizer.UpdateWindowTitle();

    std::shared_ptr<TriangleMeshCuda> mesh = std::make_shared<TriangleMeshCuda>();
    //visualizer.AddGeometry(mesh);

    //ViewParameters vp;
    //vp.boundingbox_max_ = Eigen::Vector3d(10,10,10); 
    //vp.boundingbox_min_ = Eigen::Vector3d(0,0,0); 
    //vp.field_of_view_ = 60; 
    //vp.front_ = Eigen::Vector3d(0,0,-1); 
    //vp.lookat_ = Eigen::Vector3d(1,0,0); 
    //vp.up_ = Eigen::Vector3d(0,-1,0); 
    //vp.zoom_ = 0.5;

    //visualizer.GetViewControl().ConvertFromViewParameters(vp);


    int save_index = 0;

    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2_vector accel_data, gyro_data;
    
    
    Pose pose(conf.fps);

    bool success;
    Eigen::Matrix4d delta;
    std::vector<std::vector<float>> losses;

    double last_ts;

    PrintInfo("Discarding first %d frames\n", conf.framestart);
    for(int i=0; i<conf.framestart; i++) rs2::frameset frameset = pipe.wait_for_frames(); 

    //Display d(argc, argv, &pose);
    //d.start();

    PrintInfo("Starting to read frames, reading %d frames\n", conf.frames);
    //for(int i = 0; i<1e8; i++){
    for(int i = 0; i<conf.frames; i++){
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


        //Upload images to GPU
        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        if(i==0) {
            rgbd_prev.CopyFrom(rgbd_curr);
            continue;
        }

        // Get IMU Values
        auto accel_frame = frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        auto gyro_frame  = frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        accel_data = accel_frame.get_motion_data();
        gyro_data  = gyro_frame.get_motion_data();

        vector<double> accel{accel_data.x, accel_data.y, accel_data.z - 2.0};
        vector<double> gyro{gyro_data.x, gyro_data.y, gyro_data.z};

        // Update Pose Estimate
        pose.Update(accel, gyro, gyro_frame.get_timestamp()/1000);

        //Seed transform with current estimate
        if(conf.use_imu){
            odometry.transform_source_to_target_ =  pose.GetTransform();
        }else{
            odometry.transform_source_to_target_ =  Eigen::Matrix4d::Identity();
        }

        //Initialize odometry with current and previous images
        odometry.Initialize(rgbd_curr, rgbd_prev);

        //Compute Odometry
        std::tie(success, delta, losses) = odometry.ComputeMultiScale();

        //double qd, td, vd;
        //std::tie(qd, td, vd) = pose.Difference(odometry.transform_source_to_target_);
        //PrintInfo("Qd: %.6f Td: %.6f Vd: %.6f\n", qd, td, vd);

        if(!success){
            break;
            rgbd_prev.CopyFrom(rgbd_curr);
            continue;
        }

        //Update Target to world
        target_to_world = target_to_world * odometry.transform_source_to_target_;

        //Integrate
        extrinsics.FromEigen(target_to_world);
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);

        //Improve Pose Estimation using odometry values
        if(conf.use_imu){
            pose.Improve(target_to_world);
        }


        if(i % (conf.fps*3) == 0){
            tsdf_volume.GetAllSubvolumes();

            mesher.MarchingCubes(tsdf_volume);
            *mesh = mesher.mesh();

            //visualizer.PollEvents();
            //visualizer.UpdateGeometry(); 

            //Convert mesh to point cloud
            //ICP point cloud using last save extrinciscs 
            //Add to world point cloud with 

            WriteTriangleMeshToPLY( "fragment-" 
                  + std::to_string(save_index) + ".ply", *(mesh->Download()));

            tsdf_volume.Reset();
            save_index++;
        }

        PinholeCameraParameters params;
        params.intrinsic_ =  intrinsics;
        params.extrinsic_ = odometry.transform_source_to_target_.inverse();
        trajectory.parameters_.emplace_back(params);

        rgbd_prev.CopyFrom(rgbd_curr);

        timer.Signal();
    }

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);

    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());

    WritePoseGraph("pose_graph.json", pose.GetGraph());
    WritePinholeCameraTrajectory("trajectory.json", trajectory);

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
