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

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

    int save_index = 0;

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

    PinholeCameraTrajectory trajectory;

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();

    RGBDOdometryCuda<3> odometry;
    PinholeCameraIntrinsic intrinsics;
    PinholeCameraIntrinsicCuda cuda_intrinsics;

    //Imu fps
    Pose pose(200);

    bool success;
    Eigen::Matrix4d delta;
    std::vector<std::vector<float>> losses;

    Display d(argc, argv, &pose);
    d.start();

    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2_vector accel_data, gyro_data;
    std::vector<double> accel, gyro;

    int i;
    auto cb = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (rs2::frameset fs = frame.as<rs2::frameset>()) {

            //Get processed aligned frame
            fs = align.process(fs);

            color_frame = fs.first(RS2_STREAM_COLOR);
            depth_frame = fs.get_depth_frame();	       

            if(conf.use_filter){
                depth_frame = conf.filter(depth_frame);
            }

            if (!depth_frame || !color_frame) { return; }

            memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
            memcpy(color_image_ptr->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);


            //Upload images to GPU
            rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

            if(i==0) {
                i++;
                rgbd_prev.CopyFrom(rgbd_curr);
                return;
            }

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

            if(!success){
                i++;
                return;
                rgbd_prev.CopyFrom(rgbd_curr);
                return;
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

            if(i % (conf.fps*2) == 0){
                tsdf_volume.GetAllSubvolumes();

                mesher.MarchingCubes(tsdf_volume);
                WriteTriangleMeshToPLY( "fragment-" + std::to_string(save_index) + ".ply", *(mesher.mesh().Download()));

                tsdf_volume.Reset();
                save_index++;
            }

            PinholeCameraParameters params;
            params.intrinsic_ =  intrinsics;
            params.extrinsic_ = odometry.transform_source_to_target_.inverse();
            trajectory.parameters_.emplace_back(params);

            rgbd_prev.CopyFrom(rgbd_curr);
            i++;

        } else { //Gyro/Accel
            if(frame.get_profile().stream_type() == RS2_STREAM_ACCEL){
                accel_data = frame.as<rs2::motion_frame>().get_motion_data();
                accel = {accel_data.x, accel_data.y, accel_data.z-2.0};
            }else{
                gyro_data = frame.as<rs2::motion_frame>().get_motion_data();
                gyro = {gyro_data.x, gyro_data.y, gyro_data.z};
                pose.Update(accel, gyro, frame.as<rs2::motion_frame>().get_timestamp()/1000);
            }
        }
    };



    PrintInfo("Initializing camera...\n");

    rs2::pipeline_profile profile = pipe.start(cfg, cb);

    intrinsics = get_intrinsics(profile);
    cuda_intrinsics = PinholeCameraIntrinsicCuda(intrinsics);

    odometry.SetIntrinsics(intrinsics);
    odometry.SetParameters(OdometryOption());
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();


    while(true){ } //TODO }


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
