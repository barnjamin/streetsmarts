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
#include "config.h"

#include <iostream>
#include <csignal>

using namespace open3d;
using namespace open3d::cuda;
using namespace cv;
using namespace std;

//void signalHandler( int signum ) {
//   cout << "Interrupt signal (" << signum << ") received.\n";
//   exit(signum);
//}

int main(int argc, char * argv[]) try
{
    Config conf(argc, argv);

    //signal(SIGINT, signalHandler);

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

    PinholeCameraIntrinsic intrinsic = get_intrinsics(profile);

    WriteIJsonConvertible(conf.IntrinsicFile(), intrinsic);

    PinholeCameraIntrinsicCuda cuda_intrinsic(intrinsic);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);
    odometry.SetParameters(OdometryOption());

    auto depth_image = std::make_shared<Image>();
    auto color_image = std::make_shared<Image>();

    depth_image->PrepareImage(conf.width, conf.height, 1, 2);
    color_image->PrepareImage(conf.width, conf.height, 3, 1);

    RGBDImageCuda rgbd_prev(conf.width, conf.height, conf.max_depth);
    RGBDImageCuda rgbd_curr(conf.width, conf.height, conf.max_depth);

    float voxel_length = conf.tsdf_cubic_size / 512.0;
    TransformCuda trans = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, conf.tsdf_truncation, trans);


    FPSTimer timer("Process RGBD stream", 1000000);



    PrintInfo("Discarding first %d frames\n", conf.framestart);
    for(int i=0; i<conf.framestart; i++) rs2::frameset frameset = pipe.wait_for_frames(); 


    bool success;
    Eigen::Matrix4d t;
    std::vector<std::vector<float>> losses;

    int fragment_idx;
    PrintInfo("Starting to read frames...");

    while(true) 
    {

        Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
        PoseGraph pose_graph;
        pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry));

        for(int i = 0; i<conf.frames_per_fragment; i++)
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

            WriteImage(conf.DepthFile(fragment_idx, i), *depth_image);
            WriteImage(conf.ColorFile(fragment_idx, i), *color_image);

            //Upload images to GPU
            rgbd_curr.Upload(*depth_image, *color_image);

            if(i==0) { rgbd_prev.CopyFrom(rgbd_curr); continue; }


            //Reset Odometry transform
            odometry.transform_source_to_target_ =  Eigen::Matrix4d::Identity();

            //Initialize odometry with current and previous images
            odometry.Initialize(rgbd_curr, rgbd_prev);

            //Compute Odometry
            std::tie(success, t, losses) =  odometry.ComputeMultiScale();
            if(!success) {
                rgbd_prev.CopyFrom(rgbd_curr);
                continue; 
            }

            auto information = odometry.ComputeInformationMatrix();

            //Update Target to world
            trans_odometry = trans_odometry * odometry.transform_source_to_target_;
            trans.FromEigen(trans_odometry);

            //Integrate fragment
            tsdf_volume.Integrate(rgbd_curr, cuda_intrinsic, trans);

            //Update pose graph
            Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();
            pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            pose_graph.edges_.emplace_back(PoseGraphEdge(i-1, i, 
                        odometry.transform_source_to_target_, information, false));

            
            rgbd_prev.CopyFrom(rgbd_curr);
            timer.Signal();
        }

        WritePoseGraph(conf.PoseFile(fragment_idx), pose_graph);

        //Generate Mesh and write to disk
        tsdf_volume.GetAllSubvolumes();
        ScalableMeshVolumeCuda<8> mesher(tsdf_volume.active_subvolume_entry_array().size(), VertexWithNormalAndColor, 10000000, 20000000);
        mesher.MarchingCubes(tsdf_volume);
        auto mesh = mesher.mesh().Download();


        PointCloud pcl;
        pcl.points_ = mesh->vertices_;
        pcl.normals_ = mesh->vertex_normals_;
        pcl.colors_ = mesh->vertex_colors_;

        WritePointCloudToPLY(conf.FragmentFile(fragment_idx), pcl, true);

        tsdf_volume.Reset();

        fragment_idx++; 
    }

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
