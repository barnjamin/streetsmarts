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

    PrintInfo("Initializing camera...\n");
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2_vector accel_data, gyro_data;

    //rs2::align align(RS2_STREAM_DEPTH);
    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    rs2::pipeline_profile profile = pipe.start(cfg);

    PinholeCameraIntrinsic intrinsics = get_intrinsics(profile);
    PinholeCameraIntrinsicCuda cuda_intrinsic(intrinsics);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsics);
    odometry.SetParameters(OdometryOption());
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();


    auto depth_image_ptr = std::make_shared<Image>();
    auto color_image_ptr = std::make_shared<Image>();

    depth_image_ptr->PrepareImage(conf.width, conf.height, 1, 2);
    color_image_ptr->PrepareImage(conf.width, conf.height, 3, 1);

    RGBDImageCuda rgbd_prev((float) conf.max_depth, (float) conf.depth_factor);
    RGBDImageCuda rgbd_curr((float) conf.max_depth, (float) conf.depth_factor);

    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();

    PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry));

    PrintInfo("Discarding first %d frames\n", conf.framestart);
    for(int i=0; i<conf.framestart; i++) rs2::frameset frameset = pipe.wait_for_frames(); 


    int fragment_idx;
    PrintInfo("Starting to read frames...");

    while(true) 
    {

        for(int i = 0; i<conf.fps; i++)
        {
            frameset = pipe.wait_for_frames();

            //Get processed aligned frame
            frameset = align.process(frameset);

            color_frame = frameset.first(RS2_STREAM_COLOR);
            depth_frame = frameset.get_depth_frame();	       

            if (!depth_frame || !color_frame) { continue; }

            if(conf.use_filter){ depth_frame = conf.filter(depth_frame); }

            memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
            memcpy(color_image_ptr->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

            //TODO: Write images to disk
            

            //Upload images to GPU
            rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

            if(i==0) { rgbd_prev.CopyFrom(rgbd_curr); continue; }


            //Reset Odometry transform
            odometry.transform_source_to_target_ =  Eigen::Matrix4d::Identity();

            //Initialize odometry with current and previous images
            odometry.Initialize(rgbd_curr, rgbd_prev);

            //Compute Odometry
            odometry.ComputeMultiScale();
            auto information = odometry.ComputeInformationMatrix();

            //Update Target to world
            Eigen::Matrix4d trans = odometry.transform_source_to_target_;

            trans_odometry = trans_odometry * trans;

            Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();

            pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            pose_graph.edges_.emplace_back(PoseGraphEdge(i-1, i, trans, information, false));

            rgbd_prev.CopyFrom(rgbd_curr);
        }


        //GlobalOptimizationConvergenceCriteria criteria;
        //GlobalOptimizationOption option( conf.max_depth_diff, 0.25, conf.preference_loop_closure_odometry_, 0);
        //GlobalOptimizationLevenbergMarquardt optimization_method;
        //GlobalOptimization(pose_graph, optimization_method, criteria, option);

        //auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges( pose_graph, option);
        //auto pose_graph_prunned = pose_graph;

        WritePoseGraph(conf.PoseFile(fragment_idx), pose_graph);

        float voxel_length = conf.tsdf_cubic_size / 512.0;

        TransformCuda trans = TransformCuda::Identity();
        ScalableTSDFVolumeCuda<8> tsdf_volume( 20000, 400000, voxel_length, (float) conf.tsdf_truncation, trans);

        RGBDImageCuda rgbd((float) conf.max_depth, (float) conf.depth_factor);

        for (int i = 0; i < conf.frames_per_fragment; ++i) {
            PrintDebug("Integrating frame %d ...\n", i);

            Image depth, color;
            ReadImage(conf.DepthFile(i), depth);
            ReadImage(conf.ColorFile(i), color);
            rgbd.Upload(depth, color);

            Eigen::Matrix4d pose = pose_graph.nodes_[i].pose_;
            trans.FromEigen(pose);

            tsdf_volume.Integrate(rgbd, cuda_intrinsic, trans);
        }

        tsdf_volume.GetAllSubvolumes();
        ScalableMeshVolumeCuda<8> mesher(tsdf_volume.active_subvolume_entry_array().size(), VertexWithNormalAndColor, 10000000, 20000000);
        mesher.MarchingCubes(tsdf_volume);
        auto mesh = mesher.mesh().Download();

        PointCloud pcl;
        pcl.points_ = mesh->vertices_;
        pcl.normals_ = mesh->vertex_normals_;
        pcl.colors_ = mesh->vertex_colors_;

        WritePointCloudToPLY(conf.FragmentFile(fragment_idx), pcl);

        fragment_idx++; 
    }

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
