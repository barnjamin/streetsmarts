#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <csignal>

#include <string>
#include <iostream>
#include <sstream>
#include <bitset>
#include <fstream>
#include <ublox/ublox.h>
#include <mutex>

#include <Open3D/Open3D.h>
#include <Cuda/Open3DCuda.h>

#include <Open3D/Registration/GlobalOptimization.h>
#include <Open3D/Registration/GlobalOptimizationMethod.h>



#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include "config.h"

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::utility;
using namespace open3d::geometry;
using namespace open3d::registration;
using namespace open3d::integration;
using namespace open3d::odometry;
using namespace open3d::io;
using namespace open3d::camera;
using namespace cv;
using namespace std;

Eigen::Matrix4d PredictTransform(const PoseGraph & pg, int src, int tgt) {
    Eigen::Matrix4d src_node_pose = Eigen::Matrix4d(pg.nodes_[src].pose_);
    Eigen::Matrix4d tgt_node_pose = Eigen::Matrix4d(pg.nodes_[tgt].pose_);
    return tgt_node_pose.inverse() * src_node_pose;
}

void MakePoseGraphForSession(Config &config) {

    PinholeCameraIntrinsic intrinsic;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);


    // world_to_source
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    registration::PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(registration::PoseGraphNode(trans_odometry));

    // Setup queue for previous transforms
    std::queue<Eigen::Matrix4d> lookback_queue;

    Image depth, color;

    RGBDImageCuda rgbd_source(config.width, config.height, config.max_depth, config.depth_factor);
    RGBDImageCuda rgbd_target(config.width, config.height, config.max_depth, config.depth_factor);

    bool success;
    Eigen::Matrix4d mat;
    std::vector<std::vector<float>> losses;

    for (int frame_idx = 0; frame_idx < config.frames-1;  ++frame_idx) {
        int tgt_frame_idx = frame_idx + 1;
        Eigen::Matrix4d seed_trans = Eigen::Matrix4d::Identity();
        for(int lookback = 0; lookback < config.rgbd_lookback; ++lookback){
            int src_frame_idx = frame_idx - lookback;

            if(src_frame_idx<0) continue;

            if(lookback>0) 
                seed_trans = PredictTransform(pose_graph, src_frame_idx, tgt_frame_idx);

            PrintInfo("RGBD Odometry between (%d %d)\n", src_frame_idx, tgt_frame_idx);

            ReadImage(config.DepthFile(src_frame_idx), depth);
            ReadImage(config.ColorFile(src_frame_idx), color);
            rgbd_source.Upload(depth, color);

            ReadImage(config.DepthFile(tgt_frame_idx), depth);
            ReadImage(config.ColorFile(tgt_frame_idx), color);
            rgbd_target.Upload(depth, color);

            if(lookback==0){
                odometry.SetParameters(OdometryOption({20, 10, 5},
                                                  config.max_depth_diff,
                                                  config.min_depth,
                                                  config.max_depth), 0.5f);
            }else{
                odometry.SetParameters(OdometryOption({10, 5},
                                                  config.max_depth_diff,
                                                  config.min_depth,
                                                  config.max_depth), 0.5f);
            }

            odometry.transform_source_to_target_ = seed_trans;

            odometry.Initialize(rgbd_source, rgbd_target);
            odometry.ComputeMultiScale();

            Eigen::Matrix4d trans = odometry.transform_source_to_target_;

            //Only need this on the first one
            if(lookback == 0){
                trans_odometry = trans * trans_odometry;
                Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();
                pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            }

            Eigen::Matrix6d information = odometry.ComputeInformationMatrix();

            pose_graph.edges_.emplace_back(PoseGraphEdge( 
                        src_frame_idx, tgt_frame_idx, trans, information, false));

        }
    }

    WritePoseGraph(config.PoseFile(0), pose_graph);
}

void OptimizePoseGraphForSession(Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(0), pose_graph);

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option(config.max_depth_diff, 0.25, config.loop_close_odom, 0);
    GlobalOptimizationLevenbergMarquardt optimization_method;
    GlobalOptimization(pose_graph, optimization_method, criteria, option);

    auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges(pose_graph, option);

    WritePoseGraph(config.PoseFile(0), *pose_graph_prunned);
}

void IntegrateForFragment(int fragment_id, Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(fragment_id), pose_graph);

    float voxel_length = config.tsdf_cubic_size / 512.0;

    PinholeCameraIntrinsic intrinsic_;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic_);

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);
    TransformCuda trans = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume( 20000, 400000, 
            voxel_length, (float) config.tsdf_truncation, trans);

    RGBDImageCuda rgbd(config.width, config.height, config.max_depth, config.depth_factor);

    for (int i = 0; i < config.frames_per_fragment; i++) {

        Image depth, color;

        int frame_idx;
        if(fragment_id == 0){
            frame_idx = (fragment_id * config.frames_per_fragment) + i;
        } else {
            frame_idx = ((fragment_id * config.frames_per_fragment) - 
                                (int)((float)config.frames_per_fragment / (float)config.overlap_factor))+ i;
        }

        PrintDebug("Integrating fragment: %d frame %d \n", fragment_id, frame_idx);

        ReadImage(config.DepthFile(frame_idx), depth);
        ReadImage(config.ColorFile(frame_idx), color);
        rgbd.Upload(depth, color);

        /* Use ground truth trajectory */
        Eigen::Matrix4d pose = pose_graph.nodes_[i].pose_;
        trans.FromEigen(pose);

        tsdf_volume.Integrate(rgbd, intrinsic, trans);
    }

    tsdf_volume.GetAllSubvolumes();

    ScalableMeshVolumeCuda<8> mesher( 
            tsdf_volume.active_subvolume_entry_array().size(), 
            VertexWithNormalAndColor, 10000000, 20000000);

    mesher.MarchingCubes(tsdf_volume);
    auto mesh = mesher.mesh().Download();

    PointCloud pcl;
    pcl.points_ = mesh->vertices_;
    pcl.normals_ = mesh->vertex_normals_;
    pcl.colors_ = mesh->vertex_colors_;

    /** Write original fragments **/
    WritePointCloud(config.FragmentFile(fragment_id), pcl);

    /** Write downsampled thumbnail fragments **/
    //auto pcl_downsampled = VoxelDownSample(pcl, config.voxel_size);
    //WritePointCloudToPLY(config.ThumbnailFragmentFile(fragment_id), *pcl_downsampled);
}
