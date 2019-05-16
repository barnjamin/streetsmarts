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
#include "utils.h"

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

void MakePoseGraphForFragment(int fragment_id, Config &config) {


    PinholeCameraIntrinsic intrinsic;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);

    OdometryOption opts({20, 10, 5},
                          config.max_depth_diff,
                          config.min_depth,
                          config.max_depth);
    odometry.SetParameters(opts, 0.5f);

    RGBDImageCuda rgbd_source(config.width, config.height, config.max_depth, config.depth_factor);
    RGBDImageCuda rgbd_target(config.width, config.height, config.max_depth, config.depth_factor);

    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    registration::PoseGraph pose_graph;

    Eigen::Matrix4d trans;
    int overlap = 0;
    if(fragment_id==0){
        trans_odometry = Eigen::Matrix4d::Identity();
        pose_graph.nodes_.emplace_back(registration::PoseGraphNode(trans_odometry));
    }else{
        overlap = config.GetOverlapCount();

        PoseGraph prev_pg;
        ReadPoseGraph(config.PoseFile(fragment_id-1), prev_pg);

        std::tie(pose_graph, trans_odometry) = InitPoseGraphFromOverlap(prev_pg, overlap);
    }

    Image depth, color;
    for (int s = 0; s < config.frames_per_fragment - 1; s ++) {
        int src_frame_idx = (fragment_id * config.frames_per_fragment) + s;

        ReadImage(config.DepthFile(src_frame_idx), depth);
        ReadImage(config.ColorFile(src_frame_idx), color);

        //auto ri = CreateRGBDImageFromColorAndDepth(color, depth);

        rgbd_source.Upload(depth, color);

        int tgt_frame_idx = src_frame_idx+1;

        ReadImage(config.DepthFile(tgt_frame_idx), depth);
        ReadImage(config.ColorFile(tgt_frame_idx), color);
        rgbd_target.Upload(depth, color);

        PrintInfo("RGBD Odometry between (%d %d)\n", src_frame_idx, tgt_frame_idx);

        odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();
        odometry.Initialize(rgbd_source, rgbd_target);
        odometry.ComputeMultiScale();

        trans = odometry.transform_source_to_target_;

        Eigen::Matrix6d information = odometry.ComputeInformationMatrix();

        trans_odometry = trans * trans_odometry;

        // target_to_world
        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();

        pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
        pose_graph.edges_.emplace_back(PoseGraphEdge( 
                    s + overlap, s + overlap + 1, trans, information, false));
    }

    WritePoseGraph(config.PoseFile(fragment_id), pose_graph);
}


void MakeFullPoseGraph(Config &config) {


    PinholeCameraIntrinsic intrinsic;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);

    OdometryOption rest({20, 10, 5},
                          config.max_depth_diff,
                          config.min_depth,
                          config.max_depth);
    odometry.SetParameters(rest, 0.5f);


    RGBDImageCuda rgbd_source(config.width, config.height, config.max_depth, config.depth_factor);
    RGBDImageCuda rgbd_target(config.width, config.height, config.max_depth, config.depth_factor);

    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    registration::PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry.inverse()));

    Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();

    Image depth, color;


    for (int s = 0; s < config.frames - 1; s++) {
        for(int t=s+1; t<min(s+3, config.frames); t++) {
            ReadImage(config.DepthFile(s), depth);
            ReadImage(config.ColorFile(s), color);
            rgbd_source.Upload(depth, color);

            ReadImage(config.DepthFile(t), depth);
            ReadImage(config.ColorFile(t), color);
            rgbd_target.Upload(depth, color);

            PrintInfo("RGBD Odometry between (%d %d)\n", s, t);

            odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();
            odometry.Initialize(rgbd_source, rgbd_target);
            odometry.ComputeMultiScale();

            trans = odometry.transform_source_to_target_;

            Eigen::Matrix6d information = odometry.ComputeInformationMatrix();

            trans_odometry = trans * trans_odometry;

            if(t==s+1){
                Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();
                pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            }

            pose_graph.edges_.emplace_back(PoseGraphEdge( 
                        s , t, trans, information, false));
        
        }
    }

    WritePoseGraph(config.PoseFile(-1), pose_graph);
}

void OptimizePoseGraphForFragment(int fragment_id, Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(fragment_id), pose_graph);

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option( config.max_depth_diff, 0.25, config.loop_close_odom, 0);
    GlobalOptimizationLevenbergMarquardt optimization_method;
    GlobalOptimization(pose_graph, optimization_method, criteria, option);

    auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges(pose_graph, option);

    WritePoseGraph(config.PoseFile(fragment_id), *pose_graph_prunned);
}

void MakePointCloudForFragment(int fragment_id, Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(fragment_id), pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic_);

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);

    RGBDImageCuda rgbd(config.width, config.height, config.max_depth, config.depth_factor);

    PointCloud pcd;
    for (int i = 0; i < config.frames_per_fragment; i++) {
        Image depth, color, mask;

        int frame_idx = (config.frames_per_fragment * fragment_id) + i;

        PrintDebug("Integrating fragment: %d frame %d \n", fragment_id, frame_idx);

        ReadImage(config.DepthFile(frame_idx), depth);
        ReadImage(config.ColorFile(frame_idx), color);

        ReadImage(config.MaskFile(frame_idx), mask);

        for (int y = 0; y < mask.height_; y++) {
            for (int x = 0; x < mask.width_; x++) {
                uint8_t *p = PointerAt<uint8_t>(mask, x, y);

                if(*p!=255){
                    uint16_t *d = PointerAt<uint16_t>(depth, x,y); 
                    *d = (uint16_t) 0;

                    //float *cr = PointerAt<float>(color,x,y,0); 
                    //float *cg = PointerAt<float>(color,x,y,1); 
                    //float *cb = PointerAt<float>(color,x,y,2); 

                    //*cr = 0.0;
                    //*cg = 0.0;
                    //*cb = 0.0;
                }
            }
        }


        auto rgbd = geometry::CreateRGBDImageFromColorAndDepth(color, depth, 
                config.depth_factor, config.max_depth, false);

        auto pcd_i = geometry::CreatePointCloudFromRGBDImage(*rgbd, intrinsic_);

        //visualization::DrawGeometries({pcd_i});

        int node_id = i;
        if(fragment_id>0){
            node_id += config.GetOverlapCount(); 
        }

        Eigen::Matrix4d pose = pose_graph.nodes_[node_id].pose_.inverse();
        pcd += *pcd_i;
    }

    //pcl.Transform(Flatten(pcl));

    //WritePointCloud(config.FragmentFile(fragment_id), pcd);

    auto pcl_downsampled = VoxelDownSample(pcd, config.voxel_size);
    WritePointCloud(config.FragmentFile(fragment_id), *pcl_downsampled);
}

void IntegrateForFragment(int fragment_id, Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(fragment_id), pose_graph);

    float voxel_length = config.tsdf_cubic_size / 512.0;

    PinholeCameraIntrinsic intrinsic_;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic_);

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);
    TransformCuda trans = TransformCuda::Identity();
    ScalableTSDFVolumeCuda tsdf_volume(8, voxel_length, (float) config.tsdf_truncation);

    RGBDImageCuda rgbd(config.width, config.height, config.max_depth, config.depth_factor);

    for (int i = 0; i < config.frames_per_fragment; i++) {
        Image depth, color;
        int frame_idx = (config.frames_per_fragment * fragment_id) + i;

        PrintDebug("Integrating fragment: %d frame %d \n", fragment_id, frame_idx);

        ReadImage(config.DepthFile(frame_idx), depth);
        ReadImage(config.ColorFile(frame_idx), color);
        rgbd.Upload(depth, color);

        int node_id = i;
        if(fragment_id>0){
            node_id += config.GetOverlapCount(); 
        }

        /* Use ground truth trajectory */
        Eigen::Matrix4d pose = pose_graph.nodes_[node_id].pose_;
        trans.FromEigen(pose);

        tsdf_volume.Integrate(rgbd, intrinsic, trans);
    }

    tsdf_volume.GetAllSubvolumes();

    std::cout << tsdf_volume.active_subvolume_entry_array_.size() << std::endl;

    int subvols = tsdf_volume.active_subvolume_entry_array_.size(); 
    int max_vert = 15 * subvols;
    int max_tri = 2 * max_vert;
    ScalableMeshVolumeCuda mesher(VertexWithNormalAndColor, 8, subvols, max_vert, max_tri);

    mesher.MarchingCubes(tsdf_volume);
    auto mesh = mesher.mesh().Download();

    PointCloud pcl;
    pcl.points_ = mesh->vertices_;
    pcl.normals_ = mesh->vertex_normals_;
    pcl.colors_ = mesh->vertex_colors_;

    //pcl.Transform(Flatten(pcl));

    /** Write original fragments **/
    WritePointCloud(config.FragmentFile(fragment_id), pcl);

    /** Write downsampled thumbnail fragments **/
    //auto pcl_downsampled = VoxelDownSample(pcl, config.voxel_size);
    //WritePointCloudToPLY(config.ThumbnailFragmentFile(fragment_id), *pcl_downsampled);
}
