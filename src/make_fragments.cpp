#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <csignal>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Core/Registration/PoseGraph.h>
#include <Core/Registration/GlobalOptimization.h>
#include <Core/Utility/Timer.h>

#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>


#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include "utils.h" 
#include "pose.h"
#include "config.h"

using namespace open3d;
using namespace open3d::cuda;
using namespace cv;
using namespace std;

void MakePoseGraphForFragment(int fragment_id, Config &config) {


    PinholeCameraIntrinsic intrinsic;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);
    odometry.SetParameters(OdometryOption({20, 10, 5},
                                          config.max_depth_diff,
                                          config.min_depth,
                                          config.max_depth), 0.5f);

    RGBDImageCuda rgbd_source(config.width, config.height, config.max_depth, config.depth_factor);
    RGBDImageCuda rgbd_target(config.width, config.height, config.max_depth, config.depth_factor);

    // world_to_source
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry));

    for (int s = 0; s < config.frames_per_fragment-1; ++s) {
        Image depth, color;

        ReadImage(config.DepthFile(fragment_id, s), depth);
        ReadImage(config.ColorFile(fragment_id, s), color);

        rgbd_source.Upload(depth, color);

        int t = s + 1;

        ReadImage(config.DepthFile(fragment_id, t), depth);
        ReadImage(config.ColorFile(fragment_id, t), color);
        rgbd_target.Upload(depth, color);

        PrintInfo("RGBD Odometry between (%d %d)\n", s, t);
        odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();
        odometry.Initialize(rgbd_source, rgbd_target);
        odometry.ComputeMultiScale();

        Eigen::Matrix4d trans = odometry.transform_source_to_target_;
        Eigen::Matrix6d information = odometry.ComputeInformationMatrix();

        // source_to_target * world_to_source = world_to_target
        trans_odometry = trans * trans_odometry;

        // target_to_world
        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();

        pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
        pose_graph.edges_.emplace_back(PoseGraphEdge( s , t , trans, information, false));
    }


    WritePoseGraph(config.PoseFile(fragment_id), pose_graph);
}

void OptimizePoseGraphForFragment(int fragment_id, Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(fragment_id), pose_graph);

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option( config.max_depth_diff, 0.25, config.preference_loop_closure_odometry, 0);
    GlobalOptimizationLevenbergMarquardt optimization_method;
    GlobalOptimization(pose_graph, optimization_method, criteria, option);

    auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges(pose_graph, option);

    WritePoseGraph(config.PoseFile(fragment_id), *pose_graph_prunned);
}

void IntegrateForFragment(int fragment_id, Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFile(fragment_id), pose_graph);

    float voxel_length = config.tsdf_cubic_size / 512.0;


    PinholeCameraIntrinsic intrinsic_;
    ReadIJsonConvertible(config.IntrinsicFile(), intrinsic_);

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);
    TransformCuda trans = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume( 20000, 400000, voxel_length, (float) config.tsdf_truncation, trans);

    RGBDImageCuda rgbd(config.width, config.height, config.max_depth, config.depth_factor);

    for (int i = 0; i < config.frames_per_fragment; ++i) {
        PrintDebug("Integrating frame %d ...\n", i);

        Image depth, color;
        ReadImage(config.DepthFile(fragment_id, i), depth);
        ReadImage(config.ColorFile(fragment_id, i), color);
        rgbd.Upload(depth, color);

        /* Use ground truth trajectory */
        Eigen::Matrix4d pose = pose_graph.nodes_[i].pose_;
        trans.FromEigen(pose);

        tsdf_volume.Integrate(rgbd, intrinsic, trans);
    }

    tsdf_volume.GetAllSubvolumes();
    ScalableMeshVolumeCuda<8> mesher( tsdf_volume.active_subvolume_entry_array().size(), VertexWithNormalAndColor, 10000000, 20000000);
    mesher.MarchingCubes(tsdf_volume);
    auto mesh = mesher.mesh().Download();

    PointCloud pcl;
    pcl.points_ = mesh->vertices_;
    pcl.normals_ = mesh->vertex_normals_;
    pcl.colors_ = mesh->vertex_colors_;

    /** Write original fragments **/
    WritePointCloudToPLY(config.FragmentFile(fragment_id), pcl);

    /** Write downsampled thumbnail fragments **/
    auto pcl_downsampled = VoxelDownSample(pcl, config.voxel_size);
    WritePointCloudToPLY(config.ThumbnailFragmentFile(fragment_id), *pcl_downsampled);
}

int main(int argc, char * argv[])
{
    Config conf(argc, argv);

    Timer timer;
    timer.Start();

    for (int i = 0; i < conf.fragments; ++i) {

        //PrintInfo("Processing fragment %d / %d\n", i, num_fragments - 1);

        MakePoseGraphForFragment(i, conf);
        //OptimizePoseGraphForFragment(i, conf);
        IntegrateForFragment(i, conf);

    }

    timer.Stop();
    PrintInfo("MakeFragment takes %.3f s\n", timer.GetDuration() / 1000.0f);
}
