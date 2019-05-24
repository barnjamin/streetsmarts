#include <vector>
#include <string>

#include <Open3D/Open3D.h>
#include <Open3D/Cuda/Open3DCuda.h>

#include "config.h"

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::geometry;
using namespace open3d::integration;
using namespace open3d::registration;
using namespace open3d::utility;
using namespace open3d::camera;
using namespace open3d::io;

void IntegrateSceneCPU(Config& conf){
    float voxel_length = conf.tsdf_cubic_size / 512.0;
    ScalableTSDFVolume tsdf_volume(voxel_length, (float) conf.tsdf_truncation, TSDFVolumeColorType::RGB8);

    PoseGraph global_pose_graph;
    ReadPoseGraph(conf.PoseFileScene(), global_pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    if(!ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        PrintError("Failed to read intrinsic\n");
        return;
    }

    int fragments = conf.GetFragmentCount();
    for(int fragment_id=0; fragment_id<fragments; fragment_id++){

        PoseGraph local_pose_graph;
        ReadPoseGraph(conf.PoseFile(fragment_id), local_pose_graph);

        for (int img_id = 0; img_id < conf.frames_per_fragment; img_id++) {
            Image depth, color;

            int frame_idx = (conf.frames_per_fragment * fragment_id) + img_id;

            if(!ReadImage(conf.DepthFile(frame_idx), depth) ||
                !ReadImage(conf.ColorFile(frame_idx), color)) {
                PrintInfo("Failed to read frame_idx: %d\n", frame_idx);
                continue;
            }

            auto rgbd = geometry::CreateRGBDImageFromColorAndDepth(color, depth, 1000.0, 4.0, false);

            int node_id = img_id;
            if(fragment_id>0){
                node_id += conf.GetOverlapCount(); 
            }

            Eigen::Matrix4d pose = global_pose_graph.nodes_[fragment_id].pose_ * local_pose_graph.nodes_[node_id].pose_;


            tsdf_volume.Integrate(*rgbd, intrinsic_, pose);
            conf.LogStatus("INTEGRATE", frame_idx, (conf.frames_per_fragment * fragments));
        }
        break;
    }

    auto mesh = tsdf_volume.ExtractTriangleMesh();

    WriteTriangleMesh(conf.SceneMeshFile(), *mesh);
}

void IntegrateScene(Config& conf){

    TransformCuda trans = TransformCuda::Identity();

    float voxel_length = conf.tsdf_cubic_size / 512.0;

    ScalableTSDFVolumeCuda tsdf_volume(8, voxel_length, (float) conf.tsdf_truncation, trans, 40000, 80000);

    PoseGraph full_pose_graph;

    PoseGraph global_pose_graph;
    ReadPoseGraph(conf.PoseFileScene(), global_pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    if(!ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        PrintError("Failed to read intrinsic\n");
        return;
    }

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);

    RGBDImageCuda rgbd(conf.width, conf.height, conf.max_depth, conf.depth_factor);
    int fragments = conf.GetFragmentCount();
    for(int fragment_id=0; fragment_id<fragments; fragment_id++){

        PoseGraph local_pose_graph;
        ReadPoseGraph(conf.PoseFile(fragment_id), local_pose_graph);

        TransformCuda trans;
        for (int img_id = 0; img_id < conf.frames_per_fragment; img_id++) {
            Image depth, color;

            int frame_idx = (conf.frames_per_fragment * fragment_id) + img_id;

            if(!ReadImage(conf.DepthFile(frame_idx), depth) ||
                !ReadImage(conf.ColorFile(frame_idx), color)) {
                PrintInfo("Failed to read frame_idx: %d\n", frame_idx);
                continue;
            }

            MaskHorizon(depth);

            rgbd.Upload(depth, color);

            int node_id = img_id;
            if(fragment_id>0){
                node_id += conf.GetOverlapCount(); 
            }

            Eigen::Matrix4d pose = global_pose_graph.nodes_[fragment_id].pose_ * local_pose_graph.nodes_[node_id].pose_;

            //Its inverted here already
            full_pose_graph.nodes_.push_back(pose);

            trans.FromEigen(pose);

            tsdf_volume.Integrate(rgbd, intrinsic, trans);
            conf.LogStatus("INTEGRATE", frame_idx, (conf.frames_per_fragment * fragments));
        }
    }

    tsdf_volume.GetAllSubvolumes();

    int subvols = tsdf_volume.active_subvolume_entry_array_.size(); 
     ScalableMeshVolumeCuda mesher(
        VertexWithNormalAndColor, 8,
        subvols, 20000000, 40000000);

    mesher.MarchingCubes(tsdf_volume);

    auto mesh = mesher.mesh().Download();

    //auto tt = Flatten(*mesh);
    //mesh->Transform(tt);

    //auto bbox = LineSetFromBBox(mesh->GetMinBound(), mesh->GetMaxBound());
    //visualization::DrawGeometries({mesh, bbox});

    WriteTriangleMeshToPLY(conf.SceneMeshFile(), *mesh);
}

void IntegrateSceneFromFullPG(Config& conf){

    TransformCuda trans = TransformCuda::Identity();

    float voxel_length = conf.tsdf_cubic_size / 512.0;

    ScalableTSDFVolumeCuda tsdf_volume(8, voxel_length, 
            (float) conf.tsdf_truncation, trans, 100000, 200000);

    PoseGraph full_pose_graph;
    ReadPoseGraph(conf.PoseFile(-1), full_pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    if(!ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        PrintError("Failed to read intrinsic\n");
        return;
    }

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);

    RGBDImageCuda rgbd(conf.width, conf.height, conf.max_depth, conf.depth_factor);
    for (int img_id = 0; img_id < conf.frames; img_id++) {
        Image depth, color;

        if(!ReadImage(conf.DepthFile(img_id), depth) ||
            !ReadImage(conf.ColorFile(img_id), color)) {
            PrintInfo("Failed to read frame_idx: %d\n", img_id);
            continue;
        }

        MaskHorizon(depth);

        rgbd.Upload(depth, color);

        Eigen::Matrix4d pose = full_pose_graph.nodes_[img_id].pose_;

        trans.FromEigen(pose);

        tsdf_volume.Integrate(rgbd, intrinsic, trans);
        PrintInfo("%d\n",img_id);
    }

    tsdf_volume.GetAllSubvolumes();

    int subvols = tsdf_volume.active_subvolume_entry_array_.size(); 
     ScalableMeshVolumeCuda mesher(
        VertexWithNormalAndColor, 8,
        subvols, 20000000, 40000000);

    mesher.MarchingCubes(tsdf_volume);

    auto mesh = mesher.mesh().Download();

    auto tt = Flatten(*mesh);
    mesh->Transform(tt);

    //auto bbox = LineSetFromBBox(mesh->GetMinBound(), mesh->GetMaxBound());
    //visualization::DrawGeometries({mesh, bbox});

    WriteTriangleMeshToPLY(conf.SceneMeshFile(), *mesh);
}
