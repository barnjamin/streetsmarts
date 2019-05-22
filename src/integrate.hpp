#include <vector>
#include <string>

#include <Open3D/Open3D.h>
#include <Cuda/Open3DCuda.h>

#include "config.h"

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::geometry;
using namespace open3d::integration;
using namespace open3d::registration;
using namespace open3d::utility;
using namespace open3d::camera;
using namespace open3d::io;

void IntegrateScene(Config& conf){

    TransformCuda trans = TransformCuda::Identity();

    float voxel_length = conf.tsdf_cubic_size / 512.0;

    ScalableTSDFVolumeCuda tsdf_volume(8, voxel_length, (float) conf.tsdf_truncation);

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
    int max_vert = 15 * subvols;
    int max_tri = 3 * max_vert;
    ScalableMeshVolumeCuda mesher(VertexWithNormalAndColor, 8, subvols, max_vert, max_tri);

    mesher.MarchingCubes(tsdf_volume);

    auto mesh = mesher.mesh().Download();

    //auto tt = Flatten(*mesh);
    //std::cout << tt << std::endl;

    //for(int x=0;x<full_pose_graph.nodes_.size(); x++){
    //    full_pose_graph.nodes_[x].pose_  *= tt;
    //}

    WritePoseGraph(conf.PoseFileSceneRectified(), full_pose_graph);

    //mesh->Transform(tt);

    //auto bbox = LineSetFromBBox(mesh->GetMinBound(), mesh->GetMaxBound());
    //visualization::DrawGeometries({mesh, bbox});

    WriteTriangleMeshToPLY(conf.SceneMeshFile(), *mesh);
}
