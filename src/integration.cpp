#include <vector>
#include <string>

#include <Open3D/Open3D.h>
#include <Open3D/Cuda/Open3DCuda.h>

#include "config.h"
#include "utils.h"
#include "integrate.hpp"

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::geometry;
using namespace open3d::integration;
using namespace open3d::registration;
using namespace open3d::utility;
using namespace open3d::camera;
using namespace open3d::io;

void MakePointCloudForScene(Config& conf){

    PoseGraph full_pose_graph;
    PoseGraph global_pose_graph;
    ReadPoseGraph(conf.PoseFileScene(), global_pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    if(!ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        PrintError("Failed to read intrinsic\n");
        return;
    }

    PointCloud pcd;
    int fragments = conf.GetFragmentCount();
    for(int fragment_id=0; fragment_id<fragments; fragment_id++){

        PoseGraph local_pose_graph;
        ReadPoseGraph(conf.PoseFile(fragment_id), local_pose_graph);

        for (int img_id = 0; img_id < conf.frames_per_fragment; img_id++) {
            Image depth, color, mask;

            int frame_idx = (conf.frames_per_fragment * fragment_id) + img_id;

            if(!ReadImage(conf.DepthFile(frame_idx), depth) ||
                !ReadImage(conf.ColorFile(frame_idx), color)) {
                PrintInfo("Failed to read frame_idx: %d\n", frame_idx);
                continue;
            }

            MaskRoad(conf, depth, frame_idx);

            int node_id = img_id;
            if(fragment_id>0){
                node_id += conf.GetOverlapCount(); 
            }

            auto rgbd = geometry::CreateRGBDImageFromColorAndDepth(color, depth, 
                conf.depth_factor, conf.max_depth, false);

            auto pcd_i = geometry::CreatePointCloudFromRGBDImage(*rgbd, intrinsic_);


            Eigen::Matrix4d pose = global_pose_graph.nodes_[fragment_id].pose_ * local_pose_graph.nodes_[node_id].pose_;

            pcd_i->Transform(pose);

            pcd += *pcd_i;

            //Its inverted here already
            full_pose_graph.nodes_.push_back(pose);

            conf.LogStatus("INTEGRATE", frame_idx, (conf.frames_per_fragment * fragments));
        }
    }

    WritePoseGraph(conf.PoseFileSceneRectified(), full_pose_graph);

    auto pcl_downsampled = VoxelDownSample(pcd, conf.voxel_size);
    //EstimateNormals(*pcl_downsampled, KDTreeSearchParamRadius(conf.voxel_size * 1.5));
    WritePointCloud(conf.ScenePointCloudFile(), *pcl_downsampled);
}


int main(int argc, char * argv[]) {
    Config conf;
    
    // Assume json
    if(argc==2){
        std::string config_path = argv[1];
        if(!open3d::io::ReadIJsonConvertible(config_path, conf)) {
            open3d::utility::PrintError("Failed to read config\n");
            return 1;
        }
    }else{
        conf = Config(argc, argv);
    }

    //IntegrateScene(conf);
    MakePointCloudForScene(conf);
}
