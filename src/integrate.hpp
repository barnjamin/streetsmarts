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

            //trans.FromEigen(pose.inverse());
            trans.FromEigen(pose);

            tsdf_volume.Integrate(rgbd, intrinsic, trans);
            conf.LogStatus("INTEGRATE", frame_idx, (conf.frames_per_fragment * fragments));
        }
    }

    tsdf_volume.GetAllSubvolumes();
    ScalableMeshVolumeCuda mesher(VertexWithNormalAndColor, 8,
        tsdf_volume.active_subvolume_entry_array_.size());

    mesher.MarchingCubes(tsdf_volume);

    auto mesh = mesher.mesh().Download();

    auto tt = Flatten(*mesh);
    std::cout << tt << std::endl;
    mesh->Transform(tt);

    WriteTriangleMeshToPLY(conf.SceneMeshFile(), *mesh);
}
