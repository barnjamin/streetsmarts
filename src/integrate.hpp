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

    ScalableTSDFVolumeCuda<8> tsdf_volume(100000, 200000,
            voxel_length, (float) conf.tsdf_truncation, trans);

    PoseGraph global_pose_graph;
    ReadPoseGraph(conf.PoseFileScene(), global_pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    if(!ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        PrintError("Failed to read intrinsic\n");
        return;
    }

    PinholeCameraIntrinsicCuda intrinsic(intrinsic_);

    PoseGraph local_pose_graph;
    ReadPoseGraph(conf.PoseFile(0), local_pose_graph);

    RGBDImageCuda rgbd(conf.width, conf.height, conf.final_max_depth, conf.depth_factor);

    for (int frame_idx = 0; frame_idx < conf.frames; frame_idx++) {
        Image depth, color;

        if(!ReadImage(conf.DepthFile(frame_idx), depth) ||
            !ReadImage(conf.ColorFile(frame_idx), color)) {
            PrintInfo("Failed to read frame_idx: %d\n", frame_idx);
            continue;
        }


        rgbd.Upload(depth, color);

        int fragment_id = conf.GetFragmentIdForFrame(frame_idx);
        Eigen::Matrix4d pose = global_pose_graph.nodes_[fragment_id].pose_ * local_pose_graph.nodes_[frame_idx].pose_;

        //trans.FromEigen(pose.inverse());
        trans.FromEigen(pose);

        tsdf_volume.Integrate(rgbd, intrinsic, trans);
        //conf.LogStatus("INTEGRATE", frame_idx, (conf.frames_per_fragment * conf.fragments));
    }

    tsdf_volume.GetAllSubvolumes();
    ScalableMeshVolumeCuda<8> mesher(
        tsdf_volume.active_subvolume_entry_array().size(),
        VertexWithNormalAndColor, 10000000, 20000000);

    mesher.MarchingCubes(tsdf_volume);

    auto mesh = mesher.mesh().Download();

    WriteTriangleMeshToPLY(conf.SceneMeshFile(), *mesh);
}
