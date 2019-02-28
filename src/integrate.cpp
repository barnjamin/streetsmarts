#include <vector>
#include <string>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

#include <Core/Registration/Registration.h>
#include <Core/Registration/PoseGraph.h>

#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>

#include "config.h"

using namespace open3d;

int main(int argc, char ** argv)
{

    Config conf(argc, argv);

    cuda::TransformCuda trans = cuda::TransformCuda::Identity();
    cuda::ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, conf.tsdf_cubic_size / 512.0, conf.tsdf_truncation, trans);

    PoseGraph global_pose_graph;
    ReadPoseGraph(conf.PoseFileScene(), global_pose_graph);

    PinholeCameraIntrinsic intrinsic_;
    if(!ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        PrintError("Failed to read intrinsic\n");
        return 1; 
    }

    cuda::PinholeCameraIntrinsicCuda intrinsic(intrinsic_);

    cuda::RGBDImageCuda rgbd(conf.width, conf.height, conf.max_depth);
    for(int fragment_id=0; fragment_id<conf.GetFragmentCount(); fragment_id++){

        PoseGraph local_pose_graph;
        ReadPoseGraph(conf.PoseFile(fragment_id), local_pose_graph);


        for (int img_id = 0; img_id < conf.frames_per_fragment; img_id++) {
            Image depth, color;

            if(!ReadImage(conf.DepthFile(fragment_id, img_id), depth) ||
                !ReadImage(conf.ColorFile(fragment_id, img_id), color)) {
                PrintInfo("Failed to read %d_%d\n", fragment_id, img_id);
                continue;
            }

            rgbd.Upload(depth, color);

            Eigen::Matrix4d pose = global_pose_graph.nodes_[fragment_id].pose_ * local_pose_graph.nodes_[img_id].pose_;

            cuda::TransformCuda trans;
            trans.FromEigen(pose);

            PrintInfo("Integrating %d and %d\n", fragment_id, img_id);

            tsdf_volume.Integrate(rgbd, intrinsic, trans);
        }
    }

    PrintInfo("Getting subvolumes\n");
    tsdf_volume.GetAllSubvolumes();

    PrintInfo("Creating mesher\n");
    cuda::ScalableMeshVolumeCuda<8> mesher(100000, cuda::VertexWithNormalAndColor, 10000000, 20000000);

    PrintInfo("Meshing\n");
    mesher.MarchingCubes(tsdf_volume);

    PrintInfo("Downloading\n");
    auto mesh = mesher.mesh().Download();

    PrintInfo("Writing\n");
    WriteTriangleMeshToPLY(conf.SceneMeshFile(), *mesh);
}
