#include <Open3D/Open3D.h>
#include "config.h"

using namespace open3d;

int main(int argc, char **argv) {
    SetVerbosityLevel(utility::VerbosityLevel::VerboseDebug);

    Config conf(argc, argv);


    camera::PinholeCameraIntrinsic intrinsic_;
    if(!io::ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        utility::PrintError("Failed to read intrinsic\n");
        return 1; 
    }

    int frames = conf.fragments * conf.frames_per_fragment;
    for(int i = conf.img_idx; i<frames*2; i+=2){
        geometry::Image depth, color;
        io::ReadImage(conf.DepthFile(i), depth);
        io::ReadImage(conf.ColorFile(i), color);

        auto rgbd = geometry::CreateRGBDImageFromColorAndDepth(color, depth, 
                conf.depth_factor, conf.max_depth, false);

        auto pcd = geometry::CreatePointCloudFromRGBDImage(*rgbd, intrinsic_);

        visualization::DrawGeometries({pcd});
    }
    return 0;
}
