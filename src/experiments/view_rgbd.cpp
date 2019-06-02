#include <Open3D/Open3D.h>
#include "config.h"
#include "utils.h"

using namespace open3d;

int main(int argc, char **argv) {
    SetVerbosityLevel(utility::VerbosityLevel::VerboseDebug);

    Config conf(argc, argv);


    camera::PinholeCameraIntrinsic intrinsic_;
    if(!io::ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        utility::PrintError("Failed to read intrinsic\n");
        return 1; 
    }


    //double fx = intrinsic_.intrinsic_matrix_(0, 0);
    //double fy = intrinsic_.intrinsic_matrix_(1, 1);
    //double cx = intrinsic_.intrinsic_matrix_(0, 2);
    //double cy = intrinsic_.intrinsic_matrix_(1, 2)-100;

    //std::cout << fx << " : " << fy  << std::endl;
    //std::cout << cx << " : " << cy << std::endl;
    //intrinsic_.SetIntrinsics(intrinsic_.width_, intrinsic_.height_, fx, fy, cx, cy);


    for(int i = conf.img_idx; i<conf.frames; i++){

        geometry::Image depth, color;

        io::ReadImage(conf.DepthFile(i), depth);
        io::ReadImage(conf.ColorFile(i), color);

        utility::PrintInfo("%d %f %f\n", i, conf.depth_factor, conf.max_depth);

        auto rgbd = geometry::CreateRGBDImageFromColorAndDepth(color, depth, 
                conf.depth_factor, conf.max_depth, false);

        auto pcd = geometry::CreatePointCloudFromRGBDImage(*rgbd, intrinsic_);
        auto bbox = LineSetFromBBox(pcd->GetMinBound(), pcd->GetMaxBound());

        visualization::DrawGeometries({pcd, bbox});
    }
    return 0;
}
