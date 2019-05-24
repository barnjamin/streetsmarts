#include <Eigen/Eigen>
#include <Open3D/Open3D.h>
#include <Open3D/Cuda/Open3DCuda.h>
#include "config.h"
#include "utils.h"
#include <Open3D/Registration/GlobalOptimization.h>
#include <Open3D/Registration/GlobalOptimizationMethod.h>

using namespace open3d;
using namespace open3d::io;
using namespace open3d::camera;
using namespace open3d::color_map;
using namespace open3d::utility;
using namespace open3d::geometry;
using namespace open3d::registration;

int main(int argc, char **argv) {
    SetVerbosityLevel(utility::VerbosityLevel::VerboseDebug);

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


    std::vector<std::string> depth_filenames, color_filenames;
    std::vector<std::shared_ptr<geometry::RGBDImage>> rgbd_images;

    filesystem::ListFilesInDirectoryWithExtension(conf.session_path + "/depth/", "png",
                                      depth_filenames);
    filesystem::ListFilesInDirectoryWithExtension(conf.session_path + "/image/", "jpg",
                                      color_filenames);
    sort(depth_filenames.begin(), depth_filenames.end());
    sort(color_filenames.begin(), color_filenames.end());

    for (int i = 0; i < conf.frames; i++) {
        Image depth, color;
        ReadImage(conf.DepthFile(i), depth);
        ReadImage(conf.ColorFile(i), color);

        auto rgbd_image = CreateRGBDImageFromColorAndDepth(
                color, depth, 1000.0, 5.0, false);

        rgbd_images.push_back(rgbd_image);
    }

    PinholeCameraIntrinsic intrinsic;
    ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic);

    PoseGraph pg;
    ReadPoseGraph(conf.PoseFile(-1), pg);

    std::vector<PinholeCameraParameters> params;
    for(int x=0; x<pg.nodes_.size(); x++){
        PinholeCameraParameters pcp;
        pcp.intrinsic_ = intrinsic;
        pcp.extrinsic_ = pg.nodes_[x].pose_.inverse();
        params.push_back(pcp);
    }


    //std::shared_ptr<PointCloud> pcd = std::make_shared<PointCloud>();
    //for(int x=0; x<params.size()/5; x++){
    //    auto pcd_i = geometry::CreatePointCloudFromRGBDImage(*rgbd_images[x], intrinsic);
    //    pcd_i->Transform(params[x].extrinsic_);
    //    *pcd += *pcd_i;
    //}

    //visualization::DrawGeometries({pcd});

        
    std::shared_ptr<PinholeCameraTrajectory> camera = std::make_shared<PinholeCameraTrajectory>();
    camera->parameters_ = params;

    //WritePinholeCameraTrajectory("stuff.log", *camera);
    //return 0;

    auto mesh = CreateMeshFromFile(conf.SceneMeshFile());

    ColorMapOptimizationOption option;
    option.maximum_iteration_ = 10;
    option.non_rigid_camera_coordinate_ = true;
    option.depth_threshold_for_visiblity_check_ = 100.0;
    option.maximum_allowable_depth_ = 150.0;

    ColorMapOptimization(*mesh, rgbd_images, *camera, option);

    //WriteTriangleMesh("color_map_after_optimization.ply", *mesh);

    visualization::DrawGeometries({mesh});

    return 0;
}
