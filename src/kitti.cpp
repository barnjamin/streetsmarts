#include <iostream>
#include <Visualization/Visualization.h>
#include <Core/Core.h>
#include <IO/IO.h>

#include <Cuda/Registration/RegistrationCuda.h>
#include <Cuda/Registration/ColoredICPCuda.h>
#include <Cuda/Registration/FastGlobalRegistrationCuda.h>
#include <Core/Registration/PoseGraph.h>
#include <Core/Registration/GlobalOptimization.h>

#include "config.h"

std::shared_ptr<open3d::PointCloud> readKittiVelodyne(std::string& fileName){
    std::shared_ptr<open3d::PointCloud> pc = std::make_shared<open3d::PointCloud>();

    std::ifstream input(fileName.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<fileName<<std::endl;
        return pc;
    }

    float bbmax = 30.0;
    for (int iter=0; input.good() && !input.eof(); iter++) {
        float x,y,z;
        float i;

        input.read((char *) &x, sizeof(float));
        input.read((char *) &y, sizeof(float));
        input.read((char *) &z, sizeof(float));
        input.read((char *) &i, sizeof(float));


        if(abs(x)>bbmax || abs(y)>bbmax || abs(z)>bbmax){
            continue; 
        }

        pc->points_.push_back(Eigen::Vector3d(x,y,z));
        pc->colors_.push_back(Eigen::Vector3d(i, i, i));
    }
    input.close();

    EstimateNormals(*pc, open3d::KDTreeSearchParamKNN());

    auto result = RemoveStatisticalOutliers(*pc, 50, 0.3);

    return std::get<0>(result);
}

int main(int argc, char ** argv) 
{

    Config config(argc, argv);

    std::string bin_dir = "/home/ben/kitti-sync/velodyne_points/data";
    std::vector<std::string> bins;
    open3d::filesystem::ListFilesInDirectory(bin_dir, bins);

    std::shared_ptr<open3d::PointCloud> src;
    std::shared_ptr<open3d::PointCloud> tgt;

    for(int i=0; i<bins.size(); i++){
        src = readKittiVelodyne(bins[i]);
        if(i == 0) { 
            tgt = src;
            continue; 
        }

        open3d::cuda::RegistrationCuda registration(open3d::TransformationEstimationType::PointToPlane);

        registration.Initialize(*src, *tgt, (float) config.voxel_size * 1.4f, Eigen::Matrix4d::Identity());

        registration.ComputeICP(256);

        std::cout << registration.transform_source_to_target_ << std::endl;
        std::cout << registration.ComputeInformationMatrix() << std::endl;


        tgt->PaintUniformColor(Eigen::Vector3d(0,0,1.0));

        src->Transform(registration.transform_source_to_target_);
        src->PaintUniformColor(Eigen::Vector3d(1.0,0,0));

        open3d::DrawGeometries({src,tgt});



        tgt = src;
    }

}
