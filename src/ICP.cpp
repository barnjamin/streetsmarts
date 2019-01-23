//
// Created by wei on 1/3/19.
//
#include <iostream>
#include <fstream>
#include <memory>
#include <iomanip>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

#include <Core/Utility/Timer.h>
#include <Cuda/Registration/RegistrationCuda.h>
#include <Core/Camera/PinholeCameraTrajectory.h>

using namespace open3d;

void VisualizeRegistration(const open3d::PointCloud &source,
                           const open3d::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    using namespace open3d;
    std::shared_ptr<PointCloud> source_transformed_ptr(new PointCloud);
    std::shared_ptr<PointCloud> target_ptr(new PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    source_transformed_ptr->Transform(Transformation);
    DrawGeometries({source_transformed_ptr, target_ptr}, "Registration result");
}

int main(int argc, char **argv) {
    open3d::SetVerbosityLevel(open3d::VerbosityLevel::VerboseDebug);


    PinholeCameraTrajectory trajectory;
    ReadPinholeCameraTrajectory("/home/ben/streetsmarts/build/trajectory.json", trajectory);

    std::vector<std::shared_ptr<PointCloud>> pcds;

    for (size_t i = 0; i < trajectory.parameters_.size(); i++) {
        char buff[DEFAULT_IO_BUFFER_SIZE];
        sprintf(buff, "/home/ben/streetsmarts/build/fragment-%d.ply", (int)i);
        if (filesystem::FileExists(buff)) {
            auto pcd = CreatePointCloudFromFile(buff);
            pcd->Transform(trajectory.parameters_[i].extrinsic_);
            pcds.push_back(pcd);
        }

    }


    for(int i =0; i<pcds.size()-2; i++){
        auto source_origin = pcds[i];
        auto target_origin = pcds[i+1];

        open3d::EstimateNormals(*source_origin);
        open3d::EstimateNormals(*target_origin);

        auto source = open3d::VoxelDownSample(*source_origin, 0.05);
        auto target = open3d::VoxelDownSample(*target_origin, 0.05);

        auto result = open3d::RegistrationICP(*source, *target, 0.07);

        open3d::cuda::RegistrationCuda registration(
            open3d::TransformationEstimationType::PointToPoint);

        registration.Initialize(*source, *target, 0.07f);

        VisualizeRegistration(*source, *target, registration.transform_source_to_target_);
        for (int i = 0; i < 30; ++i) {
            auto result = registration.DoSingleIteration(i);
        }
        VisualizeRegistration(*source, *target, registration.transform_source_to_target_);
        VisualizeRegistration(*source, *target, result.transformation_);

        std::cout << source->points_.size() << std::endl;
    }
}
