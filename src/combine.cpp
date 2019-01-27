//Optimize Pose Graph and Combine or stream chunks
#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>
#include <Cuda/Registration/RegistrationCuda.h>

int main(int argc, char *argv[])
{
    using namespace open3d;
    SetVerbosityLevel(VerbosityLevel::VerboseAlways);

    PinholeCameraTrajectory trajectory;
    ReadPinholeCameraTrajectory("/home/ben/streetsmarts/build/trajectory.json", trajectory);

    std::shared_ptr<PointCloud> pcd;

    std::shared_ptr<PointCloud> source_origin;
    std::shared_ptr<PointCloud> source, target;
    for (size_t i = 0; i < trajectory.parameters_.size(); i++) {

        char buff[DEFAULT_IO_BUFFER_SIZE];
        sprintf(buff, "/home/ben/streetsmarts/build/fragment-%d.ply", (int)i);

        if (!filesystem::FileExists(buff)) {
            continue;
        }


        source_origin = CreatePointCloudFromFile(buff);
        source_origin->Transform(trajectory.parameters_[i].extrinsic_);
        open3d::EstimateNormals(*source_origin);
        std::shared_ptr<PointCloud> source = open3d::VoxelDownSample(*source_origin, 0.05);

        if(i==0){
            pcd = source_origin;
            target = source;
            continue;
        }

        open3d::cuda::RegistrationCuda registration(TransformationEstimationType::PointToPlane);
        registration.Initialize(*source, *target, 0.07f);

        int max_iter = 10;
        for (int i = 0; i < max_iter; ++i) {
             registration.DoSingleIteration(i);
        }
        auto result = registration.DoSingleIteration(max_iter);

        //Convert back to original then transform  
        source_origin->Transform(result.transformation_ * trajectory.parameters_[i].extrinsic_.inverse());
        *pcd += *source_origin;

        //pcds.push_back(source_origin);

        target = source;
    }

    //DrawGeometriesWithCustomAnimation(pcds);

    std::shared_ptr<PointCloud> final_pcd = open3d::VoxelDownSample(*pcd, 0.03);
    auto stat_outliers = RemoveStatisticalOutliers(*final_pcd, 300, 0.9);

    WritePointCloud("final.pcd", *(std::get<0>(stat_outliers)));


    return 0;
}
