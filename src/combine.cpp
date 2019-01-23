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

    const int NUM_OF_COLOR_PALETTE = 5;
    Eigen::Vector3d color_palette[NUM_OF_COLOR_PALETTE] = {
        Eigen::Vector3d(255, 180, 0) / 255.0,
        Eigen::Vector3d(0, 166, 237) / 255.0,
        Eigen::Vector3d(246, 81, 29) / 255.0,
        Eigen::Vector3d(127, 184, 0) / 255.0,
        Eigen::Vector3d(13, 44, 84) / 255.0,
    };


    std::string dirname =  "/home/ben/streetsmarts/dumps/latest"; 

    PinholeCameraIntrinsic intrinsics;
    ReadIJsonConvertible(dirname + "/intrinsic.json", intrinsics);

    PinholeCameraTrajectory trajectory;
    ReadPinholeCameraTrajectory("/home/ben/streetsmarts/build/trajectory.json", trajectory);

    std::vector<PinholeCameraParameters> newparams(trajectory.parameters_.size());

    std::shared_ptr<PointCloud> source_origin;
    std::shared_ptr<PointCloud> target_origin;
    for (size_t i = 0; i < trajectory.parameters_.size()-1; i++) {
        char buff[DEFAULT_IO_BUFFER_SIZE];

        sprintf(buff, "/home/ben/streetsmarts/build/fragment-%d.ply", (int)i);

        if (!filesystem::FileExists(buff)) {
            continue;
        }

        source_origin = CreatePointCloudFromFile(buff);
        source_origin->Transform(trajectory.parameters_[i].extrinsic_);

        if(i==0){
            target_origin = source_origin;
            continue;
        }

        open3d::EstimateNormals(*source_origin);
        open3d::EstimateNormals(*target_origin);

        auto source = open3d::VoxelDownSample(*source_origin, 0.05);
        auto target = open3d::VoxelDownSample(*target_origin, 0.05);

        open3d::cuda::RegistrationCuda registration(TransformationEstimationType::PointToPoint);
        registration.Initialize(*source, *target, 0.07f);

        for (int i = 0; i < 29; ++i) {
             registration.DoSingleIteration(i);
        }
        auto result = registration.DoSingleIteration(29);

        PinholeCameraParameters params;
        params.intrinsic_ =  intrinsics;
        params.extrinsic_ = result.transformation_.inverse();
        newparams.emplace_back(params);

        //VisualizeRegistration(*source, *target, result.transformation_);
    }

    WritePinholeCameraTrajectory("new-trajectory.json", trajectory);

    
    //DrawGeometriesWithCustomAnimation(pcds);

    return 0;
}
