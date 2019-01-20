//Optimize Pose Graph and Combine or stream chunks
#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

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

    PinholeCameraTrajectory trajectory;

    ReadPinholeCameraTrajectory("/home/ben/streetsmarts/build/trajectory.log", trajectory);

    std::vector<std::shared_ptr<const Geometry>> pcds;

    for (size_t i = 0; i < trajectory.parameters_.size(); i++) {

        char buff[DEFAULT_IO_BUFFER_SIZE];

        sprintf(buff, "/home/ben/streetsmarts/build/fragment-%d.ply", (int)i);

        if (filesystem::FileExists(buff)) {
            auto pcd = CreatePointCloudFromFile(buff);
            pcd->Transform(trajectory.parameters_[i].extrinsic_);
            pcd->colors_.clear();
            if ((int)i < NUM_OF_COLOR_PALETTE) {
                pcd->colors_.resize(pcd->points_.size(), color_palette[i]);
            } else {
                pcd->colors_.resize(pcd->points_.size(), (Eigen::Vector3d::Random() + Eigen::Vector3d::Constant(1.0)) * 0.5);
            }
            pcds.push_back(pcd);
        }

    }

    DrawGeometriesWithCustomAnimation(pcds);

    return 0;
}
