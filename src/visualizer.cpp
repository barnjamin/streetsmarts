// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include <iostream>
#include <memory>
#include <thread>

#include <Core/Core.h>
#include <IO/IO.h>
#include <Visualization/Visualization.h>

int main(int argc, char *argv[])
{
    using namespace open3d;

    SetVerbosityLevel(VerbosityLevel::VerboseAlways);

    std::vector<std::shared_ptr<const Geometry>> pcds;

    auto cloud_ptr = std::make_shared<PointCloud>();
    if (ReadPointCloud(argv[1], *cloud_ptr)) {
        PrintWarning("Successfully read %s\n", argv[2]);
    } else {
        PrintError("Failed to read %s\n\n", argv[2]);
        return 1;
    }

    Eigen::Matrix4d below;
    below << 1,0,0,0,
             0,1,0,0.1,
             0,0,1,0,
             0,0,0,1;

    cloud_ptr->Transform(below);
    pcds.push_back(cloud_ptr);


    auto cloud_ptr_orig = std::make_shared<PointCloud>();
    if (ReadPointCloud(argv[1], *cloud_ptr_orig)) {
        PrintWarning("Successfully read %s\n", argv[2]);
    } else {
        PrintError("Failed to read %s\n\n", argv[2]);
        return 1;
    }
    Eigen::Matrix4d beside;
    beside << 1,0,0,-10,
             0,1,0,0,
             0,0,1,0,
             0,0,0,1;
    cloud_ptr_orig->Transform(beside);
    pcds.push_back(cloud_ptr_orig);

    for(int i=2; i<argc; i++){
        auto cloud_ptr = std::make_shared<PointCloud>();
        if (ReadPointCloud(argv[i], *cloud_ptr)) {
            PrintWarning("Successfully read %s\n", argv[2]);
        } else {
            PrintError("Failed to read %s\n\n", argv[2]);
            return 1;
        }

        cloud_ptr->colors_.resize(cloud_ptr->points_.size());
        for(int idx=0; idx<cloud_ptr->points_.size(); idx++)
            cloud_ptr->colors_[idx] = Eigen::Vector3d(255,0,0);

        pcds.push_back(cloud_ptr);
    }

    DrawGeometries(pcds);



    return 0;
}
