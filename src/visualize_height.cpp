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

    auto cloud_ptr = std::make_shared<PointCloud>();
    if (ReadPointCloud(argv[1], *cloud_ptr)) {
        PrintWarning("Successfully read %s\n", argv[2]);
    } else {
        PrintError("Failed to read %s\n\n", argv[2]);
        return 1;
    }

    cloud_ptr = VoxelDownSample(*cloud_ptr, 0.05);

    cloud_ptr->NormalizeNormals();
    cloud_ptr->colors_.resize(cloud_ptr->points_.size());


    int idx = 2;
    auto max = cloud_ptr->GetMaxBound()[idx]; 
    auto min = cloud_ptr->GetMinBound()[idx]; 

    auto delta = max - min;
    
    std::cout << "max: " << max << " min: "<< min<<std::endl;

    SetGlobalColorMap(ColorMap::ColorMapOption::Jet);

    auto color_map_ptr = GetGlobalColorMap();
    for (int i; i<cloud_ptr->points_.size(); i++) {
        double val = cloud_ptr->points_[i][idx];
        double col_idx = (max - val)/delta;
        auto color = color_map_ptr->GetColor(col_idx);

        cloud_ptr->colors_[i] =  color;
    }

    DrawGeometries({cloud_ptr});

    PrintInfo("End of the test.\n");

    return 0;
}
