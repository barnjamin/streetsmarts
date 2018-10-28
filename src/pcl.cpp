// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "/home/nvidia/librealsense/examples/example.hpp" // Include short list of convenience functions for rendering
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include "utils.h"


int main(int argc, char * argv[]) try
{

    Config conf;
    conf.parseArgs(argc, argv);

    pcl::PCDWriter writer;

	auto bagfile = "/media/ssd/20180819_091914.bag";

	rs2::config cfg;    
	cfg.enable_device_from_file(bagfile);

    rs2::points points;

    rs2::decimation_filter dec_filter;
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, conf.dec_mag);  

    rs2::spatial_filter spat_filter;
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, conf.spat_mag);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, conf.spat_a);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, conf.spat_d);

    rs2::temporal_filter temp_filter;
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, conf.temp_a);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, conf.temp_d);

    rs2::disparity_transform depth_to_disparity(true);
    rs2::disparity_transform disparity_to_depth(false);

    rs2::pointcloud pc;
    rs2::pipeline pipe;
    pipe.start(cfg);

    auto frames = pipe.wait_for_frames();
    auto depth = frames.get_depth_frame();
    for(int i=0; i<conf.framestart; i++){
        frames = pipe.wait_for_frames();
    }
    for(int i=0; i<conf.frames; i++){

        frames = pipe.wait_for_frames();

        depth = frames.get_depth_frame();
        depth = frames.get_depth_frame();
        depth = dec_filter.process(depth);
        depth = depth_to_disparity.process(depth);
        depth = spat_filter.process(depth);
        depth = temp_filter.process(depth);
        depth = disparity_to_depth.process(depth);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = points_to_pcl(pc.calculate(depth));

        std::ostringstream ss;
        ss << "/media/ssd/pcls/points_"<<i<<".pcd";
        writer.write<pcl::PointXYZ> (ss.str(), *cloud, false);
    }
  


    return EXIT_SUCCESS;
} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
