// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "/home/nvidia/librealsense/examples/example.hpp" // Include short list of convenience functions for rendering
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/organized.h>

#include <pcl/gpu/segmentation/gpu_extract_clusters.h>

#include <pcl/visualization/pcl_visualizer.h>


using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

int main(int argc, char * argv[]) try
{

	auto bagfile = "/media/ssd/20180819_091914.bag";

	rs2::config cfg;    
	cfg.enable_device_from_file(bagfile);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start(cfg);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0,0,0,0,-1,0);

    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3.0);

    while (!viewer->wasStopped ()) {

      // Wait for the next set of frames from the camera
      auto frames = pipe.wait_for_frames();
  
      auto depth = frames.get_depth_frame();
  	  // Generate the pointcloud and texture mappings
      points = pc.calculate(depth);
  
      auto pcl_points = points_to_pcl(points);
  
      pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl_ptr cloud_vox_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud(pcl_points);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(1.0, 10.0);
      pass.filter(*cloud_filtered);
  
	  pcl::VoxelGrid<pcl::PointXYZ> sor;
	  sor.setInputCloud (cloud_filtered);
	  sor.setLeafSize (0.1f, 0.1f, 0.1f);
	  sor.filter (*cloud_vox_filtered);

	  //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	  //pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      //ne.setInputCloud (cloud_vox_filtered);
	  //ne.setSearchMethod (tree);
	  //ne.setKSearch (10);
	  //ne.compute (*cloud_normals);

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
      ne.setMaxDepthChangeFactor(0.02f);
      ne.setNormalSmoothingSize(10.0f);
      ne.setInputCloud(cloud_vox_filtered);
      ne.compute(*cloud_normals);

      viewer->removeAllPointClouds();
      viewer->addPointCloud<pcl::PointXYZ> (cloud_vox_filtered, "sample cloud");
      //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_vox_filtered, cloud_normals, 5, 0.15, "normals");

      viewer->spinOnce (10);
      boost::this_thread::sleep (boost::posix_time::microseconds (1000));

    }

    return EXIT_SUCCESS;
} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
