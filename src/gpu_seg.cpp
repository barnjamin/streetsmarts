#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

// The GPU specific stuff here
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <pcl/gpu/features/features.hpp>

#include <time.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "utils.h" 

int 
main (int argc, char** argv)
{

  // Load cloud in blob format
  auto bagfile = "/media/ssd/20180819_091914.bag";

  rs2::decimation_filter dec_filter;
  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3.0);  

  rs2::config cfg;    
  cfg.enable_device_from_file(bagfile);

  rs2::pointcloud pc;
  rs2::pipeline pipe;
  pipe.start(cfg);

  auto frames = pipe.wait_for_frames();
  auto depth = frames.get_depth_frame();
  for(int i=0; i<10; i++){
      frames = pipe.wait_for_frames();
  }

  frames = pipe.wait_for_frames();

  depth = frames.get_depth_frame();
  depth = dec_filter.process(depth);

  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_filtered = points_to_pcl(pc.calculate(depth));

  pcl::PointCloud<PointNormal>::Ptr normals_small(new pcl::PointCloud<PointNormal>);

  pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setInputCloud(cloud_filtered);
  ne.setRadiusSearch(0.02);
  ne.compute(*normals_small);


  pcl::PointCloud<PointNormal>::Ptr normals_large(new pcl::PointCloud<PointNormal>);
  ne.setRadiusSearch (0.18);
  ne.compute (*normals_large);


  // Create output cloud for DoN results
  PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
  copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);

  cout << "Calculating DoN... " << endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<PointNormal> ("/media/ssd/dons/don.pcd", *doncloud, false); 

  //pcl::PCDWriter writer;

  //std::cout << "INFO: starting with the GPU version" << std::endl;

  //clock_t tStart = clock();

  //pcl::gpu::Octree::PointCloud cloud_device;
  //cloud_device.upload(cloud_filtered->points);
  //
  //pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
  //octree_device->setCloud(cloud_device);
  //octree_device->build();

  //std::vector<pcl::PointIndices> cluster_indices_gpu;
  //pcl::gpu::EuclideanClusterExtraction gec;
  //gec.setClusterTolerance (0.02); // 2cm
  //gec.setMinClusterSize (100);
  //gec.setMaxClusterSize (25000);
  //gec.setSearchMethod (octree_device);
  //gec.setHostCloud(cloud_filtered);
  //gec.extract (cluster_indices_gpu);

  //printf("GPU Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
  //std::cout << "INFO: stopped with the GPU version" << std::endl;

  //int j = 0;
  //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
  //{
  //  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);
  //  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  //    cloud_cluster_gpu->points.push_back (cloud_filtered->points[*pit]); //*
  //  cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
  //  cloud_cluster_gpu->height = 1;
  //  cloud_cluster_gpu->is_dense = true;

  //  std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
  //  std::stringstream ss;
  //  ss << "gpu_cloud_cluster_" << j << ".pcd";
  //  writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_gpu, false); //*
  //  j++;
  //}

  return (0);
}

