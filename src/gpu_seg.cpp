#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/organized.h>
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

using namespace pcl;
using namespace std;
int 
main (int argc, char** argv)
{


  Config conf;
  conf.parseArgs(argc, argv);

  double scale1 = conf.don_small;
  double scale2 = conf.don_large;
  double threshold = conf.threshold;
  double segradius = conf.segradius;

  pcl::PCDWriter writer;

  auto bagfile = "/media/ssd/20180819_091914.bag";

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

  rs2::config cfg;    
  cfg.enable_device_from_file(bagfile);

  rs2::pointcloud pc;
  rs2::pipeline pipe;
  pipe.start(cfg);

  auto frames = pipe.wait_for_frames();
  auto depth = frames.get_depth_frame();
  for(int i=0; i<conf.framestart; i++){
      frames = pipe.wait_for_frames();
  }

  pcl::gpu::Octree::PointCloud cloud_device;
  pcl::gpu::NormalEstimation ne;
  pcl::gpu::Feature::Normals normals_small;
  pcl::gpu::Feature::Normals normals_large;

  pcl::PointCloud<PointNormal>::Ptr small_normals(new pcl::PointCloud<PointNormal>);
  pcl::PointCloud<PointNormal>::Ptr large_normals(new pcl::PointCloud<PointNormal>);

  std::vector<pcl::PointIndices> cluster_indices_gpu;
  pcl::gpu::EuclideanClusterExtraction gec;

  pcl::gpu::Octree::PointCloud doncloud_device;
  pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);

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

      cloud_device.release();
      cloud_device.upload(cloud->points);

      ne.setInputCloud(cloud_device);

      ne.setRadiusSearch(scale1, 100);
      ne.compute(normals_small);

      ne.setRadiusSearch(scale2, 100);
      ne.compute(normals_large);

      PointXYZ normals_small_host[cloud->points.size()];
      normals_small.download(&normals_small_host[0]);

      PointXYZ normals_large_host[cloud->points.size()];
      normals_large.download(&normals_large_host[0]);

      small_normals->clear();
      small_normals->width = cloud->points.size();
      small_normals->height = 1;

      large_normals->clear();
      large_normals->width = cloud->points.size();
      large_normals->height = 1;

      PointXYZ np;
      PointXYZ p;
      for(int x=0; x<cloud->points.size(); x++){
            PointNormal nps;
            PointNormal npl;
            p = cloud->points[x];

            npl.x = p.x; npl.y =p.y; npl.z = p.z;
            nps.x = p.x; nps.y =p.y; nps.z = p.z;

            np = normals_small_host[x];
            nps.normal_x = np.x; nps.normal_y = np.y; nps.normal_z = np.z;
            nps.curvature = np.data[3];
            small_normals->points.push_back(nps);

            np = normals_large_host[x];
            npl.normal_x = np.x; npl.normal_y = np.y; npl.normal_z = np.z;
            npl.curvature = np.data[3];
            large_normals->points.push_back(npl);
      }

      //writer.write<PointNormal> ("normals_small.pcd", *small_normals, false);

      // Create output cloud for DoN results
      PointCloud<PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
      copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);

      cout << "Calculating DoN... " << endl;
      // Create DoN operator
      pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
      don.setInputCloud (cloud);
      don.setNormalScaleLarge (large_normals);
      don.setNormalScaleSmall (small_normals);

      if (!don.initCompute ()) {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit (EXIT_FAILURE);
      }
      don.computeFeature (*doncloud);

      // Filter by magnitude
      cout << "Filtering out DoN mag <= " << threshold << "..." << endl;

      // Build the condition for filtering
      pcl::ConditionOr<PointNormal>::Ptr range_cond ( new pcl::ConditionOr<PointNormal> ());
      range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr ( new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::LT, threshold)));

      // Build the filter
      pcl::ConditionalRemoval<PointNormal> condrem;
      condrem.setCondition(range_cond);
      condrem.setInputCloud (doncloud);

      pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

      // Apply filter
      condrem.filter (*doncloud_filtered);
      doncloud = doncloud_filtered;

      // Save filtered output
      std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

      pcl::search::KdTree<PointNormal>::Ptr segtree (new pcl::search::KdTree<PointNormal>);
      segtree->setInputCloud (doncloud);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<PointNormal> ec;

      ec.setClusterTolerance (segradius);
      ec.setMinClusterSize (50);
      ec.setMaxClusterSize (100000);
      ec.setSearchMethod (segtree);
      ec.setInputCloud (doncloud);
      ec.extract (cluster_indices);

      int j = 0;
      for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, j++)
      {
        pcl::PointCloud<PointNormal>::Ptr cloud_cluster_don (new pcl::PointCloud<PointNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          cloud_cluster_don->points.push_back (doncloud->points[*pit]);
        }

        cloud_cluster_don->width = int (cloud_cluster_don->points.size ());
        cloud_cluster_don->height = 1;
        cloud_cluster_don->is_dense = true;

        //Save cluster
        cout << "PointCloud representing the Cluster: " << cloud_cluster_don->points.size () << " data points." << std::endl;
        stringstream ss;
        ss << "/media/ssd/dons/don_cluster_" << j << "_" << i << ".pcd";
        writer.write<PointNormal> (ss.str (), *cloud_cluster_don, false);
      }

      ////pcl::PointCloud<PointXYZ>::Ptr hc (new pcl::PointCloud<PointXYZ>);
      ////for(int x=0; x<doncloud->points.size(); x++){
      ////  PointXYZ p;
      ////  PointNormal dp = doncloud->points[x];
      ////  p.x = dp.x; p.y=dp.y; p.z=dp.z; 
      ////  hc->points.push_back(p);
      ////}
      ////hc->height = 1;
      ////hc->width = hc->points.size();

      ////doncloud_device.release();
      ////doncloud_device.upload(hc->points);

      ////octree_device->setCloud(doncloud_device);
      ////octree_device->build();

      ////gec.setClusterTolerance (segradius); 
      ////gec.setMinClusterSize (50);
      ////gec.setMaxClusterSize (100000);
      ////gec.setSearchMethod (octree_device);
      ////gec.setHostCloud(hc);
      ////gec.extract (cluster_indices_gpu);

      ////int j = 0;
      ////for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_gpu.begin (); it != cluster_indices_gpu.end (); ++it)
      ////{
      ////  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_gpu (new pcl::PointCloud<pcl::PointXYZ>);
      ////  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      ////    cloud_cluster_gpu->points.push_back (cloud->points[*pit]); //*
      ////  cloud_cluster_gpu->width = cloud_cluster_gpu->points.size ();
      ////  cloud_cluster_gpu->height = 1;
      ////  cloud_cluster_gpu->is_dense = true;

      ////  std::cout << "PointCloud representing the Cluster: " << cloud_cluster_gpu->points.size () << " data points." << std::endl;
      ////  std::stringstream ss;
      ////  ss << "/media/ssd/dons/don_cluster_" << j << "_" << i << ".pcd";
      ////  writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster_gpu, false); //*
      ////  j++;
      ////}

  }


  return (0);
}

