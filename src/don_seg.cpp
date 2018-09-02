#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "utils.h" 


using namespace pcl;
using namespace std;

int
main (int argc, char *argv[])
{


  Config conf;
  conf.parseArgs(argc, argv);

  ///The smallest scale to use in the DoN filter.
  double scale1 = conf.don_small;

  ///The largest scale to use in the DoN filter.
  double scale2 = conf.don_large;

  ///The minimum DoN magnitude to threshold by
  double threshold = conf.threshold;

  ///segment scene into clusters with given distance tolerance using euclidean clustering
  double segradius = conf.segradius;


  // Load cloud in blob format
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

  
  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start(cfg);

  auto frames = pipe.wait_for_frames();
  for(int i=0; i<conf.framestart; i++){
      frames = pipe.wait_for_frames();
  }

  auto depth = frames.get_depth_frame();
  depth = dec_filter.process(depth);
  depth = depth_to_disparity.process(depth);
  depth = spat_filter.process(depth);
  depth = temp_filter.process(depth);
  depth = disparity_to_depth.process(depth);

  auto cloud = points_to_pcl(pc.calculate(depth));

  // Create a search tree, use KDTreee for non-organized data.
  pcl::search::Search<PointXYZ>::Ptr tree;
  if (cloud->isOrganized ()) {
    tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
  } else {
    tree.reset (new pcl::search::KdTree<PointXYZ> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1 >= scale2)
  {
    cerr << "Error: Large scale must be > small scale!" << endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  cout << "Calculating normals for scale..." << scale1 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale1);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  cout << "Calculating normals for scale..." << scale2 << endl;
  pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

  ne.setRadiusSearch (scale2);
  ne.compute (*normals_large_scale);

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

  writer.write<PointNormal> ("/media/ssd/dons/don_filtered.pcd", *doncloud, false); 

  // Filter by magnitude
  cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

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
    ss << "/media/ssd/dons/don_cluster_" << j << ".pcd";
    writer.write<PointNormal> (ss.str (), *cloud_cluster_don, false);
  }
}
