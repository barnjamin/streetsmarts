#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/common/transforms.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


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

    pcl_ptr cloud_rot(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout <<"Rotating...";
    //Rotate
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud_rot, transform);

    std::cout <<"done"<<std::endl;

    return cloud_rot;
}



int
main (int argc, char** argv)
{

  auto bagfile = "/media/ssd/20180819_091914.bag";

  rs2::config cfg;    
  cfg.enable_device_from_file(bagfile);
  
  rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3.0);

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start(cfg);

  
  std::cout <<"starting"<<std::endl;
  // Wait for the next set of frames from the camera

  auto frames = pipe.wait_for_frames();
  for(int i=0; i<240; i++){
    frames = pipe.wait_for_frames();
  }

  auto depth = frames.get_depth_frame();
  depth = dec_filter.process(depth);
  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth);
  
  std::cout <<"got pc"<<std::endl;

  auto cloud  = points_to_pcl(points);

  std::cout << "Building normal estimator" << std::endl;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.1f);
  ne.setNormalSmoothingSize(1.0f);
  ne.setInputCloud(cloud);
  ne.compute (*normals);


  std::cout <<"built normal estimator"<<std::endl;

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 5.0);
  pass.filter (*indices);

  std::cout <<"Filtered stuff"<<std::endl;

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::cout <<"Created region grower"<<std::endl;

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout <<"extracted"<<std::endl;

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;

  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
