#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <pcl/io/vtk_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "utils.h" 

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

//Voxelization
float VOXEL_SIZE  = 0.05f;
int NEIGHBORS     = 30;

//Meshing
float GP3_MU      = 2.5f;

int
main (int argc, char** argv)
{

  Config conf;
  conf.parseArgs(argc, argv);

  for(int x=1; x<argc; x+=2){
    std::string flag (argv[x]);
     if (flag ==  "--voxel-size") {
       VOXEL_SIZE = std::stof(argv[x+1]);
     }else if(flag ==  "--neighbors"){
       NEIGHBORS = std::stoi(argv[x+1]);
     }else if(flag ==  "--mu"){
       GP3_MU = std::stof(argv[x+1]);
     }
  }


  //auto bagfile = "/media/ssd/20180819_091914.bag";
  auto bagfile = "/media/ssd/2018-10-09-15-01-09.bag";


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

  
  std::cout <<"starting"<<std::endl;

  // Initialize objects
  pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ptr cloud_vox_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::VoxelGrid<pcl::PointXYZ> sor;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);
  gp3.setSearchRadius (VOXEL_SIZE*2);
  gp3.setMu (GP3_MU);
  gp3.setMaximumNearestNeighbors (NEIGHBORS);

  pcl::PolygonMesh triangles;

  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);

  // Wait for the next set of frames from the camera
  auto frames = pipe.wait_for_frames();
  auto depth = frames.get_depth_frame();
  for(int i=0; i<conf.framestart; i++){
      pipe.wait_for_frames();
  }

  pcl_ptr cloud; 
  for(int i=0; i<conf.frames; i++) {
      frames = pipe.wait_for_frames();
      depth = frames.get_depth_frame();
      depth = dec_filter.process(depth);
      depth = depth_to_disparity.process(depth);
      depth = spat_filter.process(depth);
      depth = temp_filter.process(depth);
      depth = disparity_to_depth.process(depth);

      cloud = points_to_pcl(pc.calculate(depth));

      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(conf.min_z, conf.max_z);
      pass.filter(*cloud_filtered);

      sor.setInputCloud (cloud_filtered);
      sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
      sor.filter (*cloud_vox_filtered);

      // Normal estimation
      tree->setInputCloud (cloud_vox_filtered);

      n.setInputCloud (cloud_vox_filtered);
      n.setSearchMethod (tree);
      n.setKSearch (NEIGHBORS);
      n.compute (*normals);

      // Concatenate the XYZ and normal fields*
      pcl::concatenateFields (*cloud_vox_filtered, *normals, *cloud_with_normals);

      // Create search tree*
      tree2->setInputCloud (cloud_with_normals);

      // Get result
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);

      // Additional vertex information
      std::vector<int> parts = gp3.getPartIDs();
      std::vector<int> states = gp3.getPointStates();

      pcl::io::saveVTKFile ("/media/ssd/meshes2/mesh_"+std::to_string(i)+".vtk", triangles);
  }

  // Finish
  return (0);
}
