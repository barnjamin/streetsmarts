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

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

//Pasthrough Filter
float MIN_Z       = 0.0f;
float MAX_Z       = 5.0f;

//Voxelization
float VOXEL_SIZE  = 0.05f;
int NEIGHBORS     = 30;

//Meshing
float GP3_MU      = 2.5f;

int FRAMES = 180;
int FRAMESTART = 180;

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


  for(int x=1; x<argc; x+=2){
    std::string flag (argv[x]);
    if(flag == "--minz"){
       //std::cout<<argv[x]<<std::endl;
       MIN_Z = std::stof(argv[x+1]);
    } else if(flag == "--maxz"){
       //std::cout<<argv[x]<<std::endl;
       MAX_Z = std::stof(argv[x+1]);
    } else if (flag ==  "--voxel-size") {
       //std::cout<<argv[x]<<std::endl;
       VOXEL_SIZE = std::stof(argv[x+1]);
     }else if(flag ==  "--neighbors"){
       //std::cout<<argv[x]<<std::endl;
       NEIGHBORS = std::stoi(argv[x+1]);
     }else if(flag ==  "--mu"){
       //std::cout<<argv[x]<<std::endl;
       GP3_MU = std::stof(argv[x+1]);
     }else if(flag ==  "--frames"){
       //std::cout<<argv[x]<<std::endl;
       FRAMES = std::stoi(argv[x+1]);
     }else if(flag ==  "--fstart"){
       //std::cout<<argv[x]<<std::endl;
       FRAMESTART = std::stoi(argv[x+1]);
     }
  }


  auto bagfile = "/media/ssd/20180819_091914.bag";

  rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 4.0);  // Decimation - reduces depth frame density
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
  auto cloud = points_to_pcl(pc.calculate(depth));
  for(int i=FRAMESTART; i<FRAMESTART+FRAMES; i++){
      frames = pipe.wait_for_frames();
      depth = frames.get_depth_frame();
      depth = dec_filter.process(depth);

      cloud = points_to_pcl(pc.calculate(depth));

      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(MIN_Z, MAX_Z);
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

      std::cout << "And also here" << std::endl;

      // Get result
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);

      std::cout << "And also here man" << std::endl;

      // Additional vertex information
      std::vector<int> parts = gp3.getPartIDs();
      std::vector<int> states = gp3.getPointStates();

      pcl::io::saveVTKFile ("/media/ssd/meshes/mesh_"+std::to_string(i)+".vtk", triangles);
  }

  // Finish
  return (0);
}
