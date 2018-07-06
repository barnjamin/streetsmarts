#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <pcl/io/vtk_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

//Pasthrough Filter
const float MIN_Z       = 0.0f;
const float MAX_Z       = 5.0f;

//Voxelization
const float VOXEL_SIZE  = 0.05f;
const int NEIGHBORS     = 30;

//Meshing
const float GP3_MU      = 2.5f;

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


int
main (int argc, char** argv)
{

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;
  
  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  // Start streaming with default recommended configuration
  pipe.start();

  // Wait for the next set of frames from the camera
  auto frames = pipe.wait_for_frames();
  auto depth = frames.get_depth_frame();

  auto cloud = points_to_pcl(pc.calculate(depth));

  pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_ptr cloud_vox_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(MIN_Z, MAX_Z);
  pass.filter(*cloud_filtered);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setLeafSize (VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
  sor.filter (*cloud_vox_filtered);

  // Normal estimation
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_vox_filtered);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  n.setInputCloud (cloud_vox_filtered);
  n.setSearchMethod (tree);
  n.setKSearch (NEIGHBORS);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_vox_filtered, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  std::cout << "And also here" << std::endl;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (VOXEL_SIZE*2);

  // Set typical values for the parameters
  gp3.setMu (GP3_MU);
  gp3.setMaximumNearestNeighbors (NEIGHBORS);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  std::cout << "And also here man" << std::endl;
  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();


  pcl::io::saveVTKFile ("mesh.vtk", triangles);

  // Finish
  return (0);
}