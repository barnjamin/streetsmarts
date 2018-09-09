#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>


#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <pcl/gpu/features/features.hpp>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "utils.h" 

Config conf;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

void loadData (std::vector<PCD, Eigen::aligned_allocator<PCD> > &models) {
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
  for(int i=1; i<conf.framestart; i++){
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

      PCD m;
      m.cloud = points_to_pcl(pc.calculate(depth));
      m.f_name = "frame_"+std::to_string(i);

      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
  
      models.push_back (m);
  }
}

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{

  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (conf.icp_leaf, conf.icp_leaf, conf.icp_leaf);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  } else {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  reg.setMaxCorrespondenceDistance (conf.icp_dist);  
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < conf.icp_iters; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
  }

  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;
}
  

int main (int argc, char** argv) {

  conf.parseArgs(argc, argv);

  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (data);

  PointCloud::Ptr result (new PointCloud), source, target;

  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  for (size_t i = 1; i < data.size (); ++i) {

    source = data[i-1].cloud;
    target = data[i].cloud;

    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());

    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

  }

  std::stringstream ss;
  ss << "final" << ".pcd";
  pcl::io::savePCDFile (ss.str (), *result, true);
}
