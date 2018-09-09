#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>


#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "utils.h"

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = sp.width()*sp.height();
    cloud->height = 1;
    cloud->is_dense = false;

    auto ptr = points.get_vertices();
    int inv = 0;
    for (int x=0; x<points.size(); x++) {
        if(ptr->z == 0){
            inv++;
        }else{
            pcl::PointXYZ p;
            p.x = ptr->x; p.y = ptr->y; p.z = ptr->z;
            cloud->points.push_back(p);
        }
        ptr++;
    }

    cloud->width -= inv;

    pcl_ptr cloud_rot(new pcl::PointCloud<pcl::PointXYZ>);
    std::cout <<"Rotating...";
    //Rotate
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(*cloud, *cloud_rot, transform);
    std::cout <<"done"<<std::endl;

    
    std::cout << "Filtering...";
    pcl_ptr cloud_z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1, 20.0);
    pass.setInputCloud (cloud_rot);
    pass.filter (*cloud_z);

    pcl_ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-5.0, 5.0);
    pass.setInputCloud (cloud_z);
    pass.filter (*cloud_x);

    pcl_ptr cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-5.0, 5.0);
    pass.setInputCloud (cloud_x);
    pass.filter (*cloud_y);


    std::cout << "Done" <<std::endl;
    return cloud_y;
}

Config::~Config() { }

Config::Config() {
    min_z       = 0.0f;
    max_z       = 5.0f;

    frames      = 180;
    framestart  = 180;

    dec_mag     = 1.0;
    spat_mag    = 1.0;
    spat_a      = 0.5;
    spat_d      = 25;
    temp_a      = 0.5;
    temp_d      = 50;


    don_small			= 0.05;
    don_large			= 0.25;
    threshold			= 0.1 ;
    segradius			= 0.02;  // threshold for radius segmentation

    icp_iters           = 30;
    icp_dist            = 1.0;
    icp_leaf            = 0.15;

}

void Config::parseArgs(int argc, char **argv) {
  for(int x=1; x<argc; x+=2){
    std::string flag (argv[x]);
    if(flag == "--minz"){
       min_z = std::stof(argv[x+1]);
    } else if(flag == "--maxz"){
       max_z = std::stof(argv[x+1]);
    }else if(flag ==  "--frames"){
      frames = std::stoi(argv[x+1]);
    }else if(flag ==  "--fstart"){
      framestart = std::stoi(argv[x+1]);
    }else if(flag ==  "--dec-mag"){
      dec_mag = std::stoi(argv[x+1]);
    }else if(flag ==  "--spat-mag"){
      spat_mag = std::stoi(argv[x+1]);
    }else if(flag ==  "--spat-a"){
      spat_a = std::stof(argv[x+1]);
    }else if(flag ==  "--spat-d"){
      spat_d = std::stoi(argv[x+1]);
    }else if(flag ==  "--temp-a"){
      temp_a = std::stof(argv[x+1]);
    }else if(flag ==  "--temp-d"){
      temp_d = std::stoi(argv[x+1]);
    }else if(flag == "--don_small"){
      don_small = std::stof(argv[x+1]);
    }else if(flag == "--don_large"){
      don_large = std::stof(argv[x+1]);
    }else if(flag == "--don_thresh"){
      threshold = std::stof(argv[x+1]);
    }else if(flag == "--don_rad"){
      segradius = std::stof(argv[x+1]);
    }else if(flag == "--icp_iters"){
      icp_iters = std::stoi(argv[x+1]);
    }else if(flag == "--icp_dist"){
      icp_dist = std::stof(argv[x+1]);
    }else if(flag == "--icp_leaf"){
      icp_leaf = std::stof(argv[x+1]);
    }
  }
}
