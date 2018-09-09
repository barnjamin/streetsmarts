#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

pcl_ptr points_to_pcl(const rs2::points& points);

pcl::PointCloud<pcl::PointXYZ> p2pcl(const rs2::points& points);


class Config {
public:
    float min_z;
    float max_z;

    int frames;
    int framestart;

    int dec_mag;

    int spat_mag;
    float spat_a;
    float spat_d;

    float temp_a;
    float temp_d;


    double don_small;
    double don_large;
    double threshold;
    double segradius;   // threshold for radius segmentation

    int icp_iters;
    float icp_dist;
    float icp_leaf;


    Config();

    void parseArgs(int argc, char **argv);

    virtual ~Config();    
};
