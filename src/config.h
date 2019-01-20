#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class Config {
public:

    float min_z;
    float max_z;

    int fps;
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

    int width;
    int height;

    bool use_imu;
    bool use_filter;

    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;

    Config();
    Config(std::string fname);

    void parseArgs(int argc, char **argv);

    rs2::frame filter(rs2::depth_frame depth);

    void save(std::string filename);

    virtual ~Config();    
};
