#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

class Config {
public:


    int width;
    int height;
    int fps;
    int framestart;

    int frames_per_fragment;

    double tsdf_cubic_size;
    double tsdf_truncation;

    //Filter options
    int dec_mag;

    int spat_mag;
    double spat_a;
    double spat_d;

    double temp_a;
    double temp_d;


    double min_depth;
    double max_depth;
    double depth_factor;

    double don_small;
    double don_large;

    double threshold_min;
    double threshold_max;

    double segradius;   // threshold for radius segmentation


    bool use_imu;
    bool use_filter;
    bool write_losses;

    std::string session_path;

    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;


    Config(int argc, char ** argv);

    std::string PoseFile(int idx);
    std::string DepthFile(int f_idx, int i_idx);
    std::string ColorFile(int f_idx, int i_idx);
    std::string FragmentFile(int idx);

    rs2::frame Filter(rs2::depth_frame depth);

    virtual ~Config();    
};
