#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>

#include <Core/Core.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API



struct Intrinsic {
    int width, height;
    rs2_intrinsics intrinsics;
};


cv::Mat     frame_to_mat(const rs2::frame& f);
float       get_depth_scale(rs2::device dev);
open3d::PinholeCameraIntrinsic   get_intrinsics(rs2::pipeline_profile); 

class Config {
public:

    std::string imu_src;

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

    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;

    Config();

    void parseArgs(int argc, char **argv);
    rs2::frame filter(rs2::depth_frame depth);


    virtual ~Config();    
};
