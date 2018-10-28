#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


cv::Mat frame_to_mat(const rs2::frame& f);

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
