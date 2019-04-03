#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>

#include <Open3D/Open3D.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API


struct Intrinsic {
    int width, height;
    rs2_intrinsics intrinsics;
};


unsigned long get_timestamp();

cv::Mat     frame_to_mat(const rs2::frame& f);
float       get_depth_scale(rs2::device dev);
open3d::camera::PinholeCameraIntrinsic   get_intrinsics(rs2::pipeline_profile); 
Eigen::Matrix4d imu_extrinsic(rs2::pipeline_profile);
void VisualizeRegistration(const open3d::geometry::PointCloud &source, 
                            const open3d::geometry::PointCloud &target, 
                            const Eigen::Matrix4d &Transformation);

rs2::sensor get_rgb_sensor(rs2::device dev);
rs2::sensor get_stereo_sensor(rs2::device dev);
rs2::sensor get_motion_sensor(rs2::device dev);

rs2::device get_first_device(rs2::device dev);

bool set_white_balance(float val);
bool set_exposure(float val);

bool set_roi(int xmax, int xmin, int ymax, int ymin);
bool set_high_accuracy();
bool set_depth_units(float val);

void WriteLossesToLog(std::ofstream &fout, int frame_idx, std::vector<std::vector<float>> &losses);

float invSqrt(float x);
