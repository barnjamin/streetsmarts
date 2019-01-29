#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
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
Eigen::Matrix4d imu_extrinsic(rs2::pipeline_profile);
void VisualizeRegistration(const open3d::PointCloud &source, const open3d::PointCloud &target, const Eigen::Matrix4d &Transformation);

void WriteLossesToLog(std::ofstream &fout, int frame_idx, std::vector<std::vector<float>> &losses);

float invSqrt(float x);
