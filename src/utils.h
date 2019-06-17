#pragma once

#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <Open3D/Open3D.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "config.h"


struct Intrinsic {
    int width, height;
    rs2_intrinsics intrinsics;
};


unsigned long get_timestamp();

cv::Mat     frame_to_mat(const rs2::frame& f);
float       get_depth_scale(rs2::device dev);

open3d::camera::PinholeCameraIntrinsic   get_open3d_intrinsic(rs2::pipeline_profile); 
rs2_intrinsics get_depth_intrinsic(rs2::pipeline_profile); 
rs2_intrinsics get_color_intrinsic(rs2::pipeline_profile); 


Eigen::Matrix4d imu_extrinsic(rs2::pipeline_profile);
void VisualizeRegistration(const open3d::geometry::PointCloud &source, 
                            const open3d::geometry::PointCloud &target, 
                            const Eigen::Matrix4d &Transformation);

rs2::sensor get_rgb_sensor(rs2::device dev);
rs2::sensor get_stereo_sensor(rs2::device dev);
rs2::sensor get_motion_sensor(rs2::device dev);

rs2::device get_first_device();

bool set_rgb_whitebalance(bool on);
bool set_rgb_autoexposure(bool on);

bool set_stereo_whitebalance(bool on);
bool set_stereo_autoexposure(bool on);

bool set_laser_power(int power);

void PrintStatus(std::string kind, int state, int total);

bool set_roi(int xmax, int xmin, int ymax, int ymin);
bool set_depth_mode(std::string depth_mode);
bool set_depth_units(float val);

bool set_rgb_gamma(int val);
bool set_rgb_gain(int val);
bool set_rgb_saturation(int val);
bool set_rgb_sharpness(int val);

std::tuple<open3d::registration::PoseGraph, Eigen::Matrix4d> 
InitPoseGraphFromOverlap(const open3d::registration::PoseGraph &pg, int overlap);

void WriteLossesToLog(std::ofstream &fout, int frame_idx, std::vector<std::vector<float>> &losses);

Eigen::Matrix4d Flatten(open3d::geometry::TriangleMesh & pc);
Eigen::Matrix4d Flatten(open3d::geometry::PointCloud & pcd);
std::shared_ptr<open3d::geometry::LineSet> LineSetFromBBox(Eigen::Vector3d min, Eigen::Vector3d max);

void MaskRoad(Config conf, open3d::geometry::Image &depth, int frame_idx);
void MaskHorizon(open3d::geometry::Image &depth);
cv::Mat GetMatrixFromIntrinsic(open3d::camera::PinholeCameraIntrinsic intrinsic);

float invSqrt(float x);
