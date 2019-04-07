#include <fstream>
#include <chrono>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "utils.h"
#include <Eigen/Geometry>
#include <Open3D/Visualization/Visualizer/Visualizer.h>


unsigned long get_timestamp() {
    return std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
}

void PrintStatus(std::string kind, int state, int total) {
    std::cout << kind << ":" <<state << ":" << total << std::endl;
}

void VisualizeRegistration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &Transformation) {
    using namespace open3d;
    std::shared_ptr<geometry::PointCloud> source_transformed_ptr(new geometry::PointCloud);
    std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
    *source_transformed_ptr = source;
    *target_ptr = target;
    
    source_transformed_ptr->Transform(Transformation);

    target_ptr->PaintUniformColor(Eigen::Vector3d(0,0,1.0));
    source_transformed_ptr->PaintUniformColor(Eigen::Vector3d(1.0,0,0));

    visualization::DrawGeometries({source_transformed_ptr, target_ptr}, "Registration result");
}

// Convert rs2::frame to cv::Mat
cv::Mat frame_to_mat(const rs2::frame& f)
{
	using namespace cv;
	using namespace rs2;

	auto vf = f.as<video_frame>();
	const int w = vf.get_width();
	const int h = vf.get_height();

	if (f.get_profile().format() == RS2_FORMAT_BGR8)
	{
		return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_RGB8)
	{
		auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
		cvtColor(r, r, CV_RGB2BGR);
		return r;
	}
	else if (f.get_profile().format() == RS2_FORMAT_Z16)
	{
		return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}
	else if (f.get_profile().format() == RS2_FORMAT_Y8)
	{
		return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
	}

	throw std::runtime_error("Frame format is not supported yet!");
}


open3d::camera::PinholeCameraIntrinsic get_open3d_intrinsic(rs2::pipeline_profile prof) {

    auto intrin = get_color_intrinsic(prof);

    return open3d::camera::PinholeCameraIntrinsic(intrin.width, intrin.height,
                intrin.fx, intrin.fy, intrin.ppx, intrin.ppy);
}

rs2_intrinsics get_depth_intrinsic(rs2::pipeline_profile prof)
{
    auto depth_stream = prof.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();

    return depth_stream.get_intrinsics();
}

rs2_intrinsics get_color_intrinsic(rs2::pipeline_profile prof)
{
    auto stream = prof.get_stream(RS2_STREAM_COLOR)
                                 .as<rs2::video_stream_profile>();

    return stream.get_intrinsics();
}

Eigen::Matrix4d imu_extrinsic(rs2::pipeline_profile prof) 
{

    auto depth_stream = prof.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();

    auto motion_stream = prof.get_stream(RS2_STREAM_GYRO);
    auto a = motion_stream.get_extrinsics_to(depth_stream);

    Eigen::Translation3d t(a.translation[0], a.translation[1], a.translation[2]);

    Eigen::Matrix3d mat;
    mat << a.rotation[0], a.rotation[1], a.rotation[2],
            a.rotation[3], a.rotation[4], a.rotation[5],
            a.rotation[6], a.rotation[7], a.rotation[8];

    Eigen::Quaterniond q(mat);

    Eigen::Transform<double, 3, Eigen::Affine> ext = t * q.normalized().toRotationMatrix();

    return ext.matrix();
}

rs2::sensor get_rgb_sensor(rs2::device dev) {
    rs2::sensor s;
    for(auto sensor : dev.query_sensors()){
        if (!sensor.supports(RS2_CAMERA_INFO_NAME)) continue ;

        if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)).compare("RGB Camera")) {
            return sensor; 
        }
    }
    return s;
}

rs2::sensor get_stereo_sensor(rs2::device dev) {
    rs2::sensor s;
    for(auto sensor : dev.query_sensors()){
        if (!sensor.supports(RS2_CAMERA_INFO_NAME)) continue;

        if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)).compare("Stereo Module")) {
            return sensor; 
        }
    }
    return s;
}

rs2::sensor get_motion_sensor(rs2::device dev) {
    rs2::sensor s;
    for(auto sensor : dev.query_sensors()){
        if (!sensor.supports(RS2_CAMERA_INFO_NAME)) continue;

        if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)).compare("Motion Module")) {
            return sensor; 
        }
    }
    return s;
}

rs2::device get_first_device(){
        rs2::context ctx;
        return ctx.query_devices().front();
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

bool set_white_balance(float val){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_WHITE_BALANCE)){
        cam.set_option(RS2_OPTION_WHITE_BALANCE, val);
        return true;
    }

    return false;
}

bool set_exposure(float val){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_EXPOSURE)){
        cam.set_option(RS2_OPTION_EXPOSURE, val);
    }
    return false;
}

bool set_roi(int xmax, int xmin, int ymax, int ymin){
    auto dev = get_first_device();

    rs2::region_of_interest roi;
    roi.max_x = xmax;
    roi.min_x = xmin;
    roi.min_y = ymin;
    roi.max_y = ymax;

    auto cam = get_stereo_sensor(dev);
    if(cam.is<rs2::roi_sensor>()){
        cam.as<rs2::roi_sensor>().set_region_of_interest(roi);
        return true;
    }

    return false;
}

bool set_depth_units(float val){
    auto dev = get_first_device();
    auto cam = get_stereo_sensor(dev);

    if(cam.supports(RS2_OPTION_DEPTH_UNITS)){
        cam.set_option(RS2_OPTION_DEPTH_UNITS, val);
        return true;
    }
    return false;
}

bool set_high_accuracy(){
    auto dev = get_first_device();
    auto cam = get_stereo_sensor(dev);

    if(cam.supports(RS2_OPTION_VISUAL_PRESET)){
        cam.set_option(RS2_OPTION_VISUAL_PRESET, 
                RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        return true;
    }

    return false;
}


void WriteLossesToLog(std::ofstream &fout, int frame_idx, std::vector<std::vector<float>> &losses) 
{
    assert(fout.is_open());

    fout << frame_idx << "\n";
    for (auto &losses_on_level : losses) {
        for (auto &loss : losses_on_level) {
            fout << loss << " ";
        }
        fout << "\n";
    }
}

// Fast inverse square-root See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
