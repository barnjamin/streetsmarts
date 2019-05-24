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
#include "config.h"


unsigned long get_timestamp() {
    return std::chrono::system_clock::now().time_since_epoch() / std::chrono::milliseconds(1);
}

void PrintStatus(std::string kind, int state, int total) {
    std::cout << get_timestamp() << ":" << kind << ":" << state + 1 << ":" << total << std::endl;
}

cv::Mat GetMatrixFromIntrinsic(open3d::camera::PinholeCameraIntrinsic intrinsic) {
    auto f = intrinsic.GetFocalLength();
    auto p = intrinsic.GetPrincipalPoint();
    cv::Mat K = (cv::Mat1d(3, 3) << f.first, 0, p.first, 0, f.second, p.second, 0, 0, 1);
    return K;
}

void MaskRoad(Config conf, open3d::geometry::Image &depth, int frame_idx) {
    using namespace open3d;
    using namespace open3d::geometry;
    using namespace open3d::io;

    Image mask;
    ReadImage(conf.MaskFile(frame_idx), mask);
    for (int y = 0; y < mask.height_; y++) {
        for (int x = 0; x < mask.width_; x++) {
            uint8_t *p = PointerAt<uint8_t>(mask, x, y);

            //TODO:: this also cuts the top half of the image off
            if(*p != 255 || y>(mask.height_/2)){
                uint16_t *d = PointerAt<uint16_t>(depth, x,y); 
                *d = (uint16_t) 0;
            }
        }
    }

    return;
}

std::shared_ptr<open3d::geometry::LineSet> LineSetFromBBox(Eigen::Vector3d min, Eigen::Vector3d max){
    using namespace open3d;
    using namespace open3d::geometry;

    double maxx = max[0];
    double maxy = max[1];
    double maxz = max[2];

    double minx = min[0];
    double miny = min[1];
    double minz = min[2];

    std::shared_ptr<LineSet> bb = std::make_shared<LineSet>();

    bb->points_.push_back(Eigen::Vector3d(minx,miny,minz));
    bb->points_.push_back(Eigen::Vector3d(maxx,miny,minz));
    bb->points_.push_back(Eigen::Vector3d(minx,maxy,minz));
    bb->points_.push_back(Eigen::Vector3d(maxx,maxy,minz));
    bb->points_.push_back(Eigen::Vector3d(minx,miny,maxz));
    bb->points_.push_back(Eigen::Vector3d(maxx,miny,maxz));
    bb->points_.push_back(Eigen::Vector3d(minx,maxy,maxz));
    bb->points_.push_back(Eigen::Vector3d(maxx,maxy,maxz));
    
    bb->lines_.push_back(Eigen::Vector2i(0,1));
    bb->lines_.push_back(Eigen::Vector2i(0,2));
    bb->lines_.push_back(Eigen::Vector2i(1,3));
    bb->lines_.push_back(Eigen::Vector2i(2,3));
    bb->lines_.push_back(Eigen::Vector2i(4,5));
    bb->lines_.push_back(Eigen::Vector2i(4,6));
    bb->lines_.push_back(Eigen::Vector2i(5,7));
    bb->lines_.push_back(Eigen::Vector2i(6,7));
    bb->lines_.push_back(Eigen::Vector2i(0,4));
    bb->lines_.push_back(Eigen::Vector2i(1,5));
    bb->lines_.push_back(Eigen::Vector2i(2,6));
    bb->lines_.push_back(Eigen::Vector2i(3,7));

    for(int i=0; i<bb->lines_.size(); i++) {
        bb->colors_.push_back(Eigen::Vector3d(255,0,0));
    }

    for(int i=0; i<3; i++){
        double delta = abs(max[i] - min[i]);
        std::cout << "Axis: "<< i << " " << delta << std::endl;
    }


    return bb;
}

Eigen::Matrix4d Flatten(open3d::geometry::PointCloud &pc) {
    using namespace open3d;

    auto pcd = geometry::VoxelDownSample(pc, 0.1);
    geometry::EstimateNormals(*pcd, geometry::KDTreeSearchParamRadius(1.0));

    geometry::OrientNormalsToAlignWithDirection(*pcd, Eigen::Vector3d(0.0, 1.0, 0.0));

    Eigen::Vector3d navg;
    for(int x=0; x<pcd->normals_.size(); ++x){
        if(!pcd->normals_[x].hasNaN()){
            navg += pcd->normals_[x];
        }else{
            std::cout << "\n" << pcd->normals_[x] ;
        }
    }

    navg /= pcd->normals_.size();

    Eigen::Vector3d expected(0.0, 1.0, 0.0);

    Eigen::Matrix3d R(Eigen::Quaterniond().setFromTwoVectors(navg, expected));
    
    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(0,0,0) * R; 

    return t.matrix();
}

Eigen::Matrix4d Flatten(open3d::geometry::TriangleMesh &m) {
    using namespace open3d;

    auto pcd = open3d::geometry::SamplePointsUniformly(m, 1000);

    geometry::EstimateNormals(*pcd, geometry::KDTreeSearchParamRadius(1.0));

    geometry::OrientNormalsToAlignWithDirection(*pcd, Eigen::Vector3d(0.0, 1.0, 0.0));

    Eigen::Vector3d navg;
    for(int x=0; x<pcd->normals_.size(); ++x){
        if(!pcd->normals_[x].hasNaN()){
            navg += pcd->normals_[x];
        }else{
            std::cout << "\n" << pcd->normals_[x] ;
        }
    }

    navg /= pcd->normals_.size();

    Eigen::Vector3d expected(0.0, 1.0, 0.0);

    Eigen::Matrix3d R(Eigen::Quaterniond().setFromTwoVectors(navg, expected));
    
    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(0,0,0) * R; 

    return t.matrix();
}

// Assumes single edge between all nodes
std::tuple<open3d::registration::PoseGraph, Eigen::Matrix4d>
InitPoseGraphFromOverlap(const open3d::registration::PoseGraph &pg, int overlap) 
{
    using namespace open3d;
    using namespace open3d::registration;

    PoseGraph npg;
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    npg.nodes_.emplace_back(PoseGraphNode(trans_odometry));

    int idx = pg.nodes_.size() - overlap - 1;
    for(int i=0; i<overlap; ++i)
    {
        PoseGraphEdge prev_edge = pg.edges_[idx + i];

        Eigen::Matrix4d trans = prev_edge.transformation_;
        Eigen::Matrix6d information = prev_edge.information_;

        trans_odometry = trans * trans_odometry;

        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();

        npg.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
        npg.edges_.emplace_back(PoseGraphEdge( 
                    i, i+1, trans, information, false));
    }

    return std::make_tuple(npg, trans_odometry);
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
		cvtColor(r, r, cv::COLOR_RGB2BGR);
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
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) 
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }


    std::cout << "format "<< f.get_profile().format() << std::endl;

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

        if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)).compare("RGB Camera") == 0) {
            return sensor; 
        }
    }
    return s;
}

rs2::sensor get_stereo_sensor(rs2::device dev) {
    rs2::sensor s;
    for(auto sensor : dev.query_sensors()){
        if (!sensor.supports(RS2_CAMERA_INFO_NAME)) continue;

        if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)).compare("Stereo Module") == 0) {
            return sensor; 
        }
    }
    return s;
}

rs2::sensor get_motion_sensor(rs2::device dev) {
    rs2::sensor s;
    for(auto sensor : dev.query_sensors()){
        if (!sensor.supports(RS2_CAMERA_INFO_NAME)) continue;

        if (std::string(sensor.get_info(RS2_CAMERA_INFO_NAME)).compare("Motion Module") == 0) {
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

bool set_rgb_saturation(int val){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_SATURATION)){
        cam.set_option(RS2_OPTION_SATURATION, val);
        return true;
    }
    return false;
}

bool set_rgb_gain(int val){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_GAIN)){
        cam.set_option(RS2_OPTION_GAIN, val);
        return true;
    }
    return false;
}

bool set_rgb_gamma(int val){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_GAMMA)){
        cam.set_option(RS2_OPTION_GAMMA, val);
        return true;
    }
    return false;
}

bool set_rgb_sharpness(int val){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_SHARPNESS)){
        cam.set_option(RS2_OPTION_SHARPNESS, val);
        return true;
    }
    return false;
}

bool set_stereo_whitebalance(bool on){
    auto dev = get_first_device();
    auto cam = get_stereo_sensor(dev);

    if(cam.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)){
        int setting = on?1:0;
        cam.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, setting);
        return true;
    }

    return false;
}

bool set_stereo_autoexposure(bool on){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
        int setting = on?1:0;
        cam.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, setting);
    }
    return false;
}

bool set_rgb_whitebalance(bool on){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)){
        int setting = on?1:0;
        cam.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, setting);
        return true;
    }

    return false;
}

bool set_rgb_autoexposure(bool on){
    auto dev = get_first_device();
    auto cam = get_rgb_sensor(dev);

    if(cam.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)){
        int setting = on?1:0;
        cam.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, setting);
    }
    return false;
}

bool set_roi(int xmin, int xmax, int ymin, int ymax){
    auto dev = get_first_device();
    auto cam = get_stereo_sensor(dev);

    rs2::region_of_interest roi;
    roi.max_x = xmax;
    roi.min_x = xmin;
    roi.min_y = ymin;
    roi.max_y = ymax;

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
        cam.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
        return true;
    }

    return false;
}

bool set_max_laser_power(){
    auto dev = get_first_device();
    auto cam = get_stereo_sensor(dev);

    if(cam.supports(RS2_OPTION_LASER_POWER)){
        cam.set_option(RS2_OPTION_LASER_POWER, 360);
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
