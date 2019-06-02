#pragma once

#include <fstream>
#include <iostream>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h> // Include RealSense Cross Platform API
#include <json/json.h>
#include <Open3D/Open3D.h>

class Config : public open3d::utility::IJsonConvertible {
public:

    float exposure;
    float white_balance;

    int width;
    int height;
    int fps;
    int framestart;
    int frames;
    
    int depth_mult;

    int img_idx;

    bool capture_gps;
    bool capture_imu;
    bool make_fragments;

    double sigma;

    int overlap_factor;
    int rgbd_lookback;

    int frames_per_fragment;
    int fragments;

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

    double don_downsample;

    double don_small;
    double don_large;

    double threshold_min;
    double threshold_max;

    double cluster_radius;   
    int cluster_min;   
    int cluster_max;   


    bool use_imu;
    bool use_filter;
    bool write_losses;

    std::string session_path;
    std::string session_prefix;

    double max_depth_diff;
    double loop_close_odom;
    double loop_close_reg;
    double voxel_size;
    double final_max_depth;

    int registration_window_size;


    bool stereo_autoexposure; 
    bool stereo_whitebalance; 

    bool rgb_autoexposure; 
    bool rgb_whitebalance; 

    int rgb_gamma;
    int rgb_saturation;
    int rgb_gain;
    int rgb_sharpness;

    rs2_stream aligner;
    rs2::threshold_filter threshold;

    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spat_filter;
    rs2::temporal_filter temp_filter;

    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;

    Config(){}
    Config(int argc, char ** argv);

    void CreateLocalSession();

    std::string GPSFile();
    std::string RoadSegmentFile();
    std::string IMUFile();
    std::string IntrinsicFile();
    std::string ImageTimestampFile();

    void SetExposure();
    void LogStatus(std::string kind, int state, int total);
    std::shared_ptr<std::ofstream> logfile = nullptr;

    std::string PoseFile(int idx);
    std::string DepthFile(int i_idx);
    std::string MaskFile(int i_idx);
    std::string InfraFile(int i_idx);
    std::string ColorFile(int i_idx);
    std::string FragmentFile(int idx);
    std::string ThumbnailFragmentFile(int idx);

    std::string ScenePointCloudFile();
    std::string SceneMeshFile();
    std::string PoseFileScene();
    std::string PoseFileSceneRectified();

    rs2::frame Filter(rs2::depth_frame depth);

    int GetFragmentIdForFrame(int frame_id);
    std::tuple<int,int> GetFramesFromFragment(int fragment_id);
    int GetFragmentCount();
    int GetOverlapCount();

    float GetInvalidDepth(rs2::depth_frame, rs2_intrinsics&);

    virtual ~Config();    

    bool ConvertToJsonValue(Json::Value &value) const override {};
    bool ConvertFromJsonValue(const Json::Value &value) override;

};
