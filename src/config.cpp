#include <iostream>
#include <iomanip>
#include <unistd.h>

#include <json/json.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <Open3D/Open3D.h>


#include "config.h"
#include "utils.h"

std::string generate_local_session(std::string prefix) {
    using namespace open3d;

    std::stringstream ss;
    std::time_t result = std::time(0);
    ss << "/home/ben/local-sessions/"<< prefix << "-" <<  result;

    std::string session_path = ss.str(); 

    if( !utility::filesystem::MakeDirectoryHierarchy(session_path + "/pose") ||
        !utility::filesystem::MakeDirectoryHierarchy(session_path + "/scene") ||
        !utility::filesystem::MakeDirectoryHierarchy(session_path + "/color") ||
        !utility::filesystem::MakeDirectoryHierarchy(session_path + "/depth") ||
        !utility::filesystem::MakeDirectoryHierarchy(session_path + "/infra") ||
        !utility::filesystem::MakeDirectoryHierarchy(session_path + "/thumbnail_fragment") ||
        !utility::filesystem::MakeDirectoryHierarchy(session_path + "/fragment")){
        std::cout << "failed to create files" << std::endl; 
    }

    unlink("/home/ben/local-sessions/latest");
    if(symlink(session_path.c_str(), "/home/ben/local-sessions/latest")==-1){
        std::cout << "failed to symlink" << std::endl; 
        exit(1);
    }

    return session_path; 
}

Config::~Config() { }

void Config::SetExposure(){
    set_stereo_autoexposure(stereo_autoexposure);
    set_stereo_whitebalance(stereo_whitebalance);

    set_rgb_autoexposure(rgb_autoexposure);
    set_rgb_whitebalance(rgb_whitebalance);
}

void Config::LogStatus(std::string kind, int state, int total) {
    std::stringstream out;
    out << get_timestamp() << ":" << kind << ":" << state << ":" << total-1;

    if(logfile == NULL || ! logfile->is_open()){
        logfile = std::make_shared<std::ofstream>(session_path+"/status.log",  
                std::fstream::in | std::fstream::out | std::fstream::app);
    }

    std::cout<< out.str() << std::endl;

    //write to logfile
    *logfile <<  out.str() << std::endl;
}

float Config::GetInvalidDepth(rs2::depth_frame depth_frame, rs2_intrinsics& intrinsics) {
    rs2::disparity_frame disp_frame = depth_to_disparity.process(depth_frame);
    float baseline = disp_frame.get_baseline();
    
    //TODO, why?
    float distance = 2000; // mm

    std::array<float, 2> fov;
    rs2_fov( &intrinsics, &fov[0] );
    float horizontal_fov = fov[0] * M_PI / 180.0; // radian

    // DBR
    float distance_band_ratio = baseline / std::floor( 2.0 * distance * std::tan( horizontal_fov / 2.0 ) );

    // IDB
    uint32_t depth_width = depth_frame.get_width();
    float invalid_depth_band = depth_width * distance_band_ratio;

    return invalid_depth_band * 2;
}

Config::Config(int argc, char ** argv) {
    using namespace open3d;

    // Where to write the files
    session_path = utility::GetProgramOptionAsString(argc, argv, "--session");
    session_prefix = utility::GetProgramOptionAsString(argc, argv, "--session_prefix", "abc123");

    if (session_path == "latest"){
        session_path =  "/home/ben/local-sessions/latest";
    }


    depth_mult = utility::GetProgramOptionAsInt(argc, argv, "--depth_mult", 1);

    if(utility::ProgramOptionExists(argc, argv, "--set_cam_opts")){

        stereo_autoexposure = utility::ProgramOptionExists(argc, argv, "--stereo_autoexposure");
        stereo_whitebalance = utility::ProgramOptionExists(argc, argv, "--stereo_whitebalance");

        rgb_autoexposure = utility::ProgramOptionExists(argc, argv, "--rgb_exposure");
        rgb_whitebalance = utility::ProgramOptionExists(argc, argv, "--rgb_whitebalance");


        //Initialize to true, after we get 30 frames, we can set to the appropriate setting
        set_stereo_autoexposure(true);
        set_stereo_whitebalance(true);

        set_rgb_autoexposure(true);
        set_rgb_whitebalance(true);

        set_depth_units(0.001/depth_mult);
        set_max_laser_power();

        set_roi(100,540,100,200);
        
        if(utility::ProgramOptionExists(argc, argv, "--high_accuracy")){
            set_high_accuracy();
        }
    }

    img_idx = utility::GetProgramOptionAsInt(argc, argv,  "--iidx",  10);

    //Camera params
    width               = utility::GetProgramOptionAsInt(argc, argv,  "--width",  640);
    height              = utility::GetProgramOptionAsInt(argc, argv,  "--height", 480);
    fps                 = utility::GetProgramOptionAsInt(argc, argv,  "--fps",    30);
    framestart          = utility::GetProgramOptionAsInt(argc, argv,  "--fstart", 30);
    frames              = utility::GetProgramOptionAsInt(argc, argv,  "--frames", 480);

    //Capture params
    capture_gps     = utility::ProgramOptionExists(argc, argv, "--gps");
    capture_imu     = utility::ProgramOptionExists(argc, argv, "--imu");

    //Post Processing
    bool align_color_to_depth = utility::ProgramOptionExists(argc, argv, "--align_color_to_depth");
    if(align_color_to_depth){
        aligner = RS2_STREAM_DEPTH;
    }else{
        aligner = RS2_STREAM_COLOR;
    }

    use_filter  = utility::ProgramOptionExists(argc, argv,        "--use_filter");
    dec_mag     = utility::GetProgramOptionAsDouble(argc, argv,   "--dec-mag",    1.0) ;

    //Unused
    spat_mag    = utility::GetProgramOptionAsDouble(argc, argv,   "--spat-mag",   1.0);
    spat_a      = utility::GetProgramOptionAsDouble(argc, argv,   "--spat-a",     0.5);
    spat_d      = utility::GetProgramOptionAsDouble(argc, argv,   "--spat-d",     25);
    temp_a      = utility::GetProgramOptionAsDouble(argc, argv,   "--temp-a",     0.5);
    temp_d      = utility::GetProgramOptionAsDouble(argc, argv,   "--temp-d",     50);

    //Integration Params
    tsdf_cubic_size = utility::GetProgramOptionAsDouble(argc, argv, "--tsdf_cubic",      3.0/depth_mult);
    tsdf_truncation = utility::GetProgramOptionAsDouble(argc, argv, "--tsdf_truncation", 0.04/depth_mult);

    //RGBD Image params
    min_depth   = utility::GetProgramOptionAsDouble(argc, argv,   "--min_depth",      0.01/depth_mult);
    max_depth   = utility::GetProgramOptionAsDouble(argc, argv,   "--max_depth",      3.0/depth_mult);
    depth_factor= utility::GetProgramOptionAsDouble(argc, argv,   "--depth_factor",   1000*depth_mult);

    //Odometry Params
    use_imu             = utility::ProgramOptionExists(argc, argv,        "--use_imu");

    frames_per_fragment = utility::GetProgramOptionAsInt(argc, argv,      "--frames_per_fragment", 120);
    overlap_factor      = utility::GetProgramOptionAsInt(argc, argv,      "--overlap_factor",      4);
    rgbd_lookback       = utility::GetProgramOptionAsInt(argc, argv,      "--rgbd_lookback",       2);

    max_depth_diff      = utility::GetProgramOptionAsDouble(argc, argv,   "--max_depth_diff",     0.075 * depth_mult);
    loop_close_odom     = utility::GetProgramOptionAsDouble(argc, argv,   "--loop_closure_odom",  0.2);
    
    //Refine Params
    registration_window_size = utility::GetProgramOptionAsInt(argc, argv,   "--registration_window",        5);

    loop_close_reg  = utility::GetProgramOptionAsDouble(argc, argv,"--loop_closure_registration",  0.8);
    voxel_size      = utility::GetProgramOptionAsDouble(argc, argv,"--voxel_size",                 0.005/depth_mult);
    final_max_depth = utility::GetProgramOptionAsDouble(argc, argv,"--final_max_depth",            3.0/depth_mult);

    //DoN params
    don_downsample = utility::GetProgramOptionAsDouble(argc, argv, "--don_downsample", 0.02/depth_mult);

    don_small = utility::GetProgramOptionAsDouble(argc, argv,      "--don_small",      0.03/depth_mult);
    don_large = utility::GetProgramOptionAsDouble(argc, argv,      "--don_large",      0.5/depth_mult);

    threshold_min = utility::GetProgramOptionAsDouble(argc, argv,  "--don_thresh_min", 0.01/depth_mult);//, 0.032);
    threshold_max = utility::GetProgramOptionAsDouble(argc, argv,  "--don_thresh_max", 0.9/depth_mult);//, 0.1);

    //Cluster Params
    cluster_radius  = utility::GetProgramOptionAsDouble(argc, argv,   "--cluster_rad",  0.02/depth_mult);
    cluster_min     = utility::GetProgramOptionAsInt(argc, argv,     "--cluster_min",  100);
    cluster_max     = utility::GetProgramOptionAsInt(argc, argv,     "--cluster_max",  10000);


    //Set threshold
    threshold.set_option(RS2_OPTION_MIN_DISTANCE,min_depth);
    threshold.set_option(RS2_OPTION_MAX_DISTANCE,max_depth);
    
    //Set Filter opts
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,      dec_mag);  
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE,     spat_mag);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,  spat_a);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,  spat_d);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA,  temp_a);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA,  temp_d);

    depth_to_disparity = rs2::disparity_transform(true);
    disparity_to_depth = rs2::disparity_transform(false);

}

bool Config::ConvertFromJsonValue(const Json::Value &value)  {

    if (!value.isObject()) {
        open3d::utility::PrintWarning("DatasetConfig read JSON failed: unsupported json format.\n");
        return false;
    }

    session_path = value.get("session", "").asString();
    session_prefix = value.get("session-name", "").asString();

    if (session_path == "latest"){
        session_path =  "/home/ben/local-sessions/latest";
    }

    depth_mult = value.get("depth-mult", 1).asInt();

    if(value.get("set-cam-opts", false).asBool()){

        stereo_autoexposure = value.get("stereo-autoexposure", true).asBool();
        stereo_whitebalance = value.get("stereo-whitebalance", true).asBool();

        rgb_autoexposure = value.get("rgb-autoexposure", false).asBool();
        rgb_whitebalance = value.get("rgb-whitebalance", false).asBool();

        set_stereo_autoexposure(true);
        set_stereo_whitebalance(true);

        set_rgb_autoexposure(true);
        set_rgb_whitebalance(true);


        rgb_gamma = value.get("rgb-gamma", 450).asInt();
        rgb_saturation = value.get("rgb-saturation", 10).asInt();
        rgb_gain = value.get("rgb-gain", 128).asInt();
        rgb_sharpness = value.get("rgb-sharpness", 100).asInt();

        if(!set_rgb_gamma(rgb_gamma)) std::cout << "Failed to set gamma" << std::endl;
        if(!set_rgb_saturation(rgb_saturation)) std::cout << "Failed to set sat" << std::endl;
        if(!set_rgb_gain(rgb_gain)) std::cout << "Failed to set gain" << std::endl;
        if(!set_rgb_sharpness(rgb_sharpness)) std::cout << "Failed to set gain" << std::endl;

        set_roi(100, 540 ,100,200);

        set_depth_units(0.001/depth_mult);
        set_max_laser_power();


        if(value.get("high-accuracy", false).asBool()){
            set_high_accuracy();
        }
    }


    img_idx = value.get("iidx",  10).asInt();

    //Camera params
    width               = value.get("width",  640).asInt();
    height              = value.get("height", 480).asInt();
    fps                 = value.get("fps",    30).asInt();
    framestart          = value.get("fstart", 30).asInt();
    frames              = value.get("frames", 480).asInt();

    //Capture params
    capture_gps     = value.get( "gps", false).asBool();
    capture_imu     = value.get( "imu", false).asBool();

    //Post Processing
    bool align_color_to_depth = value.get("align-color-to-depth", false).asBool();
    if(align_color_to_depth){
        aligner = RS2_STREAM_DEPTH;
    }else{
        aligner = RS2_STREAM_COLOR;
    }

    use_filter  = value.get("use-filter", false).asBool();
    dec_mag     = value.get("dec-mag",    1.0).asDouble();

    //Unused
    spat_mag    = value.get("spat-mag",   1.0).asDouble();
    spat_a      = value.get("spat-a",     0.5).asDouble();
    spat_d      = value.get("spat-d",     25).asDouble();
    temp_a      = value.get("temp-a",     0.5).asDouble();
    temp_d      = value.get("temp-d",     50).asDouble();

    //Integration Params
    tsdf_cubic_size = value.get("tsdf-cubic",      7.0).asDouble() / depth_mult;
    tsdf_truncation = value.get("tsdf-truncation", 0.05).asDouble() / depth_mult;

    //RGBD Image params
    min_depth   = value.get("min-depth",      0.1).asDouble() /depth_mult;
    max_depth   = value.get("max-depth",      10.0).asDouble() / depth_mult;
    depth_factor= value.get("depth-factor",   1000).asDouble() * depth_mult;

    //Odometry Params
    use_imu             = value.get("use-imu", false).asBool();

    frames_per_fragment = value.get("frames-per-fragment", 120).asInt();

    overlap_factor      = value.get("overlap-factor",     4).asInt();
    rgbd_lookback       = value.get("rgbd-lookback",      2).asInt();
    max_depth_diff      = value.get("max-depth-diff",     0.075).asDouble() * depth_mult;
    loop_close_odom     = value.get("loop-closure-odom",  0.2).asDouble();
    
    //Refine Params
    registration_window_size= value.get("registration-window",        5).asInt();

    loop_close_reg          = value.get("loop-closure-registration",  0.8).asDouble();
    voxel_size              = value.get("voxel-size",                 0.005).asDouble() / depth_mult;
    final_max_depth         = value.get("final-max-depth",            3.0).asDouble() / depth_mult;

    //DoN params
    don_downsample = value.get("don-downsample", 0.02).asDouble() / depth_mult;

    don_small = value.get("don-small",      0.03).asDouble() / depth_mult;
    don_large = value.get("don-large",      0.5).asDouble() / depth_mult;

    threshold_min = value.get("don-thresh-min", 0.01).asDouble() / depth_mult;
    threshold_max = value.get("don-thresh-max", 0.9).asDouble() / depth_mult;

    //Cluster Params
    cluster_radius  = value.get("cluster-rad",  0.02).asDouble() / depth_mult;
    cluster_min     = value.get("cluster-min",  100).asInt();
    cluster_max     = value.get("cluster-max",  10000).asInt();


    return true;
}

int Config::GetFragmentIdForFrame(int frame_id) { return  frame_id / frames_per_fragment; }
int Config::GetOverlapCount(){ return frames_per_fragment / overlap_factor; }
int Config::GetFragmentCount() { return frames / frames_per_fragment; }

std::tuple<int,int> Config::GetFramesFromFragment(int fragment_id) {
    int fstart = fragment_id * frames_per_fragment;
    if(fragment_id>0){
        fstart -= GetOverlapCount();
    }
    return std::make_tuple(fstart, (frames_per_fragment * fragment_id) + frames_per_fragment);
}

void Config::CreateLocalSession()
{
    session_path = generate_local_session(session_prefix);
}


std::string Config::IMUFile()
{
    std::stringstream ss;
    ss << session_path <<  "/imu.csv";
    return ss.str();
}

std::string Config::IntrinsicFile()
{
    
    std::stringstream ss;
    ss << session_path <<  "/intrinsic.json";
    return ss.str();
}

std::string Config::FragmentFile(int idx)
{
    std::stringstream ss;
    ss << session_path <<  "/fragment/";
    ss << std::setw(5) << std::setfill('0') << idx << ".pcd";
    return ss.str();
}

std::string Config::ThumbnailFragmentFile(int idx)
{
    std::stringstream ss;
    ss << session_path <<  "/thumbnail_fragment/";
    ss << std::setw(5) << std::setfill('0') << idx << ".ply";
    return ss.str();
}

std::string Config::ColorFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/color/";
    ss << std::setw(6) << std::setfill('0') <<  idx << ".jpg";
    return ss.str();
}

std::string Config::DepthFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/depth/";
    ss << std::setw(6) << std::setfill('0') << idx << ".png";
    return ss.str();
}

std::string Config::MaskFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/masks/";
    ss << std::setw(6) << std::setfill('0') << idx << "_pred_labelIds.png";
    return ss.str();
}

std::string Config::InfraFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/infra/";
    ss << std::setw(6) << std::setfill('0') << idx << ".png";
    return ss.str();
}

std::string Config::PoseFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/pose/";
    if(idx>=0){
        ss << std::setw(5) << std::setfill('0') << idx << ".json";
    }else{
        ss << "full.json";
    }
    return ss.str();
}

std::string Config::SceneMeshFile()
{
    
    std::stringstream ss;
    ss << session_path <<  "/scene/scene.ply";
    return ss.str();
}

std::string Config::PoseFileScene()
{
    
    std::stringstream ss;
    ss << session_path <<  "/scene/pose.json";
    return ss.str();
}

std::string Config::PoseFileSceneRectified()
{
    
    std::stringstream ss;
    ss << session_path <<  "/scene/pose_rectified.json";
    return ss.str();
}

std::string Config::ImageTimestampFile()
{
    std::stringstream ss;
    ss << session_path <<  "/timestamps.csv";
    return ss.str();
}

std::string Config::GPSFile()
{
    
    std::stringstream ss;
    ss << session_path <<  "/gps.csv";
    return ss.str();
}


rs2::frame Config::Filter(rs2::depth_frame depth)
{
    return dec_filter.process(depth);
    //depth = depth_to_disparity.process(depth);
    //depth = spat_filter.process(depth);
    //depth = temp_filter.process(depth);
    //return disparity_to_depth.process(depth);
}
