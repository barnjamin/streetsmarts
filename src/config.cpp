#include <iostream>
#include <iomanip>
#include <unistd.h>

#include <json/json.h>
#include <librealsense2/rs.hpp>

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

Config::Config(int argc, char ** argv) {
    using namespace open3d;

    // Where to write the files
    session_path = utility::GetProgramOptionAsString(argc, argv, "--session");
    session_prefix = utility::GetProgramOptionAsString(argc, argv, "--session_prefix", "abc123");

    if (session_path == "latest"){
        session_path =  "/home/ben/local-sessions/latest";
    }


    depth_mult = utility::GetProgramOptionAsInt(argc, argv, "--depth_mult", 10);

    if(utility::ProgramOptionExists(argc, argv, "--high_accuracy")){
        set_depth_units(0.001/depth_mult);
        set_roi(100,540,100,200);
        set_high_accuracy();
    }

    if(utility::ProgramOptionExists(argc, argv, "--exposure")){
        exposure = utility::GetProgramOptionAsDouble(argc, argv, "--exposure", 166);
        set_exposure(exposure);
    }

    if(utility::ProgramOptionExists(argc, argv, "--wbalance")){
        white_balance = utility::GetProgramOptionAsDouble(argc, argv, "--wbalance", 4600);
        set_white_balance(white_balance);
    }


    img_idx = utility::GetProgramOptionAsInt(argc, argv,  "--iidx",  10);

    //Camera params
    width               = utility::GetProgramOptionAsInt(argc, argv,  "--width",  640);
    height              = utility::GetProgramOptionAsInt(argc, argv,  "--height", 480);
    fps                 = utility::GetProgramOptionAsInt(argc, argv,  "--fps",    30);
    framestart          = utility::GetProgramOptionAsInt(argc, argv,  "--fstart", 30);

    //Capture params
    capture_gps     = utility::ProgramOptionExists(argc, argv, "--gps");
    capture_imu     = utility::ProgramOptionExists(argc, argv, "--imu");

    make_fragments  = utility::ProgramOptionExists(argc, argv,   "--make_fragments");

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
    tsdf_cubic_size = utility::GetProgramOptionAsDouble(argc, argv, "--tsdf_cubic",      7.0/depth_mult);
    tsdf_truncation = utility::GetProgramOptionAsDouble(argc, argv, "--tsdf_truncation", 0.05/depth_mult);

    //RGBD Image params
    min_depth   = utility::GetProgramOptionAsDouble(argc, argv,   "--min_depth",      0.01/depth_mult);
    max_depth   = utility::GetProgramOptionAsDouble(argc, argv,   "--max_depth",      3.0/depth_mult);
    depth_factor= utility::GetProgramOptionAsDouble(argc, argv,   "--depth_factor",   1000*depth_mult);

    //Odometry Params
    use_imu             = utility::ProgramOptionExists(argc, argv,        "--use_imu");
    fragments           = utility::GetProgramOptionAsInt(argc, argv,      "--fragments",          8);
    frames_per_fragment = utility::GetProgramOptionAsInt(argc, argv,      "--frames_per_fragment",30);
    max_depth_diff      = utility::GetProgramOptionAsDouble(argc, argv,  "--max_depth_diff",     0.007*depth_mult);
    loop_close_odom     = utility::GetProgramOptionAsDouble(argc, argv,  "--loop_closure_odom",  0.2);
    
    //Refine Params
    registration_window_size= utility::GetProgramOptionAsInt(argc, argv,   "--registration_window",        5);

    loop_close_reg          = utility::GetProgramOptionAsDouble(argc, argv,"--loop_closure_registration",  0.8);
    voxel_size              = utility::GetProgramOptionAsDouble(argc, argv,"--voxel_size",                 0.005/depth_mult);

    //DoN params
    don_downsample = utility::GetProgramOptionAsDouble(argc, argv, "--don_downsample", 0.02/depth_mult);

    don_small = utility::GetProgramOptionAsDouble(argc, argv,      "--don_small",      0.04/depth_mult);
    don_large = utility::GetProgramOptionAsDouble(argc, argv,      "--don_large",      0.4/depth_mult);

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
    session_prefix = value.get("session_prefix", "").asString();

    if (session_path == "latest"){
        session_path =  "/home/ben/local-sessions/latest";
    }

    depth_mult = value.get("depth_mult", 10).asInt();

    if(value.get("high_accuracy", false).asBool()){
        set_depth_units(0.001/depth_mult);
        set_roi(100,540,100,200);
        set_high_accuracy();
    }

    if(value.get("exposure", false).asBool()){
        exposure = value.get("exposure", 166).asDouble();
        set_exposure(exposure);
    }

    if(value.get("wbalance", false).asBool()){
        white_balance = value.get("wbalance", 4600).asDouble();
        set_white_balance(white_balance);
    }


    img_idx = value.get("iidx",  10).asInt();

    //Camera params
    width               = value.get( "width",  640).asInt();
    height              = value.get("height", 480).asInt();
    fps                 = value.get("fps",    30).asInt();
    framestart          = value.get("fstart", 30).asInt();

    //Capture params
    capture_gps     = value.get( "gps", false).asBool();
    capture_imu     = value.get( "imu", false).asBool();

    make_fragments  = value.get("make_fragments", false).asBool();

    //Post Processing
    bool align_color_to_depth = value.get("align_color_to_depth", false).asBool();
    if(align_color_to_depth){
        aligner = RS2_STREAM_DEPTH;
    }else{
        aligner = RS2_STREAM_COLOR;
    }

    use_filter  = value.get("use_filter", false).asBool();
    dec_mag     = value.get("dec-mag",    1.0).asDouble();

    //Unused
    spat_mag    = value.get("spat-mag",   1.0).asDouble();
    spat_a      = value.get("spat-a",     0.5).asDouble();
    spat_d      = value.get("spat-d",     25).asDouble();
    temp_a      = value.get("temp-a",     0.5).asDouble();
    temp_d      = value.get("temp-d",     50).asDouble();

    //Integration Params
    tsdf_cubic_size = value.get("tsdf_cubic",      7.0/depth_mult).asDouble();
    tsdf_truncation = value.get("tsdf_truncation", 0.05/depth_mult).asDouble();

    //RGBD Image params
    min_depth   = value.get("min_depth",      0.01/depth_mult).asDouble();
    max_depth   = value.get("max_depth",      3.0/depth_mult).asDouble();
    depth_factor= value.get("depth_factor",   1000*depth_mult).asDouble();

    //Odometry Params
    use_imu             = value.get("use_imu", false).asBool();
    fragments           = value.get(  "fragments",          8).asInt();
    frames_per_fragment = value.get(  "frames_per_fragment",30).asInt();
    max_depth_diff      = value.get( "max_depth_diff",     0.007*depth_mult).asDouble();
    loop_close_odom     = value.get( "loop_closure_odom",  0.2).asDouble();
    
    //Refine Params
    registration_window_size= value.get("registration_window",        5).asInt();

    loop_close_reg          = value.get("loop_closure_registration",  0.8).asDouble();
    voxel_size              = value.get("voxel_size",                 0.005/depth_mult).asDouble();

    //DoN params
    don_downsample = value.get("don_downsample", 0.02/depth_mult).asDouble();

    don_small = value.get("don_small",      0.04/depth_mult).asDouble();
    don_large = value.get("don_large",      0.4/depth_mult).asDouble();

    threshold_min = value.get("don_thresh_min", 0.01/depth_mult).asDouble();//, 0.032);
    threshold_max = value.get("don_thresh_max", 0.9/depth_mult).asDouble();//, 0.1);

    //Cluster Params
    cluster_radius  = value.get( "cluster_rad",  0.02/depth_mult).asDouble();
    cluster_min     = value.get("cluster_min",  100).asInt();
    cluster_max     = value.get("cluster_max",  10000).asInt();


    return true;
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
    ss << std::setw(5) << std::setfill('0') << idx << ".ply";
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
    ss << std::setw(5) << std::setfill('0') << idx << ".json";
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

int Config::GetFragmentCount() 
{

    //TODO Use fragment dir to get fragment count
    
    std::stringstream ss;
    ss << session_path <<  "/fragment";

    std::vector<std::string> filenames;

    if(!open3d::utility::filesystem::ListFilesInDirectory(ss.str(), filenames)) {
        return 0 ;
    }

    return filenames.size();
}

rs2::frame Config::Filter(rs2::depth_frame depth)
{
    return dec_filter.process(depth);
    //depth = depth_to_disparity.process(depth);
    //depth = spat_filter.process(depth);
    //depth = temp_filter.process(depth);
    //return disparity_to_depth.process(depth);
}
