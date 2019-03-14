#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <Core/Core.h>
#include "config.h"
#include <iostream>
#include <iomanip>
#include <unistd.h>

std::string generate_local_session() {
    using namespace open3d;

    std::stringstream ss;
    std::time_t result = std::time(0);
    ss << "/home/ben/local-sessions/abc123-" <<  result;

    std::string session_path = ss.str(); 

    if(!filesystem::MakeDirectoryHierarchy(session_path + "/pose") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/scene") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/color") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/depth") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/fragment")){
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
    session_path = GetProgramOptionAsString(argc,argv,"--session");
    if (session_path == "latest"){
        session_path =  "/home/ben/local-sessions/latest";
    }else if (session_path == "") {
        session_path = generate_local_session(); 
    }

    //Camera params
    width               = GetProgramOptionAsInt(argc,argv, "--width", 640);
    height              = GetProgramOptionAsInt(argc,argv, "--height", 480);
    fps                 = GetProgramOptionAsInt(argc,argv, "--fps", 30);
    framestart          = GetProgramOptionAsInt(argc,argv,  "--fstart", 30);

    //Post Processing
    use_filter  = ProgramOptionExists(argc,argv, "--use_filter");
    dec_mag     = GetProgramOptionAsDouble(argc,argv,  "--dec-mag", 1.0) ;
    spat_mag    = GetProgramOptionAsDouble(argc,argv,  "--spat-mag", 1.0);
    spat_a      = GetProgramOptionAsDouble(argc,argv,  "--spat-a", 0.5);
    spat_d      = GetProgramOptionAsDouble(argc,argv,  "--spat-d", 25);
    temp_a      = GetProgramOptionAsDouble(argc,argv,  "--temp-a", 0.5);
    temp_d      = GetProgramOptionAsDouble(argc,argv,  "--temp-d", 50);

    //Integration Params
    tsdf_cubic_size = GetProgramOptionAsDouble(argc, argv, "--tsdf_cubic", 5.0);
    tsdf_truncation = GetProgramOptionAsDouble(argc, argv, "--tsdf_truncation", 0.03);

    //RGBD Image params
    min_depth = GetProgramOptionAsDouble(argc,argv, "--min_depth", 0.5);
    max_depth = GetProgramOptionAsDouble(argc,argv, "--max_depth", 3.0);
    depth_factor = GetProgramOptionAsDouble(argc,argv, "--depth_factor", 1000.0);

    //Odometry Params
    use_imu             = ProgramOptionExists(argc,argv, "--use_imu");
    fragments          = GetProgramOptionAsInt(argc,argv,  "--fragments", 8);
    frames_per_fragment = GetProgramOptionAsInt(argc,argv,  "--frames_per_fragment", 30);
    preference_loop_closure_odometry = GetProgramOptionAsDouble(argc, argv, "--loop_closure_odom", 0.1);
    max_depth_diff = GetProgramOptionAsDouble(argc, argv, "--max_depth_diff", 0.07);

    //DoN params
    don_downsample = GetProgramOptionAsDouble(argc,argv, "--don_downsample", 0.02);

    don_small = GetProgramOptionAsDouble(argc,argv, "--don_small", 0.03);
    don_large = GetProgramOptionAsDouble(argc,argv, "--don_large", 0.5);

    threshold_min = GetProgramOptionAsDouble(argc,argv, "--don_thresh_min", 0.032);
    threshold_max = GetProgramOptionAsDouble(argc,argv, "--don_thresh_max", 0.1);

    //Cluster Params
    cluster_radius  = GetProgramOptionAsDouble(argc,argv, "--cluster_rad", 0.02);
    cluster_min     = GetProgramOptionAsInt(argc, argv, "--cluster_min",  100);
    cluster_max     = GetProgramOptionAsInt(argc, argv, "--cluster_max",  10000);

    //Refine Params
    registration_window_size = GetProgramOptionAsInt(argc, argv, "--registration_window",  5);
    preference_loop_closure_registration = GetProgramOptionAsDouble(argc, argv, "--loop_closure_registration",  0.5);
    voxel_size = GetProgramOptionAsDouble(argc, argv, "--voxel_size", 0.05);

    //Set threshold
    threshold.set_option(RS2_OPTION_MIN_DISTANCE,min_depth);
    threshold.set_option(RS2_OPTION_MAX_DISTANCE,max_depth);
    
    //Set Filter opts
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_mag);  
    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, spat_mag);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spat_a);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spat_d);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temp_a);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, temp_d);

    depth_to_disparity = rs2::disparity_transform(true);
    disparity_to_depth = rs2::disparity_transform(false);
}

//Set Alignment, default arg? TODO:
rs2::align Config::Aligner(){
    return rs2::align(RS2_STREAM_COLOR);
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

int Config::GetFragmentCount() 
{

    //TODO
    // Use fragment dir to get fragment count
     
    std::stringstream ss;
    ss << session_path <<  "/fragment";

    std::vector<std::string> filenames;

    if(!open3d::filesystem::ListFilesInDirectory(ss.str(), filenames)) {
        return 0 ;
    }

    return filenames.size();
}

rs2::frame Config::Filter(rs2::depth_frame depth)
{
    depth = dec_filter.process(depth);
    depth = depth_to_disparity.process(depth);
    depth = spat_filter.process(depth);
    depth = temp_filter.process(depth);
    return disparity_to_depth.process(depth);
}
