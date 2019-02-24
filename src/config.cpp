#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <Core/Core.h>
#include "config.h"
#include <iostream>
#include <iomanip>


std::string generate_local_session() {
    using namespace open3d;

    std::stringstream ss;
    std::time_t result = std::time(0);
    ss << "/home/ben/local-sessions/abc123-" <<  result;

    std::string session_path = ss.str(); 

    if(!filesystem::MakeDirectoryHierarchy(session_path + "/pose") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/color") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/depth") ||
        !filesystem::MakeDirectoryHierarchy(session_path + "/fragment")){
        std::cout << "failed to create files" << std::endl; 
    }

    return session_path; 
}

Config::~Config() { }

Config::Config(int argc, char ** argv) {
    using namespace open3d;

    session_path = GetProgramOptionAsString(argc,argv,"--session");
    if (session_path == "") {
        session_path = generate_local_session(); 
    }


    tsdf_cubic_size = GetProgramOptionAsDouble(argc, argv, "--tsdf_cubic", 15.0);
    tsdf_truncation = GetProgramOptionAsDouble(argc, argv, "--tsdf_truncation", 0.15);

    min_depth = GetProgramOptionAsDouble(argc,argv, "--min_depth", 1.0);
    max_depth = GetProgramOptionAsDouble(argc,argv, "--max_depth", 10.0);
    depth_factor = GetProgramOptionAsDouble(argc,argv, "--depth_factor", 2.0);

    fps                 = GetProgramOptionAsInt(argc,argv, "--fps", 30);
    frames_per_fragment = GetProgramOptionAsInt(argc,argv,  "--frames_per_fragment", 30);
    framestart          = GetProgramOptionAsInt(argc,argv,  "--fstart", 30);


    dec_mag     = GetProgramOptionAsDouble(argc,argv,  "--dec-mag", 1.0) ;
    spat_mag    = GetProgramOptionAsDouble(argc,argv,  "--spat-mag", 1.0);
    spat_a      = GetProgramOptionAsDouble(argc,argv,  "--spat-a", 0.5);
    spat_d      = GetProgramOptionAsDouble(argc,argv,  "--spat-d", 25);
    temp_a      = GetProgramOptionAsDouble(argc,argv,  "--temp-a", 0.5);
    temp_d      = GetProgramOptionAsDouble(argc,argv,  "--temp-d", 50);

    don_downsample = GetProgramOptionAsDouble(argc,argv, "--don_downsample", 0.02);

    don_small = GetProgramOptionAsDouble(argc,argv, "--don_small", 0.03);
    don_large = GetProgramOptionAsDouble(argc,argv, "--don_large", 0.5);

    threshold_min = GetProgramOptionAsDouble(argc,argv, "--don_thresh_min", 0.032);
    threshold_max = GetProgramOptionAsDouble(argc,argv, "--don_thresh_max", 0.1);

    cluster_radius = GetProgramOptionAsDouble(argc,argv, "--cluster_rad", 0.02);
    cluster_min = GetProgramOptionAsInt(argc, argv, "--cluster_min",  100);
    cluster_max = GetProgramOptionAsInt(argc, argv, "--cluster_max",  10000);

    width   = GetProgramOptionAsInt(argc,argv, "--width", 640);
    height  = GetProgramOptionAsInt(argc,argv, "--height", 480);

    use_imu     = ProgramOptionExists(argc,argv, "--use_imu");
    use_filter  = ProgramOptionExists(argc,argv, "--use_filter");


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



std::string Config::FragmentFile(int idx)
{
    std::stringstream ss;
    ss << session_path <<  "/fragment/";
    ss << std::setw(5) << std::setfill('0') << idx << ".ply";
    return ss.str();
}

std::string Config::ColorFile(int f_idx, int i_idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/color/";
    ss << std::setw(5) << std::setfill('0') <<  f_idx << "_" << i_idx  << ".jpg";
    return ss.str();
}

std::string Config::DepthFile(int f_idx, int i_idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/depth/";
    ss << std::setw(5) << std::setfill('0') << f_idx << "_" << i_idx << ".png";
    return ss.str();
}

std::string Config::PoseFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/pose/";
    ss << std::setw(5) << std::setfill('0') << idx << ".json";
    return ss.str();
}

rs2::frame Config::Filter(rs2::depth_frame depth)
{
    depth = dec_filter.process(depth);
    depth = depth_to_disparity.process(depth);
    depth = spat_filter.process(depth);
    depth = temp_filter.process(depth);
    return disparity_to_depth.process(depth);
}
