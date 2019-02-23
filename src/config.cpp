#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <Core/Core.h>
#include "config.h"
#include <iostream>
#include <iomanip>


std::string generate_local_session() {
    std::stringstream ss;
    std::time_t result = std::time(nullptr);
    ss << "/home/ben/local_sessions/" <<  std::asctime(std::localtime(&result));
    return ss.str();
}

Config::~Config() { }

Config::Config() {
    min_depth       = 0.0;
    max_depth       = 5.0;
    depth_factor    = 2.0;


    tsdf_cubic_size = 5.0;
    tsdf_truncation = 1.0;

    fps      	            = 30;
    frames_per_fragment     = 180;
    framestart              = 30; //Number of frames to discard at the start

    width 	= 640;
    height 	= 480;

    dec_mag     = 1.0;
    spat_mag    = 1.0;
    spat_a      = 0.5;
    spat_d      = 25;
    temp_a      = 0.5;
    temp_d      = 50;

    depth_to_disparity = rs2::disparity_transform(true);
    disparity_to_depth = rs2::disparity_transform(false);

    don_small			= 0.05;
    don_large			= 0.25;

    threshold_min      = 0.032;
    threshold_max       = 0.1;

    segradius			= 0.02;  // threshold for radius segmentation

    use_imu             = true;
    use_filter          = false;

    session_path        = generate_local_session();

}



std::string Config::FragmentFile(int idx)
{
    std::stringstream ss;
    ss << session_path <<  "/fragment/";
    ss << std::setw(5) << std::setfill('0') << idx << ".ply";
    return ss.str();
}

std::string Config::ColorFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/color/";
    ss << std::setw(5) << std::setfill('0') << idx << ".jpg";
    return ss.str();
}

std::string Config::DepthFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/depth/";
    ss << std::setw(5) << std::setfill('0') << idx << ".png";
    return ss.str();
}

std::string Config::PoseFile(int idx)
{
    
    std::stringstream ss;
    ss << session_path <<  "/pose/";
    ss << std::setw(5) << std::setfill('0') << idx << ".json";
    return ss.str();
}

void Config::parseArgs(int argc, char **argv) {

    using namespace open3d;

    if (argc == 1 || ProgramOptionExists(argc, argv, "--help")) {
        return;
    }

    session_path = GetProgramOptionAsString(argc,argv,"--session");
    if (session_path == "") {
        session_path = generate_local_session(); 
    }

    tsdf_cubic_size = GetProgramOptionAsDouble(argc,argv, "--tsdf_cubic");
    tsdf_truncation = GetProgramOptionAsDouble(argc,argv, "--tsdf_truncation");

    min_depth = GetProgramOptionAsDouble(argc,argv, "--min_depth");
    max_depth = GetProgramOptionAsDouble(argc,argv, "--max_depth");
    depth_factor = GetProgramOptionAsDouble(argc,argv, "--depth_factor");

    fps                 = GetProgramOptionAsInt(argc,argv, "--fps");
    frames_per_fragment = GetProgramOptionAsInt(argc,argv,  "--frames_per_fragment");
    framestart          = GetProgramOptionAsInt(argc,argv,  "--fstart");

    dec_mag     = GetProgramOptionAsDouble(argc,argv,  "--dec-mag");
    spat_mag    = GetProgramOptionAsDouble(argc,argv,  "--spat-mag");
    spat_a      = GetProgramOptionAsDouble(argc,argv,  "--spat-a");
    spat_d      = GetProgramOptionAsDouble(argc,argv,  "--spat-d");
    temp_a      = GetProgramOptionAsDouble(argc,argv,  "--temp-a");
    temp_d      = GetProgramOptionAsDouble(argc,argv,  "--temp-d");

    don_small = GetProgramOptionAsDouble(argc,argv, "--don_small");
    don_large = GetProgramOptionAsDouble(argc,argv, "--don_large");

    threshold_min = GetProgramOptionAsDouble(argc,argv, "--don_thresh_min");
    threshold_max = GetProgramOptionAsDouble(argc,argv, "--don_thresh_max");

    segradius = GetProgramOptionAsDouble(argc,argv, "--don_rad");


    width   = GetProgramOptionAsInt(argc,argv, "--width");
    height  = GetProgramOptionAsInt(argc,argv, "--height");

    use_imu     = ProgramOptionExists(argc,argv, "--use_imu");
    use_filter  = ProgramOptionExists(argc,argv, "--use_imu");


    //Set Filter opts
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_mag);  

    spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, spat_mag);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spat_a);
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spat_d);

    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temp_a);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, temp_d);
}



rs2::frame Config::filter(rs2::depth_frame depth)
{
    depth = dec_filter.process(depth);
    depth = depth_to_disparity.process(depth);
    depth = spat_filter.process(depth);
    depth = temp_filter.process(depth);
    return disparity_to_depth.process(depth);
}
