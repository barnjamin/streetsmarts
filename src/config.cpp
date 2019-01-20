#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "config.h"

Config::~Config() { }

Config::Config() {
    min_z       = 0.0f;
    max_z       = 5.0f;

    fps      	= 30;
    frames      = 180;
    framestart  = 10; //Number of frames to discard at the start

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
    threshold			= 0.1 ;
    segradius			= 0.02;  // threshold for radius segmentation

    icp_iters           = 30;
    icp_dist            = 1.0;
    icp_leaf            = 0.15;

    use_imu             = true;
    use_filter          = false;
}

Config::Config(std::string fname) {
    // Read in file
    // Set parameters

    min_z       = 0.0f;
    max_z       = 5.0f;

    fps      	= 30;
    frames      = 180;
    framestart  = 10; //Number of frames to discard at the start

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
    threshold			= 0.1 ;
    segradius			= 0.02;  // threshold for radius segmentation

    icp_iters           = 30;
    icp_dist            = 1.0;
    icp_leaf            = 0.15;

    use_imu             = true;
    use_filter          = false;
}

void Config::parseArgs(int argc, char **argv) {
  for(int x=1; x<argc; x+=2){
    std::string flag (argv[x]);
    if(flag == "--minz"){
       min_z = std::stof(argv[x+1]);
    } else if(flag == "--maxz"){
       max_z = std::stof(argv[x+1]);
    }else if(flag ==  "--frames"){
      frames = std::stoi(argv[x+1]);
    }else if(flag ==  "--fstart"){
      framestart = std::stoi(argv[x+1]);
    }else if(flag ==  "--dec-mag"){
      dec_mag = std::stoi(argv[x+1]);
    }else if(flag ==  "--spat-mag"){
      spat_mag = std::stoi(argv[x+1]);
    }else if(flag ==  "--spat-a"){
      spat_a = std::stof(argv[x+1]);
    }else if(flag ==  "--spat-d"){
      spat_d = std::stoi(argv[x+1]);
    }else if(flag ==  "--temp-a"){
      temp_a = std::stof(argv[x+1]);
    }else if(flag ==  "--temp-d"){
      temp_d = std::stoi(argv[x+1]);
    }else if(flag == "--don_small"){
      don_small = std::stof(argv[x+1]);
    }else if(flag == "--don_large"){
      don_large = std::stof(argv[x+1]);
    }else if(flag == "--don_thresh"){
      threshold = std::stof(argv[x+1]);
    }else if(flag == "--don_rad"){
      segradius = std::stof(argv[x+1]);
    }else if(flag == "--icp_iters"){
      icp_iters = std::stoi(argv[x+1]);
    }else if(flag == "--icp_dist"){
      icp_dist = std::stof(argv[x+1]);
    }else if(flag == "--icp_leaf"){
      icp_leaf = std::stof(argv[x+1]);
    }else if(flag == "--width"){
      width = std::stoi(argv[x+1]);
    }else if(flag == "--height"){
      height = std::stoi(argv[x+1]);
    }else if(flag == "--fps"){
      fps = std::stoi(argv[x+1]);
    }else if(flag == "--no_imu"){
      use_imu = false;
    }else if(flag == "--filter"){
      use_filter = true;
    }
  }

  dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_mag);  

  spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, spat_mag);
  spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spat_a);
  spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spat_d);

  temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, temp_a);
  temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, temp_d);
}


void Config::save(std::string fname) {
    //Save the file
}


rs2::frame Config::filter(rs2::depth_frame depth)
{
    depth = dec_filter.process(depth);
    depth = depth_to_disparity.process(depth);
    depth = spat_filter.process(depth);
    depth = temp_filter.process(depth);
    return disparity_to_depth.process(depth);
}
