
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "utils.h"

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

Config::~Config() { }

Config::Config() {
    imu_src     = "/dev/ttyACM0";
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
    }else if(flag == "--imu_src"){
      imu_src = argv[x+1];
    }else if(flag == "--width"){
      width = std::stoi(argv[x+1]);
    }else if(flag == "--height"){
      height = std::stoi(argv[x+1]);
    }else if(flag == "--fps"){
      fps = std::stoi(argv[x+1]);
    }
  }

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

open3d::PinholeCameraIntrinsic get_intrinsics(rs2::pipeline_profile prof)
{
    auto depth_stream = prof.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();
    auto intrin = depth_stream.get_intrinsics();

    open3d::PinholeCameraIntrinsic intrinsics(depth_stream.width(), depth_stream.height(), 
            intrin.fx, intrin.fy, intrin.ppx, intrin.ppy);

    return intrinsics;
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
