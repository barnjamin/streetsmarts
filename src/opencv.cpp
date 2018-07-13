// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/imgproc/imgproc.hpp>
#include <exception>
#include <mutex>
#include <chrono>
#include <thread>

const char* window_name = "Edge Map";
const int WIDTH = 640;
const int HEIGHT = 480;
rs2::frame_queue q;

cv::Mat frame_to_mat(const rs2::frame& f);

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::colorizer color_map;

    rs2::pipeline pipe;
	rs2::config cfg;

	rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    //rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    //rs2::temporal_filter temp_filter;

	dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2.0);

	cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT);
	cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT);
	pipe.start(cfg);

	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

	while (cv::waitKey(1)<0 && cvGetWindowHandle(window_name)) {
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		auto depth = data.get_depth_frame();
		auto filtered_depth = depth;

		filtered_depth = dec_filter.process(filtered_depth);

		auto mat = frame_to_mat(color_map(filtered_depth));

		//Canny
		cv::Mat src, src_gray;
		cv::Mat dst, detected_edges;
		int edgeThresh = 1;
		int lowThreshold=80;
		int const max_lowThreshold = 100;
		int ratio = 3;
		int kernel_size = 3;
		dst.create(cv::Size(WIDTH, HEIGHT), CV_16UC1);
		cv::cvtColor(mat, src_gray, cv::COLOR_BGR2GRAY);
		cv::blur(src_gray, detected_edges, cv::Size(3, 3));
		cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
		dst = cv::Scalar::all(0);
		src_gray.copyTo(dst, detected_edges);
		cv::imshow(window_name, dst);

	}

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 15));
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000 * 15));
    return EXIT_FAILURE;
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