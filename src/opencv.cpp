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

// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
	rs2::config cfg;

	std::cout << "Starting config\n";
	cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT);
	std::cout << "Done with config\n";

	pipe.start(cfg);
	std::cout << "Started pipe\n";


	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	std::cout << "Initialized window, starting while\n";

	while (cv::waitKey(1)<0 && cvGetWindowHandle(window_name)) {
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		auto mat =  frame_to_mat(data.get_color_frame());

		//Watershed
		//using namespace cv;

		//Mat gray;
		//cvtColor(mat, gray, CV_BGR2GRAY);
		//threshold(gray, gray, 100, 255, THRESH_BINARY+THRESH_OTSU);

		//Mat kernel(Size(3, 3), CV_8U);
		//Mat opening;
		//morphologyEx(gray, opening, MORPH_OPEN, kernel,Point(-1,-1), 2);

		//Mat sure_bg;
		//dilate(opening, sure_bg, kernel, Point(-1,-1), 3);

		//Mat dist;
		//distanceTransform(opening, dist, DIST_L2, 5);

		//double minVal;
		//double maxVal;
		//Point minLoc;
		//Point maxLoc;
		//minMaxLoc(dist, &minVal, &maxVal, &minLoc, &maxLoc);

		//Mat sure_fg;
		//threshold(dist, sure_fg, 0.1*maxVal, 255, 0);

		//sure_fg.convertTo(sure_fg, CV_8U);

		//Mat unknown;
		//subtract(sure_bg, sure_fg, unknown);
		//imshow(window_name, sure_fg);

		//Mat markers;
		//connectedComponents(sure_fg, markers, 4, CV_32S);
		//markers += 1;


		//markers.convertTo(markers, CV_8U);

		//unknown.convertTo(unknown, CV_8U);

		//uchar* ptr;
		//for (int row = 0; row < unknown.rows; ++row) {
		//	ptr = unknown.ptr<uchar>(row);
		//	for (int col = 0; col < unknown.cols; ++col) {
		//		if (ptr[col] == 255) {
		//			markers.at<uchar>(row, col) = 0;
		//		}
		//	}
		//}

		//markers.convertTo(markers, mat.type());
		//watershed(mat, markers);

		//imshow(window_name, markers);

		//threshold(dialated, dialated, 1, 50, THRESH_BINARY_INV);

		//Mat path_trace(gray.size(), CV_8U, Scalar(0));
		//path_trace = eroded + dialated;


		//Mat path;
		//path_trace.convertTo(path, CV_32S);
		//watershed(mat, path);

        //path.convertTo(path, CV_8U);
		//imshow(window_name, path);

		//std::cerr << dest(Rect(30, 30, 10, 10)) << std::endl; // peek into the data

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


