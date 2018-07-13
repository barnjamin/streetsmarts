#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cstdio>
#include <iostream>
#include <exception>
#include <mutex>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

const char* window_name = "Edge Map";
const int WIDTH = 640;
const int HEIGHT = 480;
rs2::frame_queue q;

cv::Mat frame_to_mat(const rs2::frame& f);

Mat markerMask, img;
Point prevPt(-1, -1);


static void onMouse( int event, int x, int y, int flags, void* )
{
    if( x < 0 || x >= img.cols || y < 0 || y >= img.rows )
        return;
    if( event == EVENT_LBUTTONUP || !(flags & EVENT_FLAG_LBUTTON) )
        prevPt = Point(-1,-1);
    else if( event == EVENT_LBUTTONDOWN )
        prevPt = Point(x,y);
    else if( event == EVENT_MOUSEMOVE && (flags & EVENT_FLAG_LBUTTON) )
    {
        Point pt(x, y);
        if( prevPt.x < 0 )
            prevPt = pt;
        line( markerMask, prevPt, pt, Scalar::all(255), 5, 8, 0 );
        line( img, prevPt, pt, Scalar::all(255), 5, 8, 0 );
        prevPt = pt;
        imshow("image", img);
    }
}




// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::pipeline pipe;
    pipe.start();

    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    auto img0 = frame_to_mat(data.get_color_frame());
    img0.copyTo(img);

    Mat imgGray;

    cvtColor(img, markerMask, COLOR_BGR2GRAY);

    cvtColor(markerMask, imgGray, COLOR_GRAY2BGR);

    namedWindow( "image", 1 );

    markerMask = Scalar::all(0);
    imshow( "image", img );
    setMouseCallback( "image", onMouse, 0 );

    for(;;)
    {
        std::cout<<"hiadsf"<<std::endl;
        char c = (char)waitKey(0);
        if( c == 27 )
            break;
        if( c == 'r' )
        {
            std::cout<<"bai"<<std::endl;
            markerMask = Scalar::all(0);
            img0.copyTo(img);
            cv::imshow( "image", img );
        }
        if( c == 'w' || c == ' ' )
        {
            int i, j, compCount = 0;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours(markerMask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            if( contours.empty() )
                continue;
            Mat markers(markerMask.size(), CV_32S);
            markers = Scalar::all(0);
            int idx = 0;
            for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
                drawContours(markers, contours, idx, Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);
            if( compCount == 0 )
                continue;
            vector<Vec3b> colorTab;
            for( i = 0; i < compCount; i++ )
            {
                int b = theRNG().uniform(0, 255);
                int g = theRNG().uniform(0, 255);
                int r = theRNG().uniform(0, 255);
                colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
            }
            double t = (double)getTickCount();
            watershed(img0, markers);
            t = (double)getTickCount() - t;
            printf( "execution time = %gms\n", t*1000./getTickFrequency() );
            Mat wshed(markers.size(), CV_8UC3);
            // paint the watershed image
            for( i = 0; i < markers.rows; i++ )
                for( j = 0; j < markers.cols; j++ )
                {
                    int index = markers.at<int>(i,j);
                    if( index == -1 )
                        wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
                    else if( index <= 0 || index > compCount )
                        wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
                    else
                        wshed.at<Vec3b>(i,j) = colorTab[index - 1];
                }
            wshed = wshed*0.5 + imgGray*0.5;
            cv::imshow("watershedded", wshed );
            std::cout<<"watersheddedboi hi"<<std::endl;
        }
    }
    return 0;
}catch (const rs2::error & e)
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

Mat frame_to_mat(const rs2::frame& f)
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