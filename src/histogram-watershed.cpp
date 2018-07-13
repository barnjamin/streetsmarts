/*
 * watershed_marker_hist.h
 *
 * This sources implement the image segmentation using an unsupervised marker-controlled Watershed algorithm
 * with an over-segmentation reduction technique based on histograms, using OpenCV 3.X and C++ as described in this web page:
 * 		http://www.codeproject.com/Articles/751744/Image-Segmentation-using-Unsupervised-Watershed-Al

 *  Created on: April 20, 2016
 *      Author: claudiu
 */

#ifndef WATERSHED_MARKERS_HIST
#define WATERSHED_MARKERS_HIST
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <thread>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "watershed_utils.h"

using namespace std;
using namespace cv;


Mat createSegmentationDisplay(Mat & segments,int numOfSegments,Mat & image)
{
	//create a new image
	Mat wshed(segments.size(), CV_8UC3);

	//Create color tab for coloring the segments
	vector<Vec3b> colorTab;
	for(int i = 0; i < numOfSegments; i++ )
	{
		int b = theRNG().uniform(0, 255);
		int g = theRNG().uniform(0, 255);
		int r = theRNG().uniform(0, 255);

		colorTab.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
	}

	//assign different color to different segments
	for(int i = 0; i < segments.rows; i++ )
	{
		for(int j = 0; j < segments.cols; j++ )
		{
			int index = segments.at<int>(i,j);
			if( index == -1 )
				wshed.at<Vec3b>(i,j) = Vec3b(255,255,255);
			else if( index <= 0 || index > numOfSegments )
				wshed.at<Vec3b>(i,j) = Vec3b(0,0,0);
			else
				wshed.at<Vec3b>(i,j) = colorTab[index - 1];
		}
	}

	//If the original image available then merge with the colors of segments
	if(image.dims>0)
		wshed = wshed*0.5+image*0.5;

	return wshed;
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

void mergeSegments(Mat & image,Mat & segments, int & numOfSegments)
{
	//To collect pixels from each segment of the image
	vector<Mat> samples;
	//In case of multiple merging iterations, the numOfSegments should be updated
	int newNumOfSegments = numOfSegments;

	//Initialize the segment samples
	for(int i=0;i<=numOfSegments;i++)
	{
		Mat sampleImage;
		samples.push_back(sampleImage);
	}

	//collect pixels from each segments
	for(int i = 0; i < segments.rows; i++ )
	{
		for(int j = 0; j < segments.cols; j++ )
		{
			//check what segment the image pixel belongs to
			int index = segments.at<int>(i,j);
			if(index >= 0 && index<numOfSegments)
			{
				samples[index].push_back(image(Rect(j,i,1,1)));
			}
		}
	}

	//create histograms
	vector<MatND> hist_bases;
	Mat hsv_base;
	/// Using 35 bins for hue component
	int h_bins = 35;
	/// Using 30 bins for saturation component
	int s_bins = 30;
	int histSize[] = { h_bins,s_bins };

	// hue varies from 0 to 256, saturation from 0 to 180
	float h_ranges[] = { 0, 256 };
	float s_ranges[] = { 0, 180 };

	const float* ranges[] = { h_ranges, s_ranges };

	// Use the 0-th and 1-st channels
	int channels[] = { 0,1 };

	// To store the histograms
	MatND hist_base;
	for(int c=1;c<numOfSegments;c++)
	{
		if(samples[c].dims>0){
			//convert the sample to HSV
			cvtColor( samples[c], hsv_base, CV_BGR2HSV );
			//calculate the histogram
			calcHist( &hsv_base, 1, channels, Mat(), hist_base,2, histSize, ranges, true, false );
			//normalize the histogram
			normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
			//append to the collection
			hist_bases.push_back(hist_base);
		}else
		{
			hist_bases.push_back(MatND());
		}

		hist_base.release();
	}

	//To store the similarity of histograms
	double similarity = 0;

	//to keep the track of already merged segments
	vector<bool> mearged;

	//initialize the merged segments tracker
	for(int k = 0; k < hist_bases.size(); k++)
	{
		mearged.push_back(false);
	}

	//calculate the similarity of the histograms of each pair of segments
	for(int c=0;c<hist_bases.size();c++)
	{
		for(int q=c+1;q<hist_bases.size();q++)
		{
			//if the segment is not merged alreay
			if(!mearged[q])
			{
				if(hist_bases[c].dims>0 && hist_bases[q].dims>0)
				{
					//calculate the histogram similarity
					similarity = compareHist(hist_bases[c],hist_bases[q],CV_COMP_BHATTACHARYYA);
					//if similay
					if(similarity>0.8)
					{
						mearged[q]=true;
						if(q!=c)
						{
							//reduce number of segments
							newNumOfSegments--;
							for(int i = 0; i < segments.rows; i++ )
							{
								for(int j = 0; j < segments.cols; j++ )
								{
									int index = segments.at<int>(i,j);
									//merge the segment q with c
									if(index==q){
										segments.at<int>(i,j) = c;
									}
								}
							}
						}
					}
				}
			}
		}
	}
	numOfSegments = newNumOfSegments;
}

Mat watershedSegment(Mat & image, int & noOfSegments)
{
	//To store the gray version of the image
	Mat gray;
	//To store the thresholded image
	Mat ret;

	//convert the image to grayscale
	cvtColor(image,gray,CV_BGR2GRAY);

	//threshold the image
	threshold(gray,ret,0,255,CV_THRESH_BINARY_INV+CV_THRESH_OTSU);

	//Execute morphological-open
	morphologyEx(ret,ret,MORPH_OPEN,Mat::ones(9,9,CV_8SC1),Point(4,4),2);

	//get the distance transformation
	Mat distTransformed(ret.rows,ret.cols,CV_32FC1);
	distanceTransform(ret,distTransformed,CV_DIST_L2,3);

	//normalize the transformed image in order to display
	normalize(distTransformed, distTransformed, 0.0, 1, NORM_MINMAX);

	//threshold the transformed image to obtain markers for watershed
	threshold(distTransformed,distTransformed,0.1,1,CV_THRESH_BINARY);

	//Renormalize to 0-255 to further calculations
	normalize(distTransformed, distTransformed, 0.0, 255.0, NORM_MINMAX);
	distTransformed.convertTo(distTransformed,CV_8UC1);

	//calculate the contours of markers
	int compCount = 0;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(distTransformed, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	if( contours.empty() )
		return Mat();
	Mat markers(distTransformed.size(), CV_32S);
	markers = Scalar::all(0);
	int idx = 0;

	//draw contours
	for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
		drawContours(markers, contours, idx, Scalar::all(compCount+1), 3, 8, hierarchy, INT_MAX);

	if( compCount == 0 )
		return Mat();

	//calculate the time taken to the watershed algorithm
	double t = (double)getTickCount();

	//apply watershed with the markers as seeds
	watershed( image, markers );
	t = (double)getTickCount() - t;


	noOfSegments = compCount;

	//returns the segments
	return markers;
}

/**
 * This is an example method showing how to use this implementation.
 *
 * @param image The original image.
 * @return wshedWithImage A merged image of the original and the segments.
 */
Mat watershedWithMarkersAndHistograms(Mat image) {

	//to store the number of segments
	int numOfSegments = 0;

	//Apply watershed
	Mat segments = watershedSegment(image, numOfSegments);

	//Merge segments in order to reduce over segmentation
	mergeSegments(image, segments, numOfSegments);


	//To display the merged segments blended with the image
	Mat wshedWithImage = createSegmentationDisplay(segments, numOfSegments, image);

	return wshedWithImage;
}


const char* window_name = "Edge Map";
const int WIDTH = 640;
const int HEIGHT = 480;
rs2::frame_queue q;


// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    // Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::colorizer color_map;
    rs2::pipeline pipe;
	pipe.start();

    while(true){
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        auto depth = data.get_depth_frame();
        auto filtered_depth = depth;
        auto image = frame_to_mat(color_map(filtered_depth));

        //auto color = data.get_color_frame();
        //auto image = frame_to_mat(color);

        imshow("orig", image);

        // display the merged segments blended with the image
        Mat wshedWithImage = watershedWithMarkersAndHistograms(image);
        imshow("watershedded", wshedWithImage);

        waitKey(0);
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




#endif /* WATERSHED_MARKERS_HIST */