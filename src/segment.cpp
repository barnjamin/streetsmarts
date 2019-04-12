#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/rgbd.hpp>

#include <opencv2/ximgproc.hpp>

#include <ctype.h>
#include <stdio.h>
#include <iostream>

#include <stdlib.h>     /* srand, rand */

#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h>

#include "config.h"
#include "utils.h"

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

static const char* window_name = "SLIC Superpixels";

static const char* keys =
    "{h help      | | help menu}"
    "{c camera    |0| camera id}"
    "{i image     | | image file}"
    "{a algorithm |1| SLIC(0),SLICO(1),MSLIC(2)}"
    ;


std::vector<std::vector<Point>> SampleSubpixels(Mat& I, int cnt) {
    int channels = I.channels();

    int nRows = I.rows;
    int nCols = I.cols * channels;

    if (I.isContinuous())
    {
        nCols *= nRows;
        nRows = 1;
    }

    std::vector<std::vector<Point>> groups; 
    groups.resize(cnt);

    std::vector<std::vector<Point>> samples;
    samples.resize(cnt);
    
    int i,j;
    uint* p;
    for(i = 0; i < nRows; ++i) {
        p = I.ptr<uint>(i);
        for ( j = 0; j < nCols; ++j) {
            groups[p[j]].push_back(Point(i,j)); 
        }
    }

    for(int x=0; x<groups.size(); ++x){
        auto sz = groups[x].size();
        if(sz==0){
            continue;
        }

        for(int s = 0; s<20; s++){
            int idx = rand() % sz ;
            samples[x].push_back(groups[x][idx]);
        }
    }

    return samples;
}

Mat ComputeNormals(Mat depth){
    Mat normals(depth.size(), CV_32FC3);

    std::cout << "Computing normals" << std::endl;
    for(int x = 0; x < depth.rows; ++x) {
        for(int y = 0; y < depth.cols; ++y) {
            float dzdx = (depth.at<float>(x+1, y) - depth.at<float>(x-1, y)) / 2.0;
            float dzdy = (depth.at<float>(x, y+1) - depth.at<float>(x, y-1)) / 2.0;

            Vec3f d(-dzdx, -dzdy, 1.0f);
            Vec3f n = normalize(d);

            normals.at<Vec3f>(x, y) = n;
        }
    }
    std::cout << "Done" << std::endl;

    return normals;
}



int main(int argc, char** argv)
{

    Config conf(argc, argv);

    rs2::pipeline pipeline;
    rs2::config cfg;
    rs2::colorizer color_map;

    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);

    pipeline.start(cfg);

    for(int i=0; i<conf.framestart; i++) pipeline.wait_for_frames(); 


    int algorithm = 0;

    int region_size = 50;
    int ruler = 30;
    int min_element_size = 50;
    int num_iterations = 3;

    namedWindow(window_name, 0);
    createTrackbar("Algorithm", window_name, &algorithm, 2, 0);
    createTrackbar("Region size", window_name, &region_size, 200, 0);
    createTrackbar("Ruler", window_name, &ruler, 100, 0);
    createTrackbar("Connectivity", window_name, &min_element_size, 100, 0);
    createTrackbar("Iterations", window_name, &num_iterations, 12, 0);

    Mat result, mask;
    int display_mode = 0;

    Mat K = (Mat_<double>(3, 3) << 610.0023193359375, 0.0, 425.36004638671875, 0.0, 609.85760498046875, 237.9273681640625, 0.0, 0.0, 1.0);

    auto rn = rgbd::RgbdNormals(conf.height, conf.width, CV_32F, K);
    rn.initialize();

    for (;;)
    {

        auto frameset = pipeline.wait_for_frames();

        auto fs = align.process(frameset.as<rs2::frameset>());

        Mat frame = frame_to_mat(fs.first(RS2_STREAM_COLOR));
        Mat df = frame_to_mat(fs.get_depth_frame());

        Mat fdf;
        df.convertTo(fdf, CV_32FC1);
        Mat norm = ComputeNormals(fdf);
        imshow(window_name, norm);
        
        int c = waitKey(1) & 0xff;
        if( c == 'q' || c == 'Q' || c == 27 )
            break;
        else if( c == ' ' )
            display_mode = (display_mode + 1) % 2;

        //Mat cdf = frame_to_mat(color_map.process(fs.get_depth_frame()));

        //result = frame;
        //result = frame;
        //Mat converted;
        //cvtColor(frame, converted, COLOR_BGR2HSV);

        //double t = (double) getTickCount();

        //Ptr<SuperpixelSLIC> slic = createSuperpixelSLIC(converted,algorithm+SLIC, region_size, float(ruler));
        //slic->iterate(num_iterations);
        //if (min_element_size>0)
        //    slic->enforceLabelConnectivity(min_element_size);

        //t = ((double) getTickCount() - t) / getTickFrequency();
        //cout << "SLIC" << (algorithm?'O':' ')
        //     << " segmentation took " << (int) (t * 1000)
        //     << " ms with " << slic->getNumberOfSuperpixels() << " superpixels" << endl;

        //// get the contours for displaying
        //slic->getLabelContourMask(mask, true);
        //result.setTo(Scalar(0, 0, 255), mask);

        //// display output
        //switch (display_mode)
        //{
        //case 0: //superpixel contours
        //    imshow(window_name, result);
        //    break;
        //case 1: //labels array
        //{
        //    Mat labels;
        //    slic->getLabels(labels);
        //    auto samples = SampleSubpixels(labels, slic->getNumberOfSuperpixels());
        //    for(auto &grp: samples){
        //        for(auto &sample: grp){
        //            std::cout << sample << std::endl; 
        //        } 
        //    }
        //    //imshow(window_name, labels);
        //    break;
        //}
        //}

        //int c = waitKey(1) & 0xff;
        //if( c == 'q' || c == 'Q' || c == 27 )
        //    break;
        //else if( c == ' ' )
        //    display_mode = (display_mode + 1) % 2;
    }

    return 0;
}
