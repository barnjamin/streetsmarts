#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/rgbd.hpp>

#include <Eigen/Geometry>
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



Mat AvgNormals(const Mat &normals, const Mat &labeled, int cnt) {

    std::vector<std::vector<Point>> groups; 
    groups.resize(cnt);

    //Create groups
    for(int i = 0; i < labeled.rows; ++i) {
        for (int j = 0; j < labeled.cols; ++j) {
            groups[labeled.at<int>(i, j)].push_back(Point(i,j)); 
        }
    }

    Mat avgd(normals.size(), CV_32FC3);
    for(int i=0; i<groups.size(); ++i){
        Vec3f a(0,0,0);
        for(int j=0; j<groups[i].size(); j++){
            auto px = groups[i][j];
            auto n = normals.at<Vec3f>(px.x, px.y);
            a[0] += abs(n[0]);
            a[1] += abs(n[1]);
            a[2] += abs(n[2]);
        }
        a /= (float)groups[i].size();
        a = normalize(a);

        for(int j=0; j<groups[i].size(); j++){
            auto px = groups[i][j];
            avgd.at<Vec3f>(px.x, px.y) = a;
        }
    }

    return avgd;
}

Mat ComputeNormals(Mat depth){
    Mat normals(depth.size(), CV_32FC3);

    int size = 3;
    for(int x = 0; x < depth.rows; ++x) {
        for(int y = 0; y < depth.cols; ++y) {
            float dzdx = (depth.at<float>(x+size, y) - depth.at<float>(x-size, y)) / 2.0;
            float dzdy = (depth.at<float>(x, y+size) - depth.at<float>(x, y-size)) / 2.0;

            Vec3f d(-dzdx, -dzdy, 1.0f);
            Vec3f n = normalize(d);

            normals.at<Vec3f>(x, y) = n;
        }
    }

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


    for (;;)
    {

        auto frameset = pipeline.wait_for_frames();
        double t = (double) getTickCount();

        auto fs = align.process(frameset.as<rs2::frameset>());

        Mat frame = frame_to_mat(fs.first(RS2_STREAM_COLOR));
        Mat df = frame_to_mat(fs.get_depth_frame());

        Mat fdf;
        df.convertTo(fdf, CV_32FC1);

        Mat norms = ComputeNormals(fdf);

        result = frame;
        Mat converted;
        cvtColor(frame, converted, COLOR_BGR2HSV);


        Ptr<SuperpixelSLIC> slic = createSuperpixelSLIC(converted,algorithm+SLIC, region_size, float(ruler));
        slic->iterate(num_iterations);
        if (min_element_size>0)
            slic->enforceLabelConnectivity(min_element_size);


        // get the contours for displaying
        slic->getLabelContourMask(mask, true);
        result.setTo(Scalar(0, 0, 255), mask);

        Mat labels;
        slic->getLabels(labels);
        //imshow(window_name, result);

        auto r = AvgNormals(norms, labels, slic->getNumberOfSuperpixels());
        imshow(window_name, r);

        t = ((double) getTickCount() - t) / getTickFrequency();
        cout << "SLIC" << (algorithm?'O':' ')
             << " segmentation took " << (int) (t * 1000)
             << " ms with " << slic->getNumberOfSuperpixels() << " superpixels" << endl;


        int c = waitKey(1) & 0xff;
        if( c == 'q' || c == 'Q' || c == 27 )
            break;
        else if( c == ' ' )
            display_mode = (display_mode + 1) % 2;
    }

    return 0;
}
