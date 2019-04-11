#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/rgbd.hpp>

#include <opencv2/ximgproc.hpp>

#include <ctype.h>
#include <stdio.h>
#include <iostream>

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
        //Mat df = frame_to_mat(fs.get_depth_frame());
        //Mat cdf = frame_to_mat(color_map.process(fs.get_depth_frame()));

        //result = frame;
        result = frame;
        Mat converted;
        cvtColor(frame, converted, COLOR_BGR2HSV);

        double t = (double) getTickCount();

        Ptr<SuperpixelSLIC> slic = createSuperpixelSLIC(converted,algorithm+SLIC, region_size, float(ruler));
        slic->iterate(num_iterations);
        if (min_element_size>0)
            slic->enforceLabelConnectivity(min_element_size);

        t = ((double) getTickCount() - t) / getTickFrequency();
        cout << "SLIC" << (algorithm?'O':' ')
             << " segmentation took " << (int) (t * 1000)
             << " ms with " << slic->getNumberOfSuperpixels() << " superpixels" << endl;

        // get the contours for displaying
        slic->getLabelContourMask(mask, true);
        result.setTo(Scalar(0, 0, 255), mask);

        // display output
        switch (display_mode)
        {
        case 0: //superpixel contours
            imshow(window_name, result);
            break;
        case 1: //mask
            imshow(window_name, mask);
            break;
        case 2: //labels array
        {
            // use the last x bit to determine the color. Note that this does not
            // guarantee that 2 neighboring superpixels have different colors.
            // retrieve the segmentation result
            Mat labels;
            slic->getLabels(labels);
            const int num_label_bits = 2;
            labels &= (1 << num_label_bits) - 1;
            labels *= 1 << (16 - num_label_bits);
            imshow(window_name, labels);
            break;
        }
        }

        int c = waitKey(1) & 0xff;
        if( c == 'q' || c == 'Q' || c == 27 )
            break;
        else if( c == ' ' )
            display_mode = (display_mode + 1) % 3;
    }

    return 0;
}
