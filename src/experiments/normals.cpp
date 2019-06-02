#include <Open3D/Open3D.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/rgbd.hpp>

#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h>

#include <iostream>
#include "config.h"
#include "utils.h"

using namespace cv;
using namespace cv::rgbd;
using namespace std;

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int main( int argc, char** argv )
{
    Config conf;
    conf = Config(argc, argv);

    rs2::pipeline pipeline;
    rs2::config cfg;

    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);

    rs2::pipeline_profile pipeline_profile = pipeline.start(cfg);

    auto intrinsic_ = get_open3d_intrinsic(pipeline_profile);
    auto f = intrinsic_.GetFocalLength();
    auto p = intrinsic_.GetPrincipalPoint();
    Mat K = (Mat1d(3, 3) << f.first, 0, p.first, 0, f.second, p.second, 0, 0, 1);
    RgbdNormals norms(intrinsic_.height_, intrinsic_.width_, CV_64F, K, 3, 0);

    for(int i=0; i<conf.framestart; i++) pipeline.wait_for_frames(); 

    namedWindow("Norm", WINDOW_AUTOSIZE );
    namedWindow("Orig", WINDOW_AUTOSIZE );
    namedWindow("Blurred", WINDOW_AUTOSIZE );

    for(int i=0; i<conf.frames; i++){
        rs2::frameset frameset = pipeline.wait_for_frames();
        rs2::depth_frame depth_frame = frameset.get_depth_frame();

        auto depth = frame_to_mat(depth_frame);

        std::cout << type2str(depth.type()) << std::endl;

        cvtColor(depth, depth, COLOR_GRAY2BGR);
        depth.convertTo(depth, CV_64F);

        //Mat blurred;
        //blur(depth, blurred, Size(5,5), Point(-1,-1)); 
        //bilateralFilter(depth, blurred, 3, 6, 1.5); 
        //GaussianBlur(depth, blurred, Size(5,5), 0,0);
        //medianBlur(depth, blurred, 5);

        Mat normals;
        norms(depth, normals);

        imshow("Norm", normals);                   
        imshow("Orig", depth);                   
        //imshow("Blurred", blurred);                   

        waitKey(10);                                          
    }
    return 0;
}


//int main( int argc, char** argv )
//{
//    Config conf;
//    
//    // Assume json
//    if(argc==2){
//        std::string config_path = argv[1];
//        if(!open3d::io::ReadIJsonConvertible(config_path, conf)) {
//            open3d::utility::PrintError("Failed to read config\n");
//            return 1;
//        }
//    }else{
//        conf = Config(argc, argv);
//    }
//
//    open3d::camera::PinholeCameraIntrinsic intrinsic_;
//    if(!open3d::io::ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
//        return 1;
//    }
//
//    Mat K = GetMatrixFromIntrinsic(intrinsic_);
//
//    RgbdNormals norms(intrinsic_.height_, intrinsic_.width_, CV_64F, K, 7, 0);
//
//    namedWindow("Orig", WINDOW_AUTOSIZE );
//    namedWindow("Norm", WINDOW_AUTOSIZE );
//    for(int i=0; i<conf.frames; i++){
//        Mat color, depth;
//        color = imread(conf.ColorFile(i));   
//        depth = imread(conf.DepthFile(i), -1);  
//
//        //blur(depth, depth, Size(3,3));
//
//        cvtColor(depth, depth, COLOR_GRAY2BGR);
//        depth.convertTo(depth, CV_64F);
//
//        Mat normals;
//        norms(depth, normals);
//        //std::cout << type2str(normals.type()) << std::endl;
//
//        Mat graynorm;
//        normals.convertTo(graynorm, CV_32F);
//        cvtColor(graynorm, graynorm, COLOR_BGR2GRAY);
//        //std::cout << type2str(graynorm.type()) << std::endl;
//
//        double minVal, maxVal;
//        minMaxLoc(graynorm, &minVal, &maxVal); //find minimum and maximum intensities
//        graynorm.convertTo(graynorm, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
//
//        //Mat canny;
//        //Canny(graynorm, canny, 50, 100, 3);
//
//        imshow("Orig", color);                   
//        imshow("Norm", graynorm);
//
//        waitKey(5);
//    }
//    return 0;
//}
