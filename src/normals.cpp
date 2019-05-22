#include <Open3D/Open3D.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/rgbd.hpp>

#include <iostream>
#include "config.h"

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
    
    // Assume json
    if(argc==2){
        std::string config_path = argv[1];
        if(!open3d::io::ReadIJsonConvertible(config_path, conf)) {
            open3d::utility::PrintError("Failed to read config\n");
            return 1;
        }
    }else{
        conf = Config(argc, argv);
    }

    open3d::camera::PinholeCameraIntrinsic intrinsic_;
    if(!open3d::io::ReadIJsonConvertible(conf.IntrinsicFile(), intrinsic_)){
        return 1;
    }
    auto f = intrinsic_.GetFocalLength();
    auto p = intrinsic_.GetPrincipalPoint();
    Mat K = (Mat1d(3, 3) << f.first, 0, p.first, 0, f.second, p.second, 0, 0, 1);
    RgbdNormals norms(intrinsic_.height_, intrinsic_.width_, CV_32F, K, 3, 0);

    namedWindow("Norm", WINDOW_AUTOSIZE );
    namedWindow("Orig", WINDOW_AUTOSIZE );
    namedWindow("Blurred", WINDOW_AUTOSIZE );

    for(int i=0; i<conf.frames; i++){
        Mat color, depth;
        color = imread(conf.ColorFile(i));   
        depth = imread(conf.DepthFile(i));  
        //std::cout << type2str(depth.type()) << std::endl;

        depth.convertTo(depth, CV_32F);

        Mat blurred;
        blur(depth, blurred, Size(5,5), Point(-1,-1)); 
        //bilateralFilter(depth, blurred, 3, 6, 1.5); 
        //GaussianBlur(depth, blurred, Size(5,5), 0,0);
        //medianBlur(depth, blurred, 5);

        Mat normals;
        norms(blurred, normals);

        imshow("Norm", normals);                   
        imshow("Orig", color);                   
        imshow("Blurred", blurred);                   

        waitKey(10);                                          
    }
    return 0;
}



