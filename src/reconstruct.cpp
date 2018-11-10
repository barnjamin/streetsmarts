#include <Core/Core.h>
#include <Geometry/Image.h>
#include <Geometry/RGBDImage.h>
#include <IO/IO.h>
#include <Core/Camera/PinholeCameraIntrinsic.h>

#include <Cuda/Odometry/RGBDOdometryCuda.h>

using namespace open3d;

int main(int argc, char * argv[])
{


    std::string base_path = "/home/nvidia/streetsmarts/python/latest/";

    PinholeCameraIntrinsic intrinsic;
    ReadIJsonConvertible(base_path + "intrinsic.json", intrinsic);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);
    odometry.SetParameters(0.2f, 0.1f, 8.0f, 0.01f);

    ImageCuda<Vector1f> source_I, target_I, source_D, target_D;

    //Image source_col, source_dep, target_col, target_dep;
    //ReadImage(base_path+"color/0.jpg", source_col);
    //ReadImage(base_path+"depth/0.png", source_dep);

    //ReadImage(base_path+"color/1.jpg", target_col);
    //ReadImage(base_path+"depth/1.png", target_dep);
    //source_I.Upload(source_col);
    //source_D.Upload(source_dep);

    //target_I.Upload(target_col);
    //target_D.Upload(target_dep);
    //auto src_rgbd = RGBDImageCuda(source_i, source_d);
    //auto tgt_rgbd = RGBDImageCuda(target_i, target_d);


    cv::Mat source_color = cv::imread(base_path + "color/0.jpg");
    cv::cvtColor(source_color, source_color, cv::COLOR_BGR2GRAY);
    source_color.convertTo(source_color, CV_32FC1, 1.0f / 255.0f);

    cv::Mat target_color = cv::imread(base_path + "color/1.jpg");
    cv::cvtColor(target_color, target_color, cv::COLOR_BGR2GRAY);
    target_color.convertTo(target_color, CV_32FC1, 1.0f / 255.0f);

    cv::Mat source_depth = cv::imread(base_path + "depth/0.png",
                                      cv::IMREAD_UNCHANGED);
    source_depth.convertTo(source_depth, CV_32FC1, 0.001f);

    cv::Mat target_depth = cv::imread(base_path + "depth/1.png",
                                      cv::IMREAD_UNCHANGED);
    target_depth.convertTo(target_depth, CV_32FC1, 0.001f);


    source_I.Upload(source_color);
    source_D.Upload(source_depth);

    target_I.Upload(target_color);
    target_D.Upload(target_depth);

    odometry.Build(source_D, source_I, target_D, target_I);
    odometry.Apply(source_D, source_I, target_D, target_I);

    std::cout<< "Transform: \n" << odometry.transform_source_to_target_ << std::endl;


    return EXIT_SUCCESS;
}
