#include <Core/Core.h>
#include <Visualization/Visualization.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Cuda/Common/TransformCuda.h>
#include <Cuda/Geometry/TriangleMeshCuda.h>
#include <Cuda/Geometry/RGBDImageCuda.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>
#include <Cuda/Common/VectorCuda.h>
#include <Core/Core.h>
#include <Eigen/Eigen>
#include <IO/IO.h>
#include <vector>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp> 
#include "utils.h" 





using namespace open3d;
using namespace cv;

int main(int argc, char * argv[]) try
{

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile selection = pipe.start(cfg);
    auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                                 .as<rs2::video_stream_profile>();

    auto resolution = std::make_pair(depth_stream.width(), depth_stream.height());
    auto i = depth_stream.get_intrinsics();
    auto principal_point = std::make_pair(i.ppx, i.ppy);
    auto focal_length = std::make_pair(i.fx, i.fy);

    PinholeCameraIntrinsic intrinsic(
        resolution.first, resolution.second, 
        focal_length.first, focal_length.second,
        principal_point.first, principal_point.second);


    Timer timer;

    ImageCuda<Vector1f> source_I, target_I, source_D, target_D;
    ImageCuda<Vector3b> source_o;

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);
    odometry.SetParameters(0.2f, 0.1f, 4.0f, 0.05f);

    //Get initial frame
    rs2::frameset data = pipe.wait_for_frames();
    rs2::frame depth = data.get_depth_frame();
    rs2::frame color = data.get_color_frame();

    Mat target_color = frame_to_mat(color);
    cvtColor(target_color, target_color, cv::COLOR_BGR2GRAY);
    target_color.convertTo(target_color, CV_32FC1, 1.0f / 255.0f);

    Mat target_depth = frame_to_mat(depth);
    target_depth.convertTo(target_depth, CV_32FC1, 0.001f);

    target_I.Upload(target_color);
    target_D.Upload(target_depth);

    float voxel_length = 0.01f;
    TransformCuda extrinsics = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, 3 * voxel_length, extrinsics);


    PinholeCameraIntrinsicCuda cudaint(intrinsic);

    bool initialized = false;
    for(int i=0; i<100; i++){

        data = pipe.wait_for_frames(); 

        Mat source_color = frame_to_mat(data.get_color_frame());
        Mat src_orig = source_color.clone();
        cvtColor(source_color, source_color, cv::COLOR_BGR2GRAY);
        source_color.convertTo(source_color, CV_32FC1, 1.0f / 255.0f);

        Mat source_depth = frame_to_mat(data.get_depth_frame());
        source_depth.convertTo(source_depth, CV_32FC1, 0.001f);

        source_o.Upload(src_orig);

        source_I.Upload(source_color);
        source_D.Upload(source_depth);

        //Reset transform
        odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

        if(!initialized) {
            odometry.Build(source_D, source_I, target_D, target_I);
            initialized = true;
        }

        odometry.Apply(source_D, source_I, target_D, target_I);

        //std::cout<< "Transform: \n" << odometry.transform_source_to_target_ << std::endl;

        TransformCuda tfc;
        tfc.FromEigen(odometry.transform_source_to_target_);

        //Set current source to target
        target_I.CopyFrom(source_I);
        target_D.CopyFrom(source_D);

        auto src_rgbd = RGBDImageCuda(source_D, source_o);
        tsdf_volume.Integrate(src_rgbd, cudaint, tfc);
    }


    ScalableMeshVolumeCuda<8> mesher(10000, VertexWithNormalAndColor, 100000, 200000);
    mesher.active_subvolumes_ = tsdf_volume.active_subvolume_entry_array().size();
    PrintInfo("Active subvolumes: %d\n", mesher.active_subvolumes_);

    mesher.MarchingCubes(tsdf_volume);

    //std::shared_ptr<TriangleMeshCuda> mesh = std::make_shared<TriangleMeshCuda>(mesher.mesh());
    //std::shared_ptr<TriangleMesh> meshcpu = mesh->Download();
    auto mesh = mesher.mesh().Download();
    PrintInfo("MaxBound: %s\n", mesh->GetMaxBound());
    PrintInfo("MinBound: %s\n", mesh->GetMinBound());
    
    //mesh->ComputeVertexNormals();

    WriteTriangleMeshToPLY("wtf.ply", *mesh, true);

    return EXIT_SUCCESS;

} catch (const rs2::error & e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}