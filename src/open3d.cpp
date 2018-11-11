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

    Config conf;
    conf.parseArgs(argc, argv);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);

    Intrinsic i = get_intrinsics(profile);
    PinholeCameraIntrinsic intrinsic(i.width, i.height, 
        i.intrinsics.fx, i.intrinsics.fy, i.intrinsics.ppx, i.intrinsics.ppy);

    PinholeCameraIntrinsicCuda cudaint(intrinsic);

    ImageCuda<Vector1f> source_I, target_I, source_D, target_D;
    ImageCuda<Vector3b> source_O;


    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);

    //sigma, depth_near_thresh, depth_far_thresh, depth_diff_thresh
    odometry.SetParameters(0.2f, 0.1f, 8.0f, 0.05f);

    //Set empty transform
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

    float voxel_length = 0.02f;

    TransformCuda extrinsics = TransformCuda::Identity();

    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, 3 * voxel_length, extrinsics);

    open3d::Timer t;
    for(int i=0; i<100; i++){
        t.Start(); 
        rs2::frameset frameset = pipe.wait_for_frames();
        t.Stop();
        t.Print("Got Frame");

        t.Start();
        //Get processed aligned frame
        auto processed = align.process(frameset);

        // Trying to get both other and aligned depth frames
        rs2::video_frame color_frame = processed.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth_frame = processed.get_depth_frame();

        //If one of them is unavailable, continue iteration
        if (!depth_frame || !color_frame) {
            continue;
        }
        t.Stop();
        t.Print("Aligned");

        //Further process
        t.Start();
        auto depth = conf.filter(depth_frame);
        t.Stop();
        t.Print("Filtered");

        t.Start();
        Mat source_color = frame_to_mat(color_frame);
        Mat source_depth = frame_to_mat(depth);
        t.Stop();
        t.Print("Converted to Matrix");

        t.Start();
        source_O.Upload(source_color);
        
        cvtColor(source_color, source_color, cv::COLOR_BGR2GRAY);
        source_color.convertTo(source_color, CV_32FC1, 1.0f / 255.0f);
        source_depth.convertTo(source_depth, CV_32FC1, 0.001f);

        source_I.Upload(source_color);
        source_D.Upload(source_depth);
        t.Stop();
        t.Print("Converted and uploaded");

        //If we've been through our first loop
        if(i>0) {
            if(i == 1){
                t.Start();
                odometry.PrepareData(source_D, source_I, target_D, target_I);
                t.Stop();
                t.Print("Prepared");
            }   
            t.Start();
            odometry.Apply();
            t.Stop();
            t.Print("Applied");


            //std::cout<< "Transform: \n" << odometry.transform_source_to_target_ << std::endl;

            t.Start();
            TransformCuda tfc;
            tfc.FromEigen(odometry.transform_source_to_target_);

            auto src_rgbd = RGBDImageCuda(source_D, source_O);
            tsdf_volume.Integrate(src_rgbd, cudaint, tfc);
            t.Stop();
            t.Print("Integrated");
        }

        //Set current source to target
        target_I.CopyFrom(source_I);
        target_D.CopyFrom(source_D);
    }

    int appxsize = tsdf_volume.active_subvolume_entry_array().size();
    ScalableMeshVolumeCuda<8> mesher(appxsize * 100, VertexWithNormalAndColor, 
                                        appxsize*100, appxsize*200);
    mesher.active_subvolumes_ = appxsize;

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
