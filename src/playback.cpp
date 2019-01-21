#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <Core/Core.h>
#include <IO/IO.h>
#include <Cuda/Odometry/RGBDOdometryCuda.h>
#include <Cuda/Integration/ScalableTSDFVolumeCuda.h>
#include <Cuda/Integration/ScalableMeshVolumeCuda.h>
#include <Visualization/Visualization.h>
#include <Core/Utility/Timer.h>
#include <opencv2/opencv.hpp>

#include <librealsense2/rs.hpp> 

#include <mutex>
#include <thread>
#include <GL/glut.h>

#include "utils.h" 
#include "pose.h"
#include "display.h"
#include "config.h"

using namespace std;
using namespace open3d;
using namespace open3d::cuda;
using namespace cv;

int main(int argc, char * argv[]) try
{
    string dirname =  "/home/ben/streetsmarts/dumps/latest"; 

    Config conf;
    conf.parseArgs(argc, argv);
    //Config conf(dirname + "/conf.json");


    PinholeCameraIntrinsic intrinsics;
    ReadIJsonConvertible(dirname + "/intrinsic.json", intrinsics);
    PinholeCameraIntrinsicCuda cuda_intrinsics(intrinsics);

    PinholeCameraTrajectory trajectory;

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsics);
    odometry.SetParameters(OdometryOption());
    odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

    float voxel_length = 0.01f;
    TransformCuda extrinsics = TransformCuda::Identity();
    ScalableTSDFVolumeCuda<8> tsdf_volume(10000, 200000, voxel_length, 
            3 * voxel_length, extrinsics);

    auto depth_image_ptr = make_shared<Image>();
    auto color_image_ptr = make_shared<Image>();
    depth_image_ptr->PrepareImage(conf.width, conf.height, 1, 2);
    color_image_ptr->PrepareImage(conf.width, conf.height, 3, 1);

    RGBDImageCuda rgbd_prev(0.1f, 4.0f, 1000.0f);
    RGBDImageCuda rgbd_curr(0.1f, 4.0f, 1000.0f);

    ScalableMeshVolumeCuda<8> mesher(100000, VertexWithNormalAndColor, 10000000, 20000000);

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();
    
    //FPSTimer timer("Process RGBD stream", conf.frames);

    int save_index = 0;
    
    //Visualizer visualizer;
    //if (!visualizer.CreateVisualizerWindow("Sequential IC RGBD Odometry", 1920, 1080, 0, 0)) {
    //    PrintWarning("Failed creating OpenGL window.\n");
    //    return -1;
    //}
    //visualizer.BuildUtilities();
    //visualizer.UpdateWindowTitle();

    //std::shared_ptr<TriangleMeshCuda> mesh = std::make_shared<TriangleMeshCuda>();
    //std::shared_ptr<PointCloud> imu_pc = std::make_shared<PointCloud>();
    //std::shared_ptr<PointCloud> odom_pc = std::make_shared<PointCloud>();
    //std::shared_ptr<PointCloud> plain_pc = std::make_shared<PointCloud>();

    //visualizer.AddGeometry(imu_pc);
    //visualizer.AddGeometry(odom_pc);
    //visualizer.AddGeometry(plain_pc);

    //ViewParameters vp;
    //vp.boundingbox_max_ = Eigen::Vector3d(10,10,10); 
    //vp.boundingbox_min_ = Eigen::Vector3d(0,0,0); 
    //vp.field_of_view_ = 60; 
    //vp.front_ = Eigen::Vector3d(0,0,-1); 
    //vp.lookat_ = Eigen::Vector3d(1,0,0); 
    //vp.up_ = Eigen::Vector3d(0,-1,0); 
    //vp.zoom_ = 0.5;

    //visualizer.GetViewControl().ConvertFromViewParameters(vp);

    ofstream fout;
    if(conf.write_losses){
        string log_filename = "odometry_less_assoc_step_raw.log";
        if(conf.use_imu){
            log_filename = "odometry_less_assoc_step_imu.log";
        }

        fout  = ofstream(log_filename);
        if (!fout.is_open()) {
            PrintError("Unable to write to log file %s, abort.\n", log_filename.c_str());
        }
    }


    ifstream imu_file(dirname + "/imu.csv");

    Pose pose(30);
    Timer t;
    double duration = 0.0;
    int i = 0;

    bool success;
    Eigen::Matrix4d delta;
    std::vector<std::vector<float>> losses;

    while(imu_file){
        string s;
        if(!getline(imu_file, s)) break;

        istringstream ss( s );
        vector <string> record;

        while(ss) {
            string s;
            if (!getline( ss, s, ',' )) break;
            record.push_back( s );
        }

        auto idx = record.at(0);
        auto ts = atof(record.at(1).c_str());

        i = atoi(idx.c_str());


        //if(i>300) { break; }

        vector<double> accel{atof(record.at(2).c_str()), atof(record.at(3).c_str()), atof(record.at(4).c_str())} ;
        vector<double> gyro{atof(record.at(5).c_str()), atof(record.at(6).c_str()), atof(record.at(7).c_str())} ;


        pose.Update(accel, gyro, ts/1000);
        
        ReadImage(dirname+"/color/"+idx+".jpg", *color_image_ptr);
        ReadImage(dirname+"/depth/"+idx+".png", *depth_image_ptr);

        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        if (i == 0 ) {
            rgbd_prev.CopyFrom(rgbd_curr);
            continue;
        }
        
        Eigen::Matrix4d tranny = Eigen::Matrix4d::Identity();
        if(conf.use_imu){
            tranny = pose.GetTransform();
        }
        odometry.transform_source_to_target_ =  tranny;

        odometry.Initialize(rgbd_curr, rgbd_prev);

        std::tie(success, delta, losses) = odometry.ComputeMultiScale();

        if (conf.write_losses) {
            WriteLossesToLog(fout, i, losses);
        }

        //Compare Pose Suggested 

        if(!success) {

        /*

            When we've computed transform, animate transformation onto other image

            compare transform suggested with odom computed where pose && computed differ by a lot

            compute distance in translation and quaterion difference from odom pose
            
            approve/decline
            
        */
            
            rgbd_prev.CopyFrom(rgbd_curr);
            continue;
        }

        double qd, td, vd;
        std::tie(qd, td, vd) = pose.Difference(odometry.transform_source_to_target_);

        //pose.PrintState();
        //std::cout << i << "," << qd << ","<< td << "," << vd << std::endl;

        //Draw Estimated Pose transform vs Odom transform 
         
        //std::shared_ptr<RGBDImage> curr = CreateRGBDImageFromColorAndDepth(*depth_image_ptr, *color_image_ptr);

        //auto imu_pc = CreatePointCloudFromRGBDImage(*curr, intrinsics, tranny);
        //imu_pc->PaintUniformColor(Eigen::Vector3d(0.5,0,0));

        //auto odom_pc = CreatePointCloudFromRGBDImage(*curr, intrinsics, odometry.transform_source_to_target_);
        //odom_pc->PaintUniformColor(Eigen::Vector3d(0,0,0.5));

        //auto plain_pc = CreatePointCloudFromRGBDImage(*curr, intrinsics, Eigen::Matrix4d::Identity());
        //plain_pc->PaintUniformColor(Eigen::Vector3d(0,0.5,0));
        //DrawGeometries({imu_pc, odom_pc, plain_pc});

        //visualizer.PollEvents();
        //visualizer.UpdateGeometry(); 


        //std::cout << qd  << std::endl;
        //std::cout << td  << std::endl;
        //std::cout << vd  << std::endl;

        target_to_world = target_to_world * odometry.transform_source_to_target_;

        //Reset Quaternion using odometry values
        if(conf.use_imu){
            pose.Improve(odometry.transform_source_to_target_, target_to_world);
        }

        extrinsics.FromEigen(target_to_world);
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);


        //if (i > 0 && int(i % conf.fps/2) == 0) {
        //    //std::cout << "Frame: " << i << std::endl << std::endl << std::endl;

        //    //tsdf_volume.GetAllSubvolumes();

        //    //mesher.MarchingCubes(tsdf_volume);
        //    //*mesh = mesher.mesh();
        //    //visualizer.PollEvents();
        //    //visualizer.UpdateGeometry(); 

        //    //WriteTriangleMeshToPLY( "fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());
        //    
        //    //tsdf_volume.Reset();

        //    //pose.Reset();

        //    //save_index++;
        //}

        rgbd_prev.CopyFrom(rgbd_curr);

        PinholeCameraParameters params;
        params.intrinsic_ =  intrinsics;
        params.extrinsic_ = target_to_world;
        trajectory.parameters_.emplace_back(params);

        //timer.Signal();
    }

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);


    std::string suffix = "no-imu";
    if(conf.use_imu){ 
        suffix = "imu";
    }

    WriteTriangleMeshToPLY("fragment-"+suffix+".ply", *mesher.mesh().Download()); 

    //WritePinholeCameraTrajectoryToLOG("trajectory-"+suffix+".log", trajectory);
    //WritePinholeCameraTrajectory("trajectory-"+suffix+".log", trajectory);
    //WriteIJsonConvertible("pose-graph-"+suffix+".json", pose.GetGraph());
    //WriteIJsonConvertable("all-graph-"+suffix+".json", pose.GetGraph());
    
    return EXIT_SUCCESS;

} catch (const exception& e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}
