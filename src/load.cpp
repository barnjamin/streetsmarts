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

using namespace std;

mutex mtx;

rs2_vector accel;
rs2_vector gyro;

Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
Eigen::Quaterniond last_q(1.0, 0.0, 0.0, 0.0);

using namespace open3d;
using namespace open3d::cuda;
using namespace cv;

void WriteLossesToLog(
    ofstream &fout,
    int frame_idx,
    vector<vector<float>> &losses) {
    assert(fout.is_open());

    fout << frame_idx << "\n";
    for (auto &losses_on_level : losses) {
        for (auto &loss : losses_on_level) {
            fout << loss << " ";
        }
        fout << "\n";
    }
}


int main(int argc, char * argv[]) try
{
    Config conf;
    conf.parseArgs(argc, argv);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align(RS2_STREAM_COLOR);

    cfg.enable_stream(RS2_STREAM_DEPTH, conf.width, conf.height, RS2_FORMAT_Z16, conf.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, conf.width, conf.height, RS2_FORMAT_BGR8, conf.fps);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);

    rs2::pipeline_profile profile = pipe.start(cfg);

    PinholeCameraIntrinsic intrinsics = get_intrinsics(profile);
    PinholeCameraIntrinsicCuda cuda_intrinsics(intrinsics);

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

    ScalableMeshVolumeCuda<8> mesher(40000, VertexWithNormalAndColor, 6000000, 12000000);

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();
    
    FPSTimer timer("Process RGBD stream", conf.frames);

    int save_index = 0;
    
    string log_filename = "odometry_less_assoc_step_1.log";
    ofstream fout(log_filename);
    if (!fout.is_open()) {
        PrintError("Unable to write to log file %s, abort.\n", log_filename.c_str());
    }

    string dirname =  "dumps/20190105102137"; //get_latest_dump_dir();
    ifstream imufile(dirname + "/imu.csv");

    Pose pose(30);
    Display display(argc, argv, &pose);
    display.start();
    while(imufile){
        string s;
        if(!getline(imufile, s)) break;

        istringstream ss( s );
        vector <string> record;

        while(ss) {
            string s;
            if (!getline( ss, s, ',' )) break;
            record.push_back( s );
        }

        auto idx = record.at(0);

        int i = atoi(idx.c_str());

        vector<double> accel{atof(record.at(1).c_str()), atof(record.at(2).c_str()), atof(record.at(3).c_str())} ;
        vector<double> gyro{atof(record.at(4).c_str()), atof(record.at(5).c_str()), atof(record.at(6).c_str())} ;

        pose.Update(accel, gyro);

        
        ReadImage(dirname+"/color/"+idx+".jpg", *color_image_ptr);
        ReadImage(dirname+"/depth/"+idx+".png", *depth_image_ptr);

        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        q = pose.GetOrientation();
        if (i > 0 ) {
            Eigen::Quaterniond diff = q * last_q.inverse();
            Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(Eigen::Vector3d(0,0,0)) * (diff.normalized().toRotationMatrix());
            odometry.transform_source_to_target_ = t.matrix();

            //odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

            odometry.Initialize(rgbd_curr, rgbd_prev);

            auto result = odometry.ComputeMultiScale();
            if (get<0>(result)) {
                WriteLossesToLog(fout, i, get<2>(result));
            }

            target_to_world = target_to_world * odometry.transform_source_to_target_;

            //Reset Quaternion using odometry values
            pose.Improve(odometry.transform_source_to_target_);

            last_q = pose.GetOrientation();
        }

        extrinsics.FromEigen(target_to_world);
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);

        rgbd_prev.CopyFrom(rgbd_curr);
        timer.Signal();
    }

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);

    WriteTriangleMeshToPLY("fragment-" + to_string(save_index) + ".ply", *mesher.mesh().Download());
    
    display.stop();

    return EXIT_SUCCESS;

} catch (const exception& e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}
