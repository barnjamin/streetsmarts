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
#include "utils.h" 

#include "MadgwickAHRS.h" // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

using namespace open3d;
using namespace open3d::cuda;
using namespace cv;

void WriteLossesToLog(
    std::ofstream &fout,
    int frame_idx,
    std::vector<std::vector<float>> &losses) {
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

    auto depth_image_ptr = std::make_shared<Image>();
    auto color_image_ptr = std::make_shared<Image>();
    depth_image_ptr->PrepareImage(conf.width, conf.height, 1, 2);
    color_image_ptr->PrepareImage(conf.width, conf.height, 3, 1);

    RGBDImageCuda rgbd_prev(0.1f, 4.0f, 1000.0f);
    RGBDImageCuda rgbd_curr(0.1f, 4.0f, 1000.0f);

    ScalableMeshVolumeCuda<8> mesher(40000, VertexWithNormalAndColor, 6000000, 12000000);

    Eigen::Matrix4d target_to_world = Eigen::Matrix4d::Identity();
    for(int i=0; i<conf.framestart; i++){
        rs2::frameset frameset = pipe.wait_for_frames();
    }
    
    FPSTimer timer("Process RGBD stream", conf.frames);

    int save_index = 0;
    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2_vector accel_data, gyro_data;


    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);
    Eigen::Quaterniond last_q(1.0, 0.0, 0.0, 0.0);
    
    std::string log_filename = "odometry_less_assoc_step_" + std::to_string(1) + ".log";
    std::ofstream fout(log_filename);
    if (!fout.is_open()) {
        PrintError("Unable to write to log file %s, abort.\n", log_filename.c_str());
    }


    PrintInfo("Starting to read frames, reading %d frames\n", conf.frames);
    for(int i=0; i< conf.frames; i++){
        frameset = pipe.wait_for_frames();

        //Get processed aligned frame
        //frameset = align.process(frameset);

        // Trying to get both other and aligned depth frames
        color_frame = frameset.first(RS2_STREAM_COLOR);
        depth_frame = frameset.get_depth_frame();

        auto accel_frame = frameset.first(RS2_STREAM_ACCEL).as<rs2::motion_frame>();
        auto gyro_frame  = frameset.first(RS2_STREAM_GYRO).as<rs2::motion_frame>();

        accel_data = accel_frame.get_motion_data();
        gyro_data  = gyro_frame.get_motion_data();

        if (!depth_frame || !color_frame) { continue; }

        //depth_frame = conf.filter(depth_frame);

	MadgwickAHRSupdateIMU(gyro_data.x, gyro_data.y, gyro_data.z, accel_data.x, accel_data.y, accel_data.z);


        memcpy(depth_image_ptr->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image_ptr->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        rgbd_curr.Upload(*depth_image_ptr, *color_image_ptr);

        q = Eigen::Quaterniond(q0, q1, q2, q3);

        if (i > 0 ) {
            Eigen::Quaterniond diff = q * last_q.inverse();

            Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Translation3d(Eigen::Vector3d(0,0,0)) * diff.normalized().toRotationMatrix();

            odometry.transform_source_to_target_ = t.matrix();
            //odometry.transform_source_to_target_ = Eigen::Matrix4d::Identity();

            odometry.Initialize(rgbd_curr, rgbd_prev);

            auto result = odometry.ComputeMultiScale();
            if (std::get<0>(result)) {
                WriteLossesToLog(fout, i, std::get<2>(result));
            }

            //std::cout << t.matrix() << std::endl;
            //std::cout << odometry.transform_source_to_target_ << std::endl;

            target_to_world = target_to_world * odometry.transform_source_to_target_;

            last_q = q;
        }

        extrinsics.FromEigen(target_to_world);
        tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);

        //if (i > 0 && i % 30 == 0) {
        //    tsdf_volume.GetAllSubvolumes();
        //    mesher.MarchingCubes(tsdf_volume);
        //    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", 
        //                *mesher.mesh().Download());
        //    //tsdf_volume.Reset();
        //    save_index++;
        //}

        rgbd_prev.CopyFrom(rgbd_curr);
        timer.Signal();
    }

    tsdf_volume.GetAllSubvolumes();
    mesher.MarchingCubes(tsdf_volume);

    WriteTriangleMeshToPLY("fragment-" + std::to_string(save_index) + ".ply", *mesher.mesh().Download());

    return EXIT_SUCCESS;

} catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
