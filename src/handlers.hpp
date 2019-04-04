#include <librealsense2/rs.hpp> 
#include <thread>
#include <atomic>
#include <Open3D/Open3D.h>
#include <Open3D/Registration/PoseGraph.h>
#include <Open3D/Registration/GlobalOptimization.h>
#include <Open3D/Utility/Timer.h>

#include <Cuda/Open3DCuda.h>
#include "config.h"
#include "utils.h"
#include "fragments.hpp"

void optimize_and_integrate(int idx, Config conf);

void record_imu(Config conf, std::atomic<bool>& running, rs2::frame_queue q) {
    std::ofstream imu_file;
    imu_file.open(conf.IMUFile());
    while(running){
        rs2::frame frame = q.wait_for_frame();
        auto stype = frame.get_profile().stream_type();
        auto mframe = frame.as<rs2::motion_frame>();
        auto vec = mframe.get_motion_data();
        auto ts = mframe.get_timestamp();

        std::string t;
        if(stype == RS2_STREAM_GYRO) t = "g";
        else t = "a";
        
        imu_file << t << "," << get_timestamp() << "," << ts 
            << "," << vec.x << "," << vec.y << "," << vec.z << std::endl;
    }
}

void record_img(Config conf, rs2::pipeline_profile profile, rs2::frame_queue q) {
    rs2::align align(conf.aligner);

    auto depth_image = std::make_shared<open3d::geometry::Image>();
    auto color_image = std::make_shared<open3d::geometry::Image>();
    auto infra_image = std::make_shared<open3d::geometry::Image>();

    depth_image->PrepareImage(conf.width, conf.height, 1, 2);
    color_image->PrepareImage(conf.width, conf.height, 3, 1);
    infra_image->PrepareImage(conf.width, conf.height, 1, 1);

    std::ofstream timestamp_file;
    timestamp_file.open(conf.ImageTimestampFile());

    open3d::camera::PinholeCameraIntrinsic intrinsic = get_intrinsics(profile);
    open3d::io::WriteIJsonConvertible(conf.IntrinsicFile(), intrinsic);

    //Discard first $framestart frames
    for(int i=0; i<conf.framestart; i++) q.wait_for_frame(); 

    rs2::frame frame;
    rs2::frameset fs;
    rs2::frame color_frame, depth_frame, infra_frame;
    for(int img_idx = 0; img_idx < conf.fragments * conf.frames_per_fragment; img_idx++) {
        frame = q.wait_for_frame();
        fs = align.process(frame.as<rs2::frameset>());

        color_frame = fs.first(RS2_STREAM_COLOR);
        depth_frame = fs.get_depth_frame();	       

        if (!depth_frame || !color_frame) { return; }

        if(conf.use_filter){ depth_frame = conf.Filter(depth_frame); }

        memcpy(depth_image->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        std::thread write_depth(open3d::io::WriteImage, conf.DepthFile(img_idx), *depth_image, 100);
        std::thread write_color(open3d::io::WriteImage, conf.ColorFile(img_idx), *color_image, 100);

        timestamp_file << img_idx << "," << depth_frame.get_timestamp() << "," 
            << color_frame.get_timestamp()  << "," << get_timestamp() << std::endl;

        write_depth.join();
        write_color.join();
    }
}


void make_fragments(Config conf, rs2::pipeline_profile profile, rs2::frame_queue q) {
    using namespace open3d;
    using namespace open3d::cuda;

    rs2::align align(conf.aligner);

    std::ofstream timestamp_file;
    timestamp_file.open(conf.ImageTimestampFile());

    camera::PinholeCameraIntrinsic intrinsic = get_intrinsics(profile);
    io::WriteIJsonConvertible(conf.IntrinsicFile(), intrinsic);

    PinholeCameraIntrinsicCuda cuda_intrinsic(intrinsic);

    utility::PrintInfo("Discarding first %d frames\n", conf.framestart);

    //Discard first $framestart frames
    for(int i=0; i<conf.framestart; i++) q.wait_for_frame(); 

    std::thread make_fragments;

    auto depth_image = std::make_shared<geometry::Image>();
    auto color_image = std::make_shared<geometry::Image>();

    depth_image->PrepareImage(conf.width, conf.height, 1, 2);
    color_image->PrepareImage(conf.width, conf.height, 3, 1);

    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;

    RGBDImageCuda rgbd_target(conf.width, conf.height, conf.max_depth, conf.depth_factor);
    RGBDImageCuda rgbd_source(conf.width, conf.height, conf.max_depth, conf.depth_factor);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);
    odometry.SetParameters(odometry::OdometryOption({20, 10, 5}, 
                conf.max_depth_diff, conf.min_depth, conf.max_depth), 0.5f);

    utility::FPSTimer timer("Process RGBD stream", conf.fragments*conf.frames_per_fragment);

    utility::PrintInfo("Starting to make fragments\n");
    for(int fragment_idx=0; fragment_idx<conf.fragments; fragment_idx++) 
    {
        utility::PrintInfo("Fragment %d\n", fragment_idx);

        Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
        registration::PoseGraph pose_graph;
        pose_graph.nodes_.emplace_back(registration::PoseGraphNode(trans_odometry));

        for(int i = 0; i < conf.frames_per_fragment; i++)
        {

            int frame_idx = (conf.frames_per_fragment * fragment_idx) + i;

            rs2::frame frame = q.wait_for_frame();

            //Get processed aligned frame
            frameset = align.process(frame.as<rs2::frameset>());

            color_frame = frameset.first(RS2_STREAM_COLOR);
            depth_frame = frameset.get_depth_frame();	       

            if (!depth_frame || !color_frame) { continue; }

            if(conf.use_filter){ depth_frame = conf.Filter(depth_frame); }

            memcpy(depth_image->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
            memcpy(color_image->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

            timestamp_file << frame_idx << "," << depth_frame.get_timestamp() << "," 
                << color_frame.get_timestamp()  << "," << get_timestamp() << std::endl;

            //Upload images to GPU
            rgbd_source.Upload(*depth_image, *color_image);

            std::thread write_depth(io::WriteImage, conf.DepthFile(frame_idx), *depth_image, 100);
            std::thread write_color(io::WriteImage, conf.ColorFile(frame_idx), *color_image, 100);

            if(i==0) { 
                rgbd_target.CopyFrom(rgbd_source); 
                write_depth.join();
                write_color.join();
                continue; 
            }

            //Reset Odometry transform
            odometry.transform_source_to_target_ =  Eigen::Matrix4d::Identity();

            //Initialize odometry with current and previous images
            odometry.Initialize(rgbd_source, rgbd_target);
            odometry.ComputeMultiScale();

            Eigen::Matrix4d trans = odometry.transform_source_to_target_;
            Eigen::Matrix6d information = odometry.ComputeInformationMatrix();

            //Update Target to world
            trans_odometry =  trans * trans_odometry;

            Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();

            //Update PoseGraph
            pose_graph.nodes_.emplace_back(registration::PoseGraphNode(trans_odometry_inv));
            pose_graph.edges_.emplace_back(registration::PoseGraphEdge(i - 1, i, 
                                            trans, information, false));

            //Overwrite previous
            rgbd_target.CopyFrom(rgbd_source);

            write_depth.join();
            write_color.join();

            timer.Signal();
        }
        io::WritePoseGraph(conf.PoseFile(fragment_idx), pose_graph);

        //Make sure we've finished the last one
        if(fragment_idx > 0){
            utility::PrintInfo("Waiting for: %d\n", fragment_idx-1);
            make_fragments.join();
            utility::PrintInfo("Finished waiting for: %d\n", fragment_idx-1);
        }

        //Kick off thread to optimize pg and create fragment
        make_fragments = std::thread(optimize_and_integrate, fragment_idx, conf);
    }

    //Wait for the last one before returning
    make_fragments.join();
}

void optimize_and_integrate(int idx, Config conf) {
    utility::PrintInfo("Starting thread for: %d\n", idx);
    OptimizePoseGraphForFragment(idx, conf);
    IntegrateForFragment(idx, conf);
    utility::PrintInfo("Finishing thread for: %d\n", idx);
}
