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

    open3d::camera::PinholeCameraIntrinsic intrinsic = get_open3d_intrinsic(profile);
    open3d::io::WriteIJsonConvertible(conf.IntrinsicFile(), intrinsic);

    //Discard first $framestart frames
    for(int i=0; i<conf.framestart; i++) q.wait_for_frame(); 

    set_stereo_autoexposure(conf.stereo_autoexposure);
    set_stereo_whitebalance(conf.stereo_whitebalance);

    set_rgb_autoexposure(conf.rgb_autoexposure);
    set_rgb_whitebalance(conf.rgb_whitebalance);

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

        write_depth.join();
        write_color.join();
    }
}

void make_posegraph(Config conf, rs2::pipeline_profile profile, 
                        rs2::frame_queue q, std::queue<int> &pg_queue, std::mutex& mtx) {
    using namespace open3d;
    using namespace open3d::cuda;

    rs2::frameset frameset;
    rs2::frame color_frame, depth_frame;
    rs2::align align(conf.aligner);


    camera::PinholeCameraIntrinsic intrinsic = get_open3d_intrinsic(profile);
    io::WriteIJsonConvertible(conf.IntrinsicFile(), intrinsic);

    PinholeCameraIntrinsicCuda cuda_intrinsic(intrinsic);

    auto depth_image = std::make_shared<geometry::Image>();
    auto color_image = std::make_shared<geometry::Image>();

    depth_image->PrepareImage(conf.width, conf.height, 1, 2);
    color_image->PrepareImage(conf.width, conf.height, 3, 1);

    RGBDOdometryCuda<3> odometry;
    odometry.SetIntrinsics(intrinsic);
    odometry.SetParameters(odometry::OdometryOption({20, 10, 5}, 
                conf.max_depth_diff, conf.min_depth, conf.max_depth), 0.5f);

    RGBDImageCuda rgbd_target(conf.width, conf.height, conf.max_depth, conf.depth_factor);
    RGBDImageCuda rgbd_source(conf.width, conf.height, conf.max_depth, conf.depth_factor);

    bool success;
    Eigen::Matrix4d mat;
    std::vector<std::vector<float>> losses;

    //Discard first $framestart frames, letting auto exposure work itself out
    for(int i=0; i<conf.framestart; i++) q.wait_for_frame(); 
    conf.SetExposure();

    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    registration::PoseGraph pose_graph;
    pose_graph.nodes_.emplace_back(registration::PoseGraphNode(trans_odometry));

    for(int frame_idx = 0; frame_idx < conf.frames; ++frame_idx)
    {

        rs2::frame frame = q.wait_for_frame();

        //Get processed aligned frame
        frameset = align.process(frame.as<rs2::frameset>());
        color_frame = frameset.first(RS2_STREAM_COLOR);
        depth_frame = frameset.get_depth_frame();	       
        if (!depth_frame || !color_frame) { continue; }

        if(conf.use_filter){ depth_frame = conf.Filter(depth_frame); }

        memcpy(depth_image->data_.data(), depth_frame.get_data(), conf.width * conf.height * 2);
        memcpy(color_image->data_.data(), color_frame.get_data(), conf.width * conf.height * 3);

        // Upload images to GPU
        rgbd_target.Upload(*depth_image, *color_image);

        std::thread write_depth(io::WriteImage, conf.DepthFile(frame_idx), *depth_image, 100);
        std::thread write_color(io::WriteImage, conf.ColorFile(frame_idx), *color_image, 100);

        // Nothing to compare it to
        if(frame_idx == 0) { 
            rgbd_source.CopyFrom(rgbd_target); 
            write_depth.join();
            write_color.join();
            continue; 
        }

        odometry.transform_source_to_target_ =  Eigen::Matrix4d::Identity();
        odometry.Initialize(rgbd_source, rgbd_target);
        std::tie(success, mat, losses) = odometry.ComputeMultiScale();

        // No solution, skip it
        if(!success) {
            //TODO:: include this frame at all?
            // Write posegraph?
            rgbd_source.CopyFrom(rgbd_target); 
            write_depth.join();
            write_color.join();
            continue; 
        }

        Eigen::Matrix4d trans = odometry.transform_source_to_target_;
        Eigen::Matrix6d information = odometry.ComputeInformationMatrix();

        //Update Target to world
        trans_odometry =  trans * trans_odometry;

        Eigen::Matrix4d trans_odometry_inv = trans_odometry.inverse();

        //Update PoseGraph
        pose_graph.nodes_.emplace_back(registration::PoseGraphNode(trans_odometry_inv));
        pose_graph.edges_.emplace_back(registration::PoseGraphEdge(frame_idx - 1, frame_idx, 
                                        trans, information, false));

        //Overwrite previous
        rgbd_source.CopyFrom(rgbd_target);

        write_depth.join();
        write_color.join();

        //timer.Signal();
        conf.LogStatus("RGBDFRAME", frame_idx, 
                conf.frames_per_fragment * conf.fragments);
    }

    io::WritePoseGraph(conf.PoseFile(0), pose_graph);
    //pg_queue.push(fragment_idx);
}

void make_fragments(Config conf, std::queue<int> &pg_queue, 
        std::queue<int>& frag_queue, std::atomic<bool>& running, std::mutex& mtx) {

    while(running){
        while (!pg_queue.empty()) {
            int idx = pg_queue.front();

            //OptimizePoseGraphForFragment(idx, conf);

            //IntegrateForFragment(idx, conf);

            frag_queue.push(idx);

            pg_queue.pop();

            conf.LogStatus("FRAGMENT", idx, conf.fragments);
        }

        //Sleep for a bit 
        usleep(1000);
    }
}

