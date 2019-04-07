#include <vector>
#include <string>
#include <Open3D/Open3D.h>
#include <Cuda/Open3DCuda.h>

#include <Open3D/Registration/GlobalOptimization.h>
#include <Open3D/Registration/GlobalOptimizationMethod.h>

#include "utils.h"
#include "config.h"

using namespace open3d;
using namespace open3d::cuda;
using namespace open3d::utility;
using namespace open3d::geometry;
using namespace open3d::registration;
using namespace open3d::integration;
using namespace open3d::io;

struct Match {
    bool success;
    int s;
    int t;
    Eigen::Matrix4d trans_source_to_target;
    Eigen::Matrix6d information;
};

void refine_fragments_streaming(Config config, std::queue<int> &frag_queue, std::atomic<bool>& running) {
    std::vector<Match> matches;
    std::vector<std::shared_ptr<PointCloud>> fragments;
    int s_idx = 0;
    while(running && !(fragments.empty())){
        if(frag_queue.empty()){
            usleep(1000);
            continue;
        }

        auto pcd_raw = CreatePointCloudFromFile(config.FragmentFile(frag_queue.front()));
        auto pcd = VoxelDownSample(*pcd_raw,config.voxel_size);

        frag_queue.pop();

        fragments.emplace_back(pcd);

        //If we dont ahve enough to compare and the program is stil running, continue
        if(fragments.size()<config.registration_window_size && !running){
            continue;
        }


        PoseGraph pose_graph_s;
        ReadPoseGraph(config.PoseFile(s_idx), pose_graph_s);

        Eigen::Matrix4d init_source_to_target = pose_graph_s.nodes_.back().pose_.inverse(); 

        for (int t_idx = s_idx+1; t_idx < fragments.size(); t_idx++) {
            auto source = fragments[s_idx];
            auto target = fragments[t_idx];

            Match match;
            match.s = s_idx;
            match.t = t_idx;

            if(t_idx == s_idx+1){ /** Colored ICP **/
                PrintInfo("ColoredICP");
                RegistrationCuda registration(TransformationEstimationType::ColoredICP);
                registration.Initialize(*source, *target, (float) config.voxel_size * 1.4f, init_source_to_target);
                registration.ComputeICP();

                match.trans_source_to_target = registration.transform_source_to_target_;

                match.information = registration.ComputeInformationMatrix();
                match.success = true;
            } else {
                PrintInfo("FGR");
                FastGlobalRegistrationCuda fgr;
                fgr.Initialize(*source, *target);

                auto result = fgr.ComputeRegistration();
                match.trans_source_to_target = result.transformation_;

                match.information = RegistrationCuda::ComputeInformationMatrix(
                    *source, *target, config.voxel_size * 1.4f, result.transformation_);

                match.success = match.trans_source_to_target.trace() != 4.0 && match.information(5, 5) / 
                        std::min(source->points_.size(), target->points_.size()) >= 0.3;
            }

            matches.push_back(match);
        }

        s_idx++;
    }

    PoseGraph pose_graph;

    /* world_to_frag_0 */
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry));

    for (auto &match : matches) {
        if (!match.success) continue;
        if (match.t == match.s + 1) {
            /* world_to_frag_i */
            trans_odometry = match.trans_source_to_target * trans_odometry;
            auto trans_odometry_inv = trans_odometry.inverse();

            pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            pose_graph.edges_.emplace_back(PoseGraphEdge(
                match.s, match.t,
                match.trans_source_to_target, match.information,
                false));
        } else {
            pose_graph.edges_.emplace_back(PoseGraphEdge(
                match.s, match.t,
                match.trans_source_to_target, match.information,
                true));
        }
    }

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option( config.voxel_size * 1.4, 0.25, config.loop_close_reg, 0);
    GlobalOptimizationLevenbergMarquardt optimization_method;
    GlobalOptimization(pose_graph, optimization_method, criteria, option);

    auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges(pose_graph, option);

    WritePoseGraph(config.PoseFileScene(), *pose_graph_prunned);
}


std::tuple<Eigen::Matrix4d, Eigen::Matrix6d>
    MultiScaleICP(const PointCloud &source, const PointCloud& target,
                    const Eigen::Matrix4d &init_trans,
                    const float voxel_size,
                    const std::vector<int> &iters = {50, 30, 14},
                    const std::vector<float> &voxel_factors = {1.0, 2.0, 4.0}) {

    assert(iters.size() == voxel_factors.size());

    Eigen::Matrix4d transformation = init_trans;
    Eigen::Matrix6d information = Eigen::Matrix6d::Identity();

    for (int i = 0; i < iters.size(); i++) {
        float voxel_size_level = voxel_size / voxel_factors[i];
        auto source_down = VoxelDownSample(source, voxel_size_level);
        auto target_down = VoxelDownSample(target, voxel_size_level);

        RegistrationCuda registration(TransformationEstimationType::ColoredICP);
        registration.Initialize(*source_down, *target_down, voxel_size_level * 1.4f, transformation);
        registration.ComputeICP(iters[i]);

        transformation = registration.transform_source_to_target_;

        if (i == iters.size() - 1) {
            information = registration.ComputeInformationMatrix();
        }
    }

    return std::make_tuple(transformation, information);
}

void RefineFragments(Config &config) {
    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFileScene(), pose_graph);

    std::vector<Match> matches;

    for (auto &edge : pose_graph.edges_) {
        Match match;
        match.s = edge.source_node_id_;
        match.t = edge.target_node_id_;
        match.success = true;

        auto source = CreatePointCloudFromFile(config.FragmentFile(match.s));
        auto target = CreatePointCloudFromFile(config.FragmentFile(match.t));

        std::tie(match.trans_source_to_target, match.information) = MultiScaleICP(*source, *target, edge.transformation_, config.voxel_size);

        PrintInfo("Point cloud odometry (%d %d)\n", match.s, match.t);

        matches.push_back(match);
    }

    PoseGraph pose_graph_matched;

    /* world_to_frag0 */
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    pose_graph_matched.nodes_.emplace_back(PoseGraphNode(trans_odometry));

    for (auto &match : matches) {
        if (!match.success) continue;
        if (match.t == match.s + 1) {
            /* world_to_fragi */
            trans_odometry = match.trans_source_to_target * trans_odometry;
            auto trans_odometry_inv = trans_odometry.inverse();

            pose_graph_matched.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            pose_graph_matched.edges_.emplace_back(PoseGraphEdge( match.s, match.t, match.trans_source_to_target, match.information, false));
        } else {
            pose_graph_matched.edges_.emplace_back(PoseGraphEdge( match.s, match.t, match.trans_source_to_target, match.information, true));
        }
    }

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option( config.voxel_size * 1.4, 0.25, config.loop_close_reg, 0);
    GlobalOptimizationLevenbergMarquardt optimization_method;
    GlobalOptimization(pose_graph_matched, optimization_method, criteria, option);

    auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges(pose_graph_matched, option);

    WritePoseGraph(config.PoseFileScene(), *pose_graph_prunned);
}
