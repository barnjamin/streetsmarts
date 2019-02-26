//
// Created by wei on 2/4/19.
//

#include <vector>
#include <string>
#include <Core/Core.h>
#include <IO/IO.h>

#include <Cuda/Registration/RegistrationCuda.h>
#include <Cuda/Registration/ColoredICPCuda.h>
#include <Cuda/Registration/FastGlobalRegistrationCuda.h>

#include <Core/Registration/PoseGraph.h>
#include <Core/Registration/GlobalOptimization.h>

#include "config.h"

using namespace open3d;

struct Match {
    bool success;
    int s;
    int t;
    Eigen::Matrix4d trans_source_to_target;
    Eigen::Matrix6d information;
};


std::tuple<int, int> GetWindow(int idx, int len, int size) {
    return std::make_tuple(idx+1, std::min(len,idx+size));
}


std::vector<Match> RegisterFragments(Config &config) {

    std::vector<Match> matches;

    int start, stop;
    int frag_count = config.GetFragmentCount();
    for (int s = 0; s < frag_count; s++) {
        auto source = CreatePointCloudFromFile(config.FragmentFile(s));

        PoseGraph pose_graph_s;
        ReadPoseGraph(config.PoseFile(s), pose_graph_s);

        Eigen::Matrix4d init_source_to_target = pose_graph_s.nodes_[0].pose_.inverse(); 

        std::tie(start, stop) = GetWindow(s, frag_count, config.registration_window_size);

        for (int t = start; t < stop; ++t) {

            if (t == s) { continue; }

            auto target = CreatePointCloudFromFile(config.FragmentFile(t));

            Match match;
            match.s = s;
            match.t = t;

            /** Colored ICP **/
            cuda::RegistrationCuda registration(TransformationEstimationType::ColoredICP);

            registration.Initialize(*source, *target, (float) config.voxel_size * 1.4f, init_source_to_target);

            registration.ComputeICP();

            match.trans_source_to_target = registration.transform_source_to_target_;

            match.information = registration.ComputeInformationMatrix();

            match.success = true;

            PrintInfo("Point cloud odometry (%d %d)\n", match.s, match.t);

            matches.push_back(match);
        }
    }

    return matches;
}

void MakePoseGraphForRegisteredScene(const std::vector<Match> &matches, Config &config) {
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

    WritePoseGraph(config.PoseFileScene(), pose_graph);
}

void OptimizePoseGraphForRegisteredScene(Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFileScene(), pose_graph);

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option( config.voxel_size * 1.4, 0.25, config.preference_loop_closure_registration, 0);
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

    for (int i = 0; i < iters.size(); ++i) {
        float voxel_size_level = voxel_size / voxel_factors[i];
        auto source_down = VoxelDownSample(source, voxel_size_level);
        auto target_down = VoxelDownSample(target, voxel_size_level);

        cuda::RegistrationCuda registration(TransformationEstimationType::ColoredICP);
        registration.Initialize(*source_down, *target_down, voxel_size_level * 1.4f, transformation);
        registration.ComputeICP(iters[i]);

        transformation = registration.transform_source_to_target_;

        if (i == iters.size() - 1) {
            information = registration.ComputeInformationMatrix();
        }
    }

    return std::make_tuple(transformation, information);
}

std::vector<Match> RefineFragments(Config &config) {

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

    return matches;
}

void MakePoseGraphForRefinedScene(const std::vector<Match> &matches, Config &config) {
    PoseGraph pose_graph;

    /* world_to_frag0 */
    Eigen::Matrix4d trans_odometry = Eigen::Matrix4d::Identity();
    pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry));

    for (auto &match : matches) {
        if (!match.success) continue;
        if (match.t == match.s + 1) {
            /* world_to_fragi */
            trans_odometry = match.trans_source_to_target * trans_odometry;
            auto trans_odometry_inv = trans_odometry.inverse();

            pose_graph.nodes_.emplace_back(PoseGraphNode(trans_odometry_inv));
            pose_graph.edges_.emplace_back(PoseGraphEdge( match.s, match.t, match.trans_source_to_target, match.information, false));
        } else {
            pose_graph.edges_.emplace_back(PoseGraphEdge( match.s, match.t, match.trans_source_to_target, match.information, true));
        }
    }

    WritePoseGraph(config.PoseFileScene(), pose_graph);
}

void OptimizePoseGraphForRefinedScene(Config &config) {

    PoseGraph pose_graph;
    ReadPoseGraph(config.PoseFileScene(), pose_graph);

    GlobalOptimizationConvergenceCriteria criteria;
    GlobalOptimizationOption option( config.voxel_size * 1.4, 0.25, config.preference_loop_closure_registration, 0);
    GlobalOptimizationLevenbergMarquardt optimization_method;
    GlobalOptimization(pose_graph, optimization_method, criteria, option);

    auto pose_graph_prunned = CreatePoseGraphWithoutInvalidEdges(pose_graph, option);

    WritePoseGraph(config.PoseFileScene(), *pose_graph_prunned);
}

int main(int argc, char ** argv) 
{

    Config conf(argc, argv);

    PrintInfo("Registering fragments");
    //Register 
    auto registered_matches = RegisterFragments(conf);
    PrintInfo("Making Pose graph for fragments");
    MakePoseGraphForRegisteredScene(registered_matches, conf);
    PrintInfo("Optimizing Pose graph for fragments");
    OptimizePoseGraphForRegisteredScene(conf);

    ////Refine
    PrintInfo("Refining Pose Graph");
    auto refined_matches = RefineFragments(conf);
    MakePoseGraphForRefinedScene(refined_matches, conf);
    OptimizePoseGraphForRefinedScene(conf);

    return 0;
}
