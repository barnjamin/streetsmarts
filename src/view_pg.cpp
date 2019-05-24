#include <Eigen/Eigen>
#include <Open3D/Open3D.h>
#include <Open3D/Cuda/Open3DCuda.h>
#include "config.h"
#include "utils.h"
#include <Open3D/Registration/GlobalOptimization.h>
#include <Open3D/Registration/GlobalOptimizationMethod.h>

using namespace open3d;
using namespace open3d::io;
using namespace open3d::utility;
using namespace open3d::geometry;
using namespace open3d::registration;

std::shared_ptr<LineSet> VisualizePoseGraph(PoseGraph &pose_graph) {
    std::shared_ptr<LineSet> pose_graph_vis =
        std::make_shared<LineSet>();

    int cnt = 0;

    int a = 0;
    const int kPointsPerFrustum = 5;
    const int kEdgesPerFrustum = 8;
    for (auto &node : pose_graph.nodes_) {
        auto pose = node.pose_;

        double norm = 0.1;
        Eigen::Vector4d ph;

        ph = pose * Eigen::Vector4d(0, 0, 0, 1);
        pose_graph_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(1, 1, 2, 1/norm));
        pose_graph_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(1, -1, 2, 1/norm));
        pose_graph_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(-1, -1, 2, 1/norm));
        pose_graph_vis->points_.emplace_back(ph.hnormalized());
        ph = pose * (norm * Eigen::Vector4d(-1, 1, 2, 1/norm));
        pose_graph_vis->points_.emplace_back(ph.hnormalized());

        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 1));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 2));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 3));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 0, cnt + 4));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 1, cnt + 2));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 2, cnt + 3));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 3, cnt + 4));
        pose_graph_vis->lines_.emplace_back(Eigen::Vector2i(cnt + 4, cnt + 1));

        for (int k = 0; k < kEdgesPerFrustum; ++k) {
            pose_graph_vis->colors_.emplace_back(Eigen::Vector3d(1, 0, 0));
        }

        cnt += kPointsPerFrustum;
    }

    for (auto &edge : pose_graph.edges_) {
        int s = edge.source_node_id_;
        int t = edge.target_node_id_;

        if (edge.uncertain_) {
            pose_graph_vis->lines_.emplace_back(
                Eigen::Vector2i(s * kPointsPerFrustum, t * kPointsPerFrustum));
            pose_graph_vis->colors_.emplace_back(Eigen::Vector3d(0, 1, 0));
        } else {
            pose_graph_vis->lines_.emplace_back(
                Eigen::Vector2i(s * kPointsPerFrustum, t * kPointsPerFrustum));
            pose_graph_vis->colors_.emplace_back(Eigen::Vector3d(0, 0, 1));
        }
    }

    return pose_graph_vis;
}




int main(int argc, char **argv) {
    SetVerbosityLevel(utility::VerbosityLevel::VerboseDebug);

    PoseGraph pose_graph;
    io::ReadPoseGraph(argv[1], pose_graph);

    // Read in first fragment
    // get transform to make it flat
    // apply transform to all components of pose graph, using running transform

    auto pcd = CreatePointCloudFromFile("/home/ben/apr13/test-1555162200/fragment/00000.pcd");

    //Eigen::Matrix4d trans = Flatten(*pcd);
    //std::cout << "Trans: " << trans << std::endl;


    //
    //int max_iteration = 100;
    //double min_relative_increment = 1e-6;
    //double min_relative_residual_increment = 1e-6;
    //double min_right_term = 1e-6;
    //double min_residual = 1e-6;
    //int max_iteration_lm = 20;
    //double upper_scale_factor = 2. / 3.;
    //double lower_scale_factor = 1. / 3.;
    //GlobalOptimizationConvergenceCriteria criteria(max_iteration, min_relative_increment, min_relative_residual_increment, 
    //        min_right_term, min_residual, max_iteration_lm, upper_scale_factor, lower_scale_factor);

    //double max_correspondence_distance = 0.075;
    //double edge_prune_threshold = 0.25;
    //double preference_loop_closure = 0.1;
    //int reference_node = 0;
    //GlobalOptimizationOption option(max_correspondence_distance, edge_prune_threshold, preference_loop_closure, reference_node);

    //GlobalOptimizationLevenbergMarquardt optimization_method;

    //GlobalOptimization(pose_graph, optimization_method, criteria, option);

    //auto pose_graph_pruned = CreatePoseGraphWithoutInvalidEdges(pose_graph, option);


    //double avg;
    //for(int x=0; x<pose_graph.edges_.size(); x++){
    //    pose_graph.edges_[x].transformation_ *= trans; 
    //    //pose_graph.edges_[x].transformation_(1,3) = 0.0;
    //    pose_graph.edges_[x].transformation_(1,3) -= 0.015;
    //}
    ////std::cout << "avg: " << avg / (double) pose_graph.edges_.size() << std::endl;
    //
    //for(int x=0; x<pose_graph.nodes_.size(); x++){
    //    Eigen::Matrix4d p = pose_graph.nodes_[x].pose_.inverse() * trans;
    //    p(1,3) += 0.015 * (double) x;
    //    pose_graph.nodes_[x].pose_.inverse();
    //}
    // auto pg_viz = VisualizePoseGraph(*pose_graph_pruned);
    
    auto pg_viz = VisualizePoseGraph(pose_graph);
    auto bbox = LineSetFromBBox(pg_viz->GetMinBound(), pg_viz->GetMaxBound());
    visualization::DrawGeometries({pg_viz, bbox});

    return 0;
}
