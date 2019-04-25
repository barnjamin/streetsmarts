//
// Created by wei on 2/5/19.
//

#include <Eigen/Eigen>
#include <Open3D/Open3D.h>
#include <Cuda/Open3DCuda.h>
#include "config.h"
#include "utils.h"

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

    registration::PoseGraph pose_graph_cuda;
    io::ReadPoseGraph(argv[1], pose_graph_cuda);
    auto pg_viz = VisualizePoseGraph(pose_graph_cuda);

    auto bbox = LineSetFromBBox(pg_viz->GetMinBound(), pg_viz->GetMaxBound());

    visualization::DrawGeometries({pg_viz, bbox});

    //registration::PoseGraph pose_graph_cuda;
    //io::ReadPoseGraph(config.PoseFileScene(), pose_graph_cuda);

    //auto pose_graph_vis_cuda = VisualizePoseGraph(pose_graph_cuda);
    //auto mesh = io::CreateMeshFromFile(config.SceneMeshFile());

    return 0;
}
