////Optimize Pose Graph and Combine or stream chunks
//#include <iostream>
//#include <memory>
//#include <Eigen/Dense>
//
//#include <Core/Core.h>
//#include <IO/IO.h>
//#include <Visualization/Visualization.h>
//#include <Cuda/Registration/RegistrationCuda.h>
//#include <Cuda/Geometry/NNCuda.h>
//#include <Cuda/Registration/FastGlobalRegistrationCuda.h>
//#include <Core/Registration/PoseGraph.h>
//#include <Core/Registration/GlobalOptimization.h>
//#include "utils.h"
//
int main(int argc, char *argv[])
{
//    using namespace open3d;
//
//    PoseGraph pose_graph;
//    ReadPoseGraph("/home/ben/streetsmarts/build/pose_graph.json", pose_graph);
//
//    std::vector<std::shared_ptr<PointCloud>> pcds;
//
//    std::shared_ptr<PointCloud> source_origin;
//    std::shared_ptr<PointCloud> source, target;
//    for (size_t i = 0; i < trajectory.parameters_.size(); i++) {
//
//        char buff[DEFAULT_IO_BUFFER_SIZE];
//        sprintf(buff, "/home/ben/streetsmarts/build/fragment-%d.ply", (int)i);
//
//        if (!filesystem::FileExists(buff)) {
//            continue;
//        }
//
//
//        source_origin = CreatePointCloudFromFile(buff);
//        source_origin->Transform(trajectory.parameters_[i].extrinsic_);
//        open3d::EstimateNormals(*source_origin, open3d::KDTreeSearchParamHybrid(0.1, 30));
//        std::shared_ptr<PointCloud> source_down = open3d::VoxelDownSample(*source_origin, 0.05);
//
//        auto stat_outliers = RemoveStatisticalOutliers(*source_down, 500, 0.05);
//        std::shared_ptr<PointCloud> source =  std::get<0>(stat_outliers);
//
//        std::cout << "fragment-"<< i << " has: " << source->points_.size() << std::endl;
//
//        if(i==0){
//            pcd = source_origin;
//            target = source;
//            continue;
//        }
//
//        int max_iter = 3;
//        open3d::cuda::RegistrationResultCuda result;
//
//        open3d::cuda::RegistrationCuda registration(TransformationEstimationType::PointToPlane);
//        registration.Initialize(*source, *target, 0.07f);
//        for (int iter = 0; iter < max_iter; ++iter) {
//            result = registration.DoSingleIteration(i);
//        }
//
//        auto odom = (trajectory.parameters_[i].extrinsic_ * result.transformation_);
//        pg.nodes_.push_back(open3d::PoseGraphNode(odom.inverse()));
//        pg.edges_.push_back(open3d::PoseGraphEdge(i-1, i, result.transformation_));
//
//        //std::cout <<std::endl << trajectory.parameters_[i].extrinsic_ <<std::endl << result.transformation_ << std::endl << std::endl;
//
//        //source->Transform(result.transformation_);
//
//        //extrinsics.FromEigen(target_to_world);
//        //tsdf_volume.Integrate(rgbd_curr, cuda_intrinsics, extrinsics);
//
//        source_origin->Transform(result.transformation_);
//        *pcd += *source_origin;
//
//        pcds.push_back(source);
//
//        target = source;
//    }
//
//
//    ////char buff[DEFAULT_IO_BUFFER_SIZE];
//    ////sprintf(buff, "/home/ben/streetsmarts/build/fragment-9.ply");
//    ////auto source_origin_init = CreatePointCloudFromFile(buff);
//    ////open3d::EstimateNormals(*source_origin_init, open3d::KDTreeSearchParamHybrid(0.1, 30));
//    ////std::shared_ptr<PointCloud> source_init_down = open3d::VoxelDownSample(*source_origin_init, 0.01);
//    ////auto source_stat_outliers = RemoveStatisticalOutliers(*source_init_down, 500, 0.05);
//    ////std::shared_ptr<PointCloud> source_init =  std::get<0>(source_stat_outliers);
//
//    ////auto target_origin_init = CreatePointCloudFromFile("/home/ben/streetsmarts/build/fragment-0.ply");
//    ////open3d::EstimateNormals(*target_origin_init, open3d::KDTreeSearchParamHybrid(0.1, 30));
//    ////std::shared_ptr<PointCloud> target_init_down = open3d::VoxelDownSample(*target_origin_init, 0.01);
//    ////auto target_stat_outliers = RemoveStatisticalOutliers(*target_init_down, 500, 0.05);
//    ////std::shared_ptr<PointCloud> target_init =  std::get<0>(target_stat_outliers);
//
//
//    ////open3d::cuda::RegistrationResultCuda result;
//    ////open3d::cuda::FastGlobalRegistrationCuda fgr;
//    ////fgr.Initialize(*source_init, *target_init);
//    ////for (int iter = 0; iter < 256; ++iter) {
//    ////    result = fgr.DoSingleIteration(iter);
//    ////    //VisualizeRegistration(*source_init, *target_init, result.transformation_);
//    ////}
//
//    ////pg.edges_.push_back(open3d::PoseGraphEdge(9, 0, result.transformation_, Eigen::Matrix6d::Identity(), true));
//
//    ////WritePoseGraph("pose_graph_registered_before_optimize.json", pg);
//
//    ////open3d::GlobalOptimizationConvergenceCriteria criteria;
//    ////open3d::GlobalOptimizationOption option;
//    ////option.max_correspondence_distance_ =  1.5 * 1.4;
//    //////option.edge_prune_threshold_ = 0.5;
//    ////option.preference_loop_closure_ = 5.0 ;
//    ////option.reference_node_ = 0;
//    ////open3d::GlobalOptimizationLevenbergMarquardt optimization_method;
//    ////open3d::GlobalOptimization(pg, optimization_method, criteria, option);
//
//    //for(size_t i=0; i<pcds.size(); i++){
//    //    pcds[i]->Transform(pg.nodes_[i].pose_);
//    //    if(i==0){
//    //        pcd = pcds[i];
//    //    }else{
//    //        *pcd += *pcds[i];
//    //    }
//    //}
//
//    //WritePoseGraph("pose_graph_registered.json", pg);
//
//    //std::shared_ptr<PointCloud> final_pcd = open3d::VoxelDownSample(*pcd, 0.01);
//    //WritePointCloud("final.pcd", *final_pcd);
//
//    ////auto stat_outliers = RemoveStatisticalOutliers(*final_pcd, 300, 0.9);
//    ////WritePointCloud("final.pcd", *(std::get<0>(stat_outliers)));
//
    return 0;
}
